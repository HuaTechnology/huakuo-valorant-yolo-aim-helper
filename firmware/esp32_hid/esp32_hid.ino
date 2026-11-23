// =============================================================================
//  esp32_hid.ino — serial binary frames -> native USB HID mouse
//  Target platform: ESP32-S3 / ESP32-S2, Arduino-ESP32 core 3.x, TinyUSB native HID
//
//  Data flow (fully non-blocking, no queue buildup):
//
//    host aim2 (Python)                  ESP32
//    ┌────────────────┐   921600 8N1   ┌───────────────────────────────────────────────────┐
//    │ SerialLink     │ ─ USB CDC ───► │ rxTask     : byte stream -> v2 frames -> dispatch │
//    │ StreamParser   │                │   MOVE     -> accumulator (int32)                 │
//    │ MotionPlanner  │                │   BUTTON   -> target button state + task notify   │
//    │ decode_status  │ ◄── STATUS ─── │   PING     -> feeds the watchdog                  │
//    └────────────────┘                │ reportTask : 1000 Hz, accumulator -> HID          │
//                                      │ statusTask : 100 ms telemetry                     │
//                                      └───────────────────────────────────────────────────┘
//
//  Three hard constraints (read these before changing anything):
//
//   1) Mouse.* is only called from reportTask. The RX side never touches HID directly: USB
//      reports are a shared resource, and interleaved calls from two tasks scramble report
//      order (cursor jitter, swallowed buttons). MOVE frames only accumulate - no HID on RX.
//
//   2) Displacement never loses its remainder. One HID report is capped at +/-127 per axis;
//      the excess stays in the accumulator for later reports - never truncated away, otherwise
//      the sub-pixel error near the target never converges to 0 and a deadzone forms.
//
//   3) deadman 300 ms. Every valid frame feeds the watchdog; on timeout the button releases and
//      output stops, so a host crash, an unplugged cable or a stuck inference never leaves it held.
//
//  Wire protocol v2 (byte-for-byte identical to src/dma/transport/protocol.py):
//
//      off 0 : 0xA5                                     magic
//      off 1 : (2 << 4) | type                          version | type
//      off 2 : seq                                      sequence number, wraps
//      off 3 : len                                      payload length
//      off 4 : payload[len]
//      last  : CRC-8/ATM(poly 0x07, init 0, MSB-first) over [0, 4+len)
//
//      1 MOVE   int16 LE dx, int16 LE dy
//      2 BUTTON uint8 mask (bit0 = left button)
//      3 PING   uint32 LE counter
//      4 STATUS device -> host, struct "<IHHHHB", 13 bytes (see the STATUS notes)
//      5 CONFIG reserved: CRC-checked then ignored, counted as unknown
//
//  The parser matches the host StreamParser: no 0xA5 / len > 128 / version mismatch /
//  bad CRC -> drop 1 byte and keep scanning; the buffer is only cleared when no magic is left.
// =============================================================================

#include <Arduino.h>
#include <USB.h>
#include <USBHIDMouse.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// TinyUSB HID endpoint readiness query. The core ships no tusb.h on the sketch include path, but
// the symbol is already linked into the firmware (USBHIDMouse relies on it), so a direct
// declaration is enough: tud_hid_n_ready(0) == true means "a host is polling HID interface 0 and
// the endpoint is writable". This is the only reliable signal for "is the host really attached".
extern "C" bool tud_hid_n_ready(uint8_t instance);

// Optional: if the framework ships a TinyUSB header, report the real mount state via tud_mounted().
#if defined(__has_include)
#  if __has_include("tusb.h")
#    include "tusb.h"
#    define FW_HAS_TINYUSB 1
#  endif
#endif
#ifndef FW_HAS_TINYUSB
#  define FW_HAS_TINYUSB 0
#endif

// Compile-time self-check: Hardware CDC and JTAG mode never initializes TinyUSB, so HID never enumerates.
#if defined(ARDUINO_USB_MODE) && (ARDUINO_USB_MODE == 1)
#  warning "ARDUINO_USB_MODE=1 (Hardware CDC and JTAG) disables TinyUSB; USBHIDMouse will not enumerate - set ARDUINO_USB_MODE=0"
#endif

// =============================================================================
// 1. Compile-time constants (every tunable lives here; none scattered into function bodies)
// =============================================================================
namespace fw {

// ---- serial -------------------------------------------------------------
constexpr uint32_t SERIAL_BAUD      = 921600;  // matches the OutputCfg.baud default (ignored under CDC)
constexpr int      SERIAL_RX_BUFSZ  = 2048;    // CDC RX ring buffer: 921600 bps ~ 92 kB/s -> 22 ms of headroom
constexpr uint32_t RX_POLL_MS       = 1;       // rxTask idle poll period

// ---- HID output ---------------------------------------------------------
constexpr uint32_t REPORT_HZ        = 1000;    // report task period
constexpr int32_t  MAX_PER_REPORT   = 127;     // per-report limit of one HID axis (int8)
constexpr int32_t  AXIS_CAP         = 4 * MAX_PER_REPORT;  // accumulator cap 508, same value as the host planner
constexpr uint32_t DEADMAN_MS       = 300;     // no valid frame for this long -> release button + stop output
constexpr uint32_t SPIN_ALIGN_US    = 250;     // spin cap for period alignment (microseconds)

// ---- protocol -----------------------------------------------------------
constexpr uint8_t  PROTO_MAGIC      = 0xA5;
constexpr uint8_t  PROTO_VERSION    = 2;
constexpr uint8_t  T_MOVE           = 1;
constexpr uint8_t  T_BUTTON         = 2;
constexpr uint8_t  T_PING           = 3;
constexpr uint8_t  T_STATUS         = 4;
constexpr uint8_t  T_CONFIG         = 5;
constexpr uint16_t MAX_PAYLOAD      = 128;     // same value as the host: len > 128 means desync
constexpr uint16_t FRAME_HDR        = 4;       // magic + ver|type + seq + len
constexpr uint16_t FRAME_MAX        = FRAME_HDR + MAX_PAYLOAD + 1;  // 133
constexpr uint16_t PARSER_BUF       = 168;     // > FRAME_MAX, leaves room for several coalesced frames
constexpr uint32_t CRC_STORM_LIMIT  = 32;      // 32 consecutive CRC errors -> clear the accumulator
constexpr uint8_t  BTN_LEFT         = 0x01;

// ---- telemetry ----------------------------------------------------------
// The host protocol.decode_status unpacks 13 bytes with struct "<IHHHHB":
//   uptime u32 | rx u16 | crc u16 | drops u16 | report_hz u16 | flags u8
//                        4        2        2        2            2         1   = 13
// Exactly matches host protocol.STATUS_LEN == 13; report_hz uses a full u16, so 1000 Hz is reported as-is.
// (Legacy: early hosts used "<IHHHBB" with 12 bytes / hz as u8; set STATUS_HZ_AS_U8 to true to fall back,
//  only for talking to old firmware or old hosts - normal builds keep it false.)
constexpr bool     STATUS_HZ_AS_U8  = false;
constexpr uint16_t STATUS_LEN_MAX   = 13;                       // larger of the two layouts
constexpr uint16_t STATUS_LEN       = STATUS_HZ_AS_U8 ? 12 : 13;
constexpr uint32_t STATUS_PERIOD_MS = 100;     // proactive report period
constexpr uint8_t  FLAG_DEADMAN     = 0x01;    // flags bit0
constexpr uint8_t  FLAG_USB_READY   = 0x02;    // flags bit1
constexpr uint8_t  FLAG_HID_STALLED = 0x04;    // flags bit2: motion pending but nothing delivered

// ---- task layout --------------------------------------------------------
// ESP32-S3 is dual core: core0 runs USB/TinyUSB + parsing, the report task is pinned to core1 (less jitter).
constexpr BaseType_t  CORE_USB       = 0;
constexpr BaseType_t  CORE_REPORT    = 1;
constexpr UBaseType_t PRIO_PARSER    = 8;
constexpr UBaseType_t PRIO_REPORT    = 20;     // above Arduino loopTask(1) and the USB tasks
constexpr UBaseType_t PRIO_STATUS    = 3;
constexpr uint32_t    STACK_PARSER   = 4096;
constexpr uint32_t    STACK_REPORT   = 3072;
constexpr uint32_t    STACK_STATUS   = 3072;

// ---- debug text ---------------------------------------------------------
// The boot banner goes out on the same CDC and is plain ASCII with no 0xA5; the host parser drops
// magic-less bytes without counting a CRC error, so telemetry stays clean. Set 0 for production.
constexpr bool     FW_BANNER        = true;

}  // namespace fw

using namespace fw;

// =============================================================================
// 2. Global device object (the only instance in the file)
// =============================================================================
USBHIDMouse Mouse;   // core 3.x native USB HID mouse (the HID interface of the composite device)

// =============================================================================
// 3. Forward declarations (the builder derives prototypes from .ino; declaring them keeps order clear)
// =============================================================================
static void        rx_task(void *arg);
static void        report_task(void *arg);
static void        status_task(void *arg);
static inline uint8_t crc8_atm(const uint8_t *data, uint16_t n);
static void        parser_feed(uint8_t byte);
static void        parser_dispatch(uint8_t kind, const uint8_t *payload, uint16_t len);
static void        parser_drop_one();
static void        acc_add(int16_t dx, int16_t dy);
static void        acc_take(int32_t *out_x, int32_t *out_y);
static void        acc_clear(bool counted);
static void        buttons_set_target(uint8_t mask);
static void        buttons_apply();
static void        buttons_release_all();
static void        service_motion(int64_t now_us);
static void        send_status();
static void        send_frame(uint8_t kind, const uint8_t *payload, uint16_t len);
static bool        serial_up();
static bool        usb_mounted();

// =============================================================================
// 4. CRC-8/ATM
//    poly 0x07, init 0, no reflection, xorout 0, MSB-first -- identical to the host crc8().
//    Bitwise implementation (~8 cycles/byte): 92 kB/s worst case costs under 1% CPU and saves 256 B of table RAM.
// =============================================================================
static inline uint8_t crc8_atm(const uint8_t *data, uint16_t n) {
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < n; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// =============================================================================
// 5. Cross-task shared state
//    Writers and readers are one-to-one; aligned 32-bit access is atomic on Xtensa, so plain volatile
//    counters are enough. Only the accumulator is read-modify-write and must take a critical section.
// =============================================================================
static portMUX_TYPE g_acc_mux       = portMUX_INITIALIZER_UNLOCKED;
static int32_t      g_acc_x         = 0;   // pending displacement (sub-frame remainders live here)
static int32_t      g_acc_y         = 0;

static volatile uint8_t g_btn_target  = 0; // target button state written by RX (bit0 = left button)
static uint8_t          g_btn_applied = 0; // button state the report task has already pushed to HID (single writer)

static volatile int64_t g_last_rx_us  = 0; // timestamp of the latest valid frame (feeds the watchdog)
static volatile bool    g_deadman     = false;
static volatile bool    g_usb_ready   = false;

static volatile uint32_t g_rx_frames      = 0;  // valid frame count
static volatile uint32_t g_crc_errors     = 0;  // CRC/length/version error count (same definition as the host)
static volatile uint32_t g_queue_drops    = 0;  // dropped displacement units (overflow / CRC storm / deadman)
static volatile uint32_t g_crc_streak     = 0;  // consecutive CRC errors
static volatile uint32_t g_unknown_frames = 0;  // CONFIG and unknown types (visible in debug text only)
static volatile uint32_t g_ping_count     = 0;  // counter of the most recent PING
static volatile uint32_t g_hid_reports    = 0;  // HID reports sent so far (button changes included)
static volatile uint16_t g_report_hz      = 0;  // delivered report rate over the latest window
static volatile uint32_t g_hid_stalls      = 0;  // number of Mouse.move() calls that were not accepted
static uint8_t           g_tx_seq         = 0;  // device-side TX sequence number

static TaskHandle_t g_task_report = nullptr;

// Parser state (only used inside rxTask, so no lock is needed)
static uint8_t  g_pbuf[PARSER_BUF];
static uint16_t g_plen = 0;

// =============================================================================
// 6. Serial helpers
// =============================================================================
// Whether CDC is really attached to a host. While unmounted, writing to Serial blocks until the tx
// timeout and would slow the telemetry task down, so callers check first.
static bool serial_up() {
#if defined(ARDUINO_USB_CDC_ON_BOOT) && (ARDUINO_USB_CDC_ON_BOOT)
    return (bool)Serial;               // USBCDC / HWCDC: true only while actually connected
#else
    return true;                       // UART0 direct wiring: always writable
#endif
}

static bool hid_ready() {
    return tud_hid_n_ready(0);         // host is polling HID interface 0 and the endpoint is writable
}

static bool usb_mounted() {
    return hid_ready();
}

// =============================================================================
// 7. Displacement accumulator
// =============================================================================
// RX side: accumulate only. AXIS_CAP saturates any overflow - a brief link stall never becomes a teleport.
static void acc_add(int16_t dx, int16_t dy) {
    bool overflow = false;
    portENTER_CRITICAL(&g_acc_mux);
    g_acc_x += (int32_t)dx;
    g_acc_y += (int32_t)dy;
    if (g_acc_x > AXIS_CAP)  { g_acc_x = AXIS_CAP;  overflow = true; }
    if (g_acc_x < -AXIS_CAP) { g_acc_x = -AXIS_CAP; overflow = true; }
    if (g_acc_y > AXIS_CAP)  { g_acc_y = AXIS_CAP;  overflow = true; }
    if (g_acc_y < -AXIS_CAP) { g_acc_y = -AXIS_CAP; overflow = true; }
    portEXIT_CRITICAL(&g_acc_mux);
    if (overflow) {
        g_queue_drops++;               // the saturated amount stays visible to the host
    }
}

// Report side: take one sub-frame of at most +/-127 and leave the remainder for the next report.
static void acc_take(int32_t *out_x, int32_t *out_y) {
    int32_t sx, sy;
    portENTER_CRITICAL(&g_acc_mux);
    sx = g_acc_x;
    sy = g_acc_y;
    if (sx > MAX_PER_REPORT)       sx = MAX_PER_REPORT;
    else if (sx < -MAX_PER_REPORT) sx = -MAX_PER_REPORT;
    if (sy > MAX_PER_REPORT)       sy = MAX_PER_REPORT;
    else if (sy < -MAX_PER_REPORT) sy = -MAX_PER_REPORT;
    g_acc_x -= sx;
    g_acc_y -= sy;
    portEXIT_CRITICAL(&g_acc_mux);
    *out_x = sx;
    *out_y = sy;
}

// Clear the accumulator. With counted=true the leftover is charged to queue_drops (the host can see that something was dropped).
static void acc_clear(bool counted) {
    int32_t leftover_x, leftover_y;
    portENTER_CRITICAL(&g_acc_mux);
    leftover_x = g_acc_x;
    leftover_y = g_acc_y;
    g_acc_x = 0;
    g_acc_y = 0;
    portEXIT_CRITICAL(&g_acc_mux);
    if (counted && (leftover_x || leftover_y)) {
        // Account in whole reports so a 500 px displacement is not booked as a single drop
        uint32_t units = (uint32_t)(abs(leftover_x) + abs(leftover_y) + MAX_PER_REPORT - 1) / MAX_PER_REPORT;
        g_queue_drops += units;
    }
}

// =============================================================================
// 8. Buttons
// =============================================================================
// RX side: write only the target state and wake the report task at once. HID calls belong to the report task - see constraint 1 at the top of the file.
static void buttons_set_target(uint8_t mask) {
    uint8_t want = (uint8_t)(mask & BTN_LEFT);
    if (want == g_btn_target) return;              // no change: do not disturb the report task
    g_btn_target = want;
    if (g_task_report != nullptr) {
        xTaskNotifyGive(g_task_report);            // wakes the report task immediately while it is waiting for the period
    }
}

// Report side: apply the target state to HID. This runs serially in one task, so press/release never interleaves with move.
static void buttons_apply() {
    if (g_deadman) return;                         // no button action of any kind while deadman is latched
    uint8_t want = g_btn_target;
    if (want == g_btn_applied) return;
    if (want & BTN_LEFT) {
        Mouse.press(MOUSE_LEFT);
    } else {
        Mouse.release(MOUSE_LEFT);
    }
    g_btn_applied = want;
    g_hid_reports++;
}

// Idempotent release: the deadman path, link recovery and error paths all come through here, so a button never sticks.
static void buttons_release_all() {
    if (g_btn_applied != 0) {
        Mouse.release(MOUSE_LEFT);
        g_btn_applied = 0;
    }
    g_btn_target = 0;                              // drop any leftover "pressed" intent
}

// =============================================================================
// 9. Frame parsing (equivalent to the host StreamParser, resync included)
// =============================================================================
// CRC/length/version error: drop 1 byte and keep scanning, counting crc_errors like the host.
// CRC_STORM_LIMIT failures in a row -> the link is considered dirty and the accumulator is cleared (noise must not become motion).
static void parser_drop_one() {
    if (g_plen > 0) {
        g_plen--;
        if (g_plen) {
            memmove(g_pbuf, g_pbuf + 1, g_plen);
        }
    }
    g_crc_errors++;
    if (++g_crc_streak >= CRC_STORM_LIMIT) {
        g_crc_streak = 0;
        acc_clear(true);
    }
}

static void parser_feed(uint8_t byte) {
    if (g_plen >= PARSER_BUF) {                    // unreachable in theory (a single frame is <=133 bytes), pure belt-and-braces
        parser_drop_one();
    }
    g_pbuf[g_plen++] = byte;

    for (;;) {
        if (g_plen == 0) {
            return;
        }
        // 1) Find magic. None anywhere -> it is all garbage, clear it (same as host find(MAGIC) < 0)
        uint16_t start = 0;
        while (start < g_plen && g_pbuf[start] != PROTO_MAGIC) {
            ++start;
        }
        if (start == g_plen) {
            g_plen = 0;
            return;
        }
        if (start > 0) {
            g_plen -= start;
            memmove(g_pbuf, g_pbuf + start, g_plen);
        }

        // 2) magic+ver/type+seq+len+crc is the minimum needed before any decision
        if (g_plen < FRAME_HDR + 1) {
            return;                                // wait for more bytes
        }

        // 3) Length sanity: > 128 means desync, drop 1 byte and rescan (same as the host)
        uint16_t payload_len = g_pbuf[3];
        if (payload_len > MAX_PAYLOAD) {
            parser_drop_one();
            continue;
        }
        uint16_t total = (uint16_t)(FRAME_HDR + payload_len + 1);
        if (g_plen < total) {
            return;                                // half a frame, wait for the remaining bytes
        }

        // 4) Version + CRC check. Any mismatch only drops 1 byte so later bytes can re-frame
        uint8_t version = (uint8_t)(g_pbuf[1] >> 4);
        if (version != PROTO_VERSION || crc8_atm(g_pbuf, (uint16_t)(total - 1)) != g_pbuf[total - 1]) {
            parser_drop_one();
            continue;
        }

        // 5) Framed: dispatch, then shift the buffer
        parser_dispatch((uint8_t)(g_pbuf[1] & 0x0F), g_pbuf + FRAME_HDR, payload_len);
        g_plen -= total;
        if (g_plen) {
            memmove(g_pbuf, g_pbuf + total, g_plen);
        }
        // keep trying to parse the next frame coalesced behind this one
    }
}

static void parser_dispatch(uint8_t kind, const uint8_t *payload, uint16_t len) {
    g_rx_frames++;
    g_crc_streak = 0;
    g_last_rx_us = esp_timer_get_time();           // every valid frame feeds the watchdog
    g_deadman = false;

    switch (kind) {
        case T_MOVE: {
            if (len < 4) { g_unknown_frames++; return; }
            int16_t dx = (int16_t)(uint16_t)((uint16_t)payload[0] | ((uint16_t)payload[1] << 8));
            int16_t dy = (int16_t)(uint16_t)((uint16_t)payload[2] | ((uint16_t)payload[3] << 8));
            acc_add(dx, dy);                       // accumulate only, never send HID here
            break;
        }
        case T_BUTTON: {
            if (len < 1) { g_unknown_frames++; return; }
            buttons_set_target(payload[0]);
            break;
        }
        case T_PING: {
            if (len < 4) { g_unknown_frames++; return; }
            g_ping_count = (uint32_t)payload[0] | ((uint32_t)payload[1] << 8) |
                           ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);
            break;                                 // PING only feeds the watchdog, there is no reply
        }
        case T_CONFIG:
        default:
            // Reserved/unknown type: CRC passed, ignored semantically, counted only (matches the host's "ignore but count")
            g_unknown_frames++;
            break;
    }
}

// =============================================================================
// 10. Transmit side (STATUS)
// =============================================================================
static void send_frame(uint8_t kind, const uint8_t *payload, uint16_t len) {
    if (len > MAX_PAYLOAD) {
        return;
    }
    if (!serial_up()) {
        return;                                    // no host listening: skip the write to avoid blocking
    }
    uint8_t frame[FRAME_MAX];
    frame[0] = PROTO_MAGIC;
    frame[1] = (uint8_t)((PROTO_VERSION << 4) | (kind & 0x0F));
    frame[2] = g_tx_seq++;
    frame[3] = (uint8_t)len;
    if (len) {
        memcpy(frame + FRAME_HDR, payload, len);
    }
    frame[FRAME_HDR + len] = crc8_atm(frame, (uint16_t)(FRAME_HDR + len));
    Serial.write(frame, (size_t)(FRAME_HDR + len + 1));
}

static inline uint16_t sat_u16(uint32_t v) {
    return (uint16_t)(v > 0xFFFFu ? 0xFFFFu : v);
}

static void send_status() {
    uint8_t  p[STATUS_LEN_MAX];
    uint32_t uptime_ms = (uint32_t)(esp_timer_get_time() / 1000);
    uint16_t rx        = sat_u16(g_rx_frames);
    uint16_t crc_err   = sat_u16(g_crc_errors);
    uint16_t drops     = sat_u16(g_queue_drops);
    uint8_t  flags     = 0;
    if (g_deadman)  flags |= FLAG_DEADMAN;
    if (g_usb_ready) flags |= FLAG_USB_READY;
    if (g_hid_stalls > 0 && g_hid_reports == 0) flags |= FLAG_HID_STALLED;

    // Little-endian by hand, no struct packing: the protocol is a byte-order contract, not a compiler contract
    p[0] = (uint8_t)(uptime_ms);
    p[1] = (uint8_t)(uptime_ms >> 8);
    p[2] = (uint8_t)(uptime_ms >> 16);
    p[3] = (uint8_t)(uptime_ms >> 24);
    p[4] = (uint8_t)(rx);
    p[5] = (uint8_t)(rx >> 8);
    p[6] = (uint8_t)(crc_err);
    p[7] = (uint8_t)(crc_err >> 8);
    p[8] = (uint8_t)(drops);
    p[9] = (uint8_t)(drops >> 8);

    if (STATUS_HZ_AS_U8) {
        // legacy 12-byte layout ("<IHHHBB"): hz has one byte and saturates at 255
        p[10] = (uint8_t)(g_report_hz > 255 ? 255 : g_report_hz);
        p[11] = flags;
    } else {
        // current 13-byte layout ("<IHHHHB"): hz is u16 LE, so 1000 Hz is reported truthfully
        p[10] = (uint8_t)(g_report_hz);
        p[11] = (uint8_t)(g_report_hz >> 8);
        p[12] = flags;
    }
    send_frame(T_STATUS, p, STATUS_LEN);
}

// =============================================================================
// 11. Task: serial parsing
// =============================================================================
// Polling instead of an onReceive callback keeps all parsing in this task's context:
// doing memmove + dispatch inside a callback stretches the ISR/event task cycle and makes the timing relation to HID hard to guarantee.
// At 921600 bps 1 ms carries at most 92 bytes while the CDC ring buffer holds 2048 -> 20 ms+ of headroom.
static void rx_task(void *arg) {
    (void)arg;
    for (;;) {
        int avail = Serial.available();
        if (avail <= 0) {
            vTaskDelay(pdMS_TO_TICKS(RX_POLL_MS));
            continue;
        }
        for (int i = 0; i < avail; ++i) {
            int b = Serial.read();
            if (b < 0) {
                break;
            }
            parser_feed((uint8_t)b);
        }
    }
}

// =============================================================================
// 12. Task: HID report (1000 Hz, the only place Mouse.* is called)
// =============================================================================
// A deadline plus a wait that button notifications can interrupt gives vTaskDelayUntil's fixed-rate
// semantics while cutting BUTTON frame latency from "up to one period" to "microseconds":
//   - ulTaskNotifyTake wakes on buttons_set_target() -> apply at once, then wait for the period again;
//   - the last SPIN_ALIGN_US microseconds spin-align the period so tick quantization never adds 1-2 ms of jitter.
static void report_task(void *arg) {
    (void)arg;
    const int64_t period_us = 1000000 / REPORT_HZ;
    int64_t next_us = esp_timer_get_time();

    for (;;) {
        buttons_apply();                           // every wake handles buttons first (higher priority than motion)

        int64_t now_us = esp_timer_get_time();
        if (now_us < next_us) {
            int64_t wait_us = next_us - now_us;
            if (wait_us > (int64_t)SPIN_ALIGN_US) {
                uint32_t ms = (uint32_t)((wait_us - SPIN_ALIGN_US + 999) / 1000);
                ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms ? ms : 1));
                continue;                          // re-evaluate after waking: it may just have been a button notification
            }
            while (esp_timer_get_time() < next_us) {
                // short spin-align (<=250 us) so the report lands close to the nominal period
            }
        }

        service_motion(next_us);

        next_us += period_us;
        int64_t after_us = esp_timer_get_time();
        if (next_us <= after_us) {
            // preempted for more than one period: resync instead of bursting catch-up reports
            next_us = after_us + period_us;
        }
    }
}

// What one report period does: deadman check -> take the sub-frame motion -> send HID.
static void service_motion(int64_t now_us) {
    // --- deadman: on timeout release the button and stop output ---
    if ((now_us - g_last_rx_us) > (int64_t)DEADMAN_MS * 1000) {
        if (!g_deadman) {
            g_deadman = true;
            buttons_release_all();                 // key point: never stick a button
            acc_clear(true);                       // stale motion is voided
        }
        return;                                    // stop output
    }
    // reached only when the link is alive (dispatch usually clears the flag on a valid frame)
    g_deadman = false;

    // --- motion: take one sub-frame of at most +/-127, the remainder stays in the accumulator ---
    int32_t dx = 0, dy = 0;
    acc_take(&dx, &dy);
    if (dx != 0 || dy != 0) {
        // Send only when the endpoint is ready; otherwise put the motion back into the accumulator.
        // That way "no host" is never disguised as "all good": report_hz stays 0,
        // hid_stalls keeps growing and the HID_STALLED flag is set - visible at a glance.
        if (hid_ready()) {
            Mouse.move((int8_t)dx, (int8_t)dy, 0);
            g_hid_reports++;
        } else {
            acc_add((int16_t)dx, (int16_t)dy);
            g_hid_stalls++;
        }
    }
}

// =============================================================================
// 13. Task: telemetry (100 ms)
// =============================================================================
static void status_task(void *arg) {
    (void)arg;
    uint32_t last_reports = 0;
    uint32_t last_ms      = (uint32_t)(esp_timer_get_time() / 1000);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(STATUS_PERIOD_MS));

        uint32_t now_ms  = (uint32_t)(esp_timer_get_time() / 1000);
        uint32_t reports = g_hid_reports;
        uint32_t dt_ms   = now_ms - last_ms;
        if (dt_ms == 0) {
            dt_ms = 1;
        }
        uint32_t hz = (uint32_t)(((uint64_t)(reports - last_reports) * 1000u) / dt_ms);
        last_reports = reports;
        last_ms      = now_ms;
        g_report_hz  = (uint16_t)(hz > 0xFFFFu ? 0xFFFFu : hz);

        g_usb_ready = usb_mounted();
        send_status();
    }
}

// =============================================================================
// 14. Lifecycle
// =============================================================================
void setup() {
    // --- serial: set the ring buffer before begin (required by core 3.x) ---
    Serial.setRxBufferSize(SERIAL_RX_BUFSZ);
    Serial.begin(SERIAL_BAUD);

    // --- USB device stack + HID interface ---
    // Same order as the official core 3.x USBHIDMouse example: start USB, then register the HID interface.
    USB.begin();
    Mouse.begin();                                  // core 3.x allows interfaces to be added after USB.begin()
    g_usb_ready = usb_mounted();

    g_last_rx_us = esp_timer_get_time();            // the watchdog starts counting at boot, so deadman cannot trip for the first 300 ms
    g_acc_x = 0;
    g_acc_y = 0;
    g_btn_target = 0;
    g_btn_applied = 0;

    if (FW_BANNER && serial_up()) {
        Serial.print("\n[esp32-hid] v2 proto, ");
        Serial.print((unsigned)REPORT_HZ);
        Serial.print(" Hz, deadman ");
        Serial.print((unsigned)DEADMAN_MS);
        Serial.println(" ms");
    }

    // --- the three tasks ---
    TaskHandle_t h;

    h = nullptr;
    if (xTaskCreatePinnedToCore(rx_task, "hid_rx", STACK_PARSER, nullptr,
                                PRIO_PARSER, &h, CORE_USB) != pdPASS) {
        if (serial_up()) Serial.println("[esp32-hid] FATAL: rx_task creation failed");
    }

    h = nullptr;
    if (xTaskCreatePinnedToCore(report_task, "hid_report", STACK_REPORT, nullptr,
                                PRIO_REPORT, &g_task_report, CORE_REPORT) != pdPASS) {
        g_task_report = nullptr;
        if (serial_up()) Serial.println("[esp32-hid] FATAL: report_task creation failed");
    }

    h = nullptr;
    if (xTaskCreatePinnedToCore(status_task, "hid_status", STACK_STATUS, nullptr,
                                PRIO_STATUS, &h, CORE_USB) != pdPASS) {
        if (serial_up()) Serial.println("[esp32-hid] FATAL: status_task creation failed");
    }
}

// loop() carries no real-time duty: all timing lives in the three tasks above.
void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
