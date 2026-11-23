<div align="right">

[简体中文](README.zh-CN.md) | **English**

</div>

# ESP32 HID Bridge Firmware (esp32_hid)

> ⚠️ **For learning, research and engineering study only.**
> Do not use this project in any competitive or ranked setting. See the
> [disclaimer](../../DISCLAIMER.md).

A bridge firmware that turns serial binary frames into a **native USB HID mouse**. It does
exactly one thing: take the displacement/button frames that the host `aim2` sends, split them
into HID reports at 1000 Hz, and send those to the target machine. All aiming logic,
coordinate conversion, smoothing/prediction live on the host side
([`src/dma/`](../../src/dma/)); this firmware **makes no decisions at all** — it only
guarantees timing, losslessness, and immediate release on failure.

```
macOS: aim2 ──USB CDC 921600──► ESP32-S3 ──USB HID──► target machine / game
                (v2 binary frames)                   (mouse reports, 1000 Hz)
```

One USB cable carries **both the CDC serial port and the HID mouse** (a TinyUSB composite
device), so the host sees `/dev/cu.usbmodem*` while the controlled machine sees an ordinary
USB mouse.

---

## 1. Wiring topology (first, which cable goes into which machine)

### 1.0 Bill of materials (the whole system is just these four items)

| # | Item | Qty | Notes |
| --- | --- | --- | --- |
| 1 | HDMI capture card (UVC / MS2109 and the like) | 1 | Windows' HDMI output goes into it, and it plugs into the Mac's USB |
| 2 | HDMI cable | 1 | Windows GPU/motherboard HDMI → capture card HDMI IN |
| 3 | ESP32-S3 dev board (two USB ports: `USB` + `UART`) | 1 | Native USB emits HID; the onboard UART bridge receives the protocol |
| 4 | USB Type-C data cable (not a charge-only cable) | **2** | One goes from the ESP32 `UART` port → Mac; one from the ESP32 `USB` port → Windows |

The entire wiring is only these four connections, and any one of them being wrong shows up as
"nothing moves":

```
Windows ──HDMI──▶ capture card ──USB──▶ Mac        (video direction)
Mac ──Type-C──▶ ESP32 `UART` port                  (control direction: protocol frames at 921600)
ESP32 `USB` port ──Type-C──▶ Windows               (output direction: HID mouse)
```

This project uses a **two-machine topology**: the ESP32 is connected to two computers at once:

```
                 ┌──────────── native USB port (silkscreen `USB`) ────────▶ Windows (gaming PC)
                 │                                                        ↑ HID mouse injection
   ESP32-S3 ─────┤
                 └──────────── UART port (silkscreen `UART`, onboard CP2102/CH343) ──▶ Mac (inference machine)
                                                                          ↑ protocol frames at 921600
```

| Port | Silkscreen | Peer | Purpose | Host-side device name |
| --- | --- | --- | --- | --- |
| Native USB | `USB` | **Windows** | HID mouse only | `USB input device / HID-compliant mouse` in Device Manager |
| UART | `UART` | **Mac** | Protocol frames only (v2, CRC8) | `/dev/cu.usbserial-*`, `/dev/cu.wchusbserial-*` |

The firmware must therefore use **`USB_MODE=0` + `CDC_ON_BOOT=0`**: `Serial` lands on
UART0(GPIO43/44), that is, the UART port; the native USB carries HID only. The default env
`esp32-s3-uart-link` is exactly this configuration.

> If your inference machine and gaming PC are the same box (single-machine topology), switch to
> env `esp32-s3-single-usb` (`CDC_ON_BOOT=1`); the serial port and HID then share one USB port,
> and the host-side serial device is `/dev/cu.usbmodem*`. Under either topology the automatic
> discovery in `SerialSink` finds the device.

| Item | Requirement |
| --- | --- |
| MCU | ESP32-S3-DevKitC-1 (recommended); ESP32-S2 also works (single core, the report task and the parse task share a core) |
| Power | Native USB power is enough; 100 mA is plenty for a HID-class device |
| Cables | Must be data cables; do not run the HID cable through a Hub/KVM/extension cable (it lowers the polling rate and causes occasional dropouts) |
| Indicator LED | None; all status goes through STATUS frame telemetry |

---

## 2. Board settings (**get this wrong and HID will never work**)

Arduino IDE 2.x + `esp32` core **3.x** (in Boards Manager pick esp32 by Espressif 3.x).

| Option | Two-machine topology (this project) | Single-machine topology |
| --- | --- | --- |
| Board | `ESP32S3 Dev Module` | Same |
| **USB Mode** | **`USB-OTG (TinyUSB)`** | Same (the only mode in which `USBHIDMouse` enumerates) |
| **USB CDC On Boot** | **`Disabled`** (`Serial`→UART0) | `Enabled` (`Serial`→USB CDC) |
| Flash Size / PSRAM / Partition | Whatever the board actually has / unused / default | Same |
| Upload Speed | `921600` (drop to `115200` if unstable) | Same |
| CPU Frequency | `240 MHz` | Same |

The PlatformIO equivalents are `env:esp32-s3-uart-link` (default) / `env:esp32-s3-single-usb`.

### Why `USB_MODE` must not be 1

`ARDUINO_USB_MODE=1` selects the "hardware USB-Serial/JTAG peripheral": TinyUSB is never
initialised, the `USBHIDMouse` class is not compiled into the firmware at all, and HID never
enumerates. The board manifest injects `-DARDUINO_USB_MODE=1` by default, so `platformio.ini`
strips it with `build_unflags` first and then sets it explicitly to `0`.
`[env:esp32-s3-hwcdc-jtag]` keeps that literal combination, **for comparison only — it must not
be used to run**.

---

## 3. Build and flashing

### 3.0 Key finding: flashing does not require unplugging the UART cable from the Mac

The ESP32-S3 ROM download mode supports **both physical ports**: UART0 and the native USB
(USB-Serial/JTAG). You are currently wired as "UART→Mac, USB→Windows", so either path can
flash, **without touching a cable**:

| Flashing method | Which port | Procedure | When it fits |
| --- | --- | --- | --- |
| **A. Flash from Windows (recommended, this answers your question)** | Native USB (already on Windows) | Put the board into download mode: hold **BOOT** → tap **RST** → release BOOT; Windows Device Manager shows `USB JTAG/serial debug unit (COMx)`; flash through it | You have Arduino IDE / PlatformIO / esptool on Windows |
| **B. Flash from the Mac** | UART (already on the Mac) | `pio run -t upload` (default env) or esptool pointed at `/dev/cu.usbserial-*`; the S3 DevKit's UART port has DTR/RTS auto-reset, so you normally need no BOOT press | You have the toolchain on the Mac, or you want to flash with a single command |

- After flashing with method A, press **RST** (or use esptool `--after hard_reset`) → the
  firmware boots → Windows re-enumerates the HID device. The Mac-side `/dev/cu.usbserial-*` is
  unaffected the whole time, because ROM download mode runs over the native USB and does not
  occupy UART0.
- Method A presupposes that you **can enter download mode**: if the BOOT button is awkward to
  press, short GPIO0 to GND and then reset.
- If you insist that "COM must be on Windows": that only means moving the UART cable over to
  Windows and using it for **method B**; but afterwards you must plug it back into the Mac,
  otherwise `SerialSink` on the Mac side finds no serial port (`aim2 doctor` reports
  "no serial device found"). **Try A first; you normally do not need to move a cable.**

### 3.1 PlatformIO

```bash
cd firmware/esp32_hid
pio run                       # two-machine topology (default env, protocol over UART0)
pio run -t upload             # flash (use this when the UART port is on the Mac; if it is moved to Windows, use method A)
pio run -e esp32-s3-single-usb -t upload   # single-machine topology
pio device monitor -b 921600  # under the two-machine topology, monitor watches the UART port
```

### 3.2 Arduino IDE

1. Open `firmware/esp32_hid/esp32_hid.ino` (a warning about missing files in the same directory
   can be ignored);
2. In Tools, set the options from section 2 (two-machine topology: `USB-OTG (TinyUSB)` +
   `CDC On Boot: Disabled`);
3. Pick the **UART port** (`/dev/cu.usbserial-*`, when flashing from the Mac) or the **native USB
   download-mode COM** (when flashing from Windows, BOOT+RST first); then Upload;
4. If it fails, first confirm that you really entered download mode (on Windows, watch whether
   the COM number changes; on the Mac, check `ls /dev/cu.*`).

Self-check after flashing:

- Windows: Device Manager should show `HID-compliant mouse` (unplug and replug once to confirm);
- Mac: `ls /dev/cu.usbserial-*` lists the device, and the serial line of `python -m dma.cli doctor`
  is OK;
- End to end: run `python -m dma.cli run --dry-run` and check that the health status is `ok`,
  then `run` and watch whether the mouse responds.

## 4. Power-on behaviour

| Time | Behaviour |
| --- | --- |
| 0 ms | `Serial`(CDC) + `USB.begin()` + `Mouse.begin()`, 3 tasks are created; `g_last_rx_us` starts counting here |
| 0–300 ms | **deadman state**: no HID output at all, STATUS `flags` bit0 = 1 (this is normal) |
| First valid frame received | deadman clears, `flags` bit0 = 0, output starts at 1000 Hz |
| Every 100 ms | STATUS is reported proactively (uptime / counters / report rate / flags) |
| 300 ms with no valid frame | immediately `Mouse.release(MOUSE_LEFT)`, clear the accumulator, stop output (`flags` bit0 = 1) |

Power-on banner (pure ASCII, no `0xA5`; the host parser discards it as garbage and does not
count a CRC error):

```
[esp32-hid] v2 proto, 1000 Hz, deadman 300 ms
```

---

## 5. Wire protocol (byte-for-byte identical to `src/dma/transport/protocol.py`)

```
off 0 : 0xA5                                  magic
off 1 : (2 << 4) | type                       version (high nibble) | type (low nibble)
off 2 : seq                                   sequence number, wraps around
off 3 : len                                   payload length
off 4 : payload[len]
last  : CRC-8/ATM (poly 0x07, init 0, MSB-first, not reflected), covering bytes[0 : 4+len]
```

| type | Direction | payload | Firmware behaviour |
| --- | --- | --- | --- |
| 1 `MOVE` | host→device | `int16 LE dx`, `int16 LE dy` | Only adds into the `int32` accumulator; **never emits HID from the parse context** |
| 2 `BUTTON` | host→device | `uint8 mask` (bit0 = left button) | Updates the target state + immediately wakes the report task (µs-level response, so a host tap of 30 ms is never lost) |
| 3 `PING` | host→device | `uint32 LE counter` | Feeds the watchdog (no reply) |
| 4 `STATUS` | **device→host** | 13 bytes, see below | Reported proactively every 100 ms |
| 5 `CONFIG` | host→device | arbitrary | Reserved: CRC is checked and the frame is then ignored, counted as `unknown`, and also treated as link-alive |

**Resynchronisation** (same semantics as the host `StreamParser`): no `0xA5` found → discard the
whole chunk and count no error; `len > 128`, version not 2, CRC mismatch → **drop only 1 byte**
and keep scanning; a half frame waits for the following bytes. That way hot-plugging, firmware
restarts and USB glitches can never wedge the link.

### STATUS payload (13 bytes, `struct "<IHHHHB"`)

Strictly consistent with the host's `protocol.STATUS_LEN == 13` /
`decode_status()`'s `struct.unpack("<IHHHHB", payload[:13])`:

| Offset | Field | Type | Notes |
| --- | --- | --- | --- |
| 0..3 | `uptime_ms` | `uint32` LE | Milliseconds since power-on (`esp_timer`, wraps after about 49 days) |
| 4..5 | `rx_frames` | `uint16` LE | Number of **valid** frames received (MOVE/BUTTON/PING/CONFIG all counted), saturating at 65535 |
| 6..7 | `crc_errors` | `uint16` LE | Count of CRC / length / version errors, counted the same way as the host's `StreamParser.crc_errors` |
| 8..9 | `queue_drops` | `uint16` LE | Displacement that was discarded (accumulator saturation / CRC storm / deadman cleanup), in units of "reports" |
| 10..11 | `report_hz` | `uint16` LE | HID reports actually sent in the last 100 ms, converted to Hz (0 when there is no displacement; 1000 is reported faithfully) |
| 12 | `flags` | `uint8` | bit0 = deadman active; bit1 = USB ready/enumerated; the remaining bits are reserved as 0 |

The whole frame is 18 bytes (magic + ver/type + seq + len + 13B payload + CRC). Every field is
written by hand in little-endian; we do not rely on the compiler's struct packing (the protocol
is a byte-order contract, not a compiler contract).

> Backwards compatibility: to interoperate with the earlier 12-byte layout (`"<IHHHBB"`, where
> `report_hz` is a `uint8` and 1000 Hz saturates to 255), set `STATUS_HZ_AS_U8` at the top of the
> source to `true` to fall back; normal builds keep it `false`.

---

## 6. Timing design (why it is written this way)

| Concern | Approach |
| --- | --- |
| Who touches HID | Only `report_task`. `Mouse.move/press/release` **are called in exactly these three places in the whole file**, and serially from the same task — `Mouse.move()` blocks until the next HID report period, and calls interleaved across tasks reorder reports and make the cursor jitter |
| Report tick | `report_task` is pinned to **core 1** at priority 20; "deadline + a wait that can be interrupted by a notification" reproduces the fixed-rate semantics of `vTaskDelayUntil`, with a spin-wait for the last ≤250 µs so that tick quantisation cannot turn 1000 Hz into 1–2 ms of jitter |
| Button latency | The parse task wakes the report task with `xTaskNotifyGive()` on a `BUTTON` frame → HID is issued immediately; measured in the µs range, far below the host's 30 ms tap |
| Lossless displacement | A single `Mouse.move` is capped at ±127 per axis; the excess stays in the accumulator (cap ±508, the same value as the host `planner`'s `4×max_per_report`) and goes out with the next report, **the remainder is never dropped**. Only then can the controller converge to sub-pixel near the target |
| Jitter isolation | The parse task runs on **core 0** (the same core as USB/TinyUSB), the report task gets **core 1** to itself |
| CRC storm | 32 consecutive CRC errors → clear the accumulator (noise must not become displacement), accounted in `queue_drops` |
| deadman | 300 ms without a valid frame → release buttons + stop output + clear the accumulator; a stuck button is the most dangerous failure mode |
| Telemetry | A separate `status_task` (core 0, low priority) sends STATUS every 100 ms; it writes only when CDC is actually mounted, so a blocking write cannot slow the other tasks down |

Task layout:

| Task | core | Priority | Stack | Responsibility |
| --- | --- | --- | --- | --- |
| `hid_rx` | 0 | 8 | 4096 | Poll CDC → parse the byte stream → accumulator/buttons/watchdog |
| `hid_report` | 1 | 20 | 3072 | 1000 Hz: deadman check → take a sub-frame → `Mouse.move()` |
| `hid_status` | 0 | 3 | 3072 | 100 ms STATUS telemetry + report-rate conversion |

All tunables are gathered at the top of the source in `namespace fw`: `REPORT_HZ`,
`DEADMAN_MS`, `MAX_PER_REPORT`, `AXIS_CAP`, `CRC_STORM_LIMIT`, `STATUS_PERIOD_MS`, the task
cores/priorities/stacks, and `SERIAL_RX_BUFSZ` (2048 B CDC ring buffer, ≈22 ms of headroom at
921600 bps).

---

## 7. Verification

### 7.1 `aim2 doctor`

```bash
aim2 doctor            # or: aim2 doctor --port /dev/cu.usbmodem1301
```

The fields provided by `SerialLink.status()` should satisfy:

| Field | Expectation |
| --- | --- |
| `port` | `/dev/cu.usbmodem*` (auto-detection order `usbmodem` → `wchusbserial` → `ttyUSB/ttyACM`) |
| `frames` / `errors` | `frames` keeps growing; `errors` should stay at 0 (5 consecutive write failures trigger a reconnect) |
| `crc_errors` | constant or growing very slowly; steady growth = cable/power problem |
| `device.uptime_s` | increases monotonically after power-on (the device is not rebooting) |
| `device.rx_frames` | grows by the same order of magnitude as the host's `frames` (the difference is frames lost inside reconnect windows) |
| `device.crc_errors` | device-side CRC errors, should stay at 0 in the long run |
| `device.queue_drops` | 0 in the long run; steady growth = accumulator saturation (the host fed too much at once) or a CRC storm |
| `device.report_hz` | close to the firmware's `REPORT_HZ` (1000) while there is displacement, and never above the USB endpoint's real polling ceiling; 0 when there is no displacement |

> `flags` (deadman / USB ready) is not currently exposed in
> `SerialLink.status()['device']`; a host-side output line in `status()` and in the metrics is
> needed before you can see it.

### 7.2 Bare-metal self-check without `aim2` (using only the project's own protocol module)

```python
import sys, time, serial
sys.path.insert(0, "src")
from dma.transport import protocol as P

ser = serial.Serial("/dev/cu.usbmodem1301", 921600, timeout=0.5)
# 1) Read STATUS: one frame every 100 ms, magic/CRC/fields should be self-consistent
p = P.StreamParser()
deadline = time.time() + 1.0
while time.time() < deadline:
    for kind, payload, seq in p.feed(ser.read(64)):
        if kind == P.T_STATUS:
            st = P.decode_status(payload)          # 13 bytes "<IHHHHB"
            print(f"uptime={st.uptime_ms}ms rx={st.rx_frames} crc={st.crc_errors} "
                  f"drops={st.queue_drops} hz={st.report_hz} flags={st.flags:#04x}")
# 2) Move the cursor 100 px and back (first make sure no game is running)
for dx in (50, 50, -50, -50):
    ser.write(P.encode_move(dx, 0, 0)); time.sleep(0.01)
# 3) A single 30 ms click
ser.write(P.encode_button(1, 0)); time.sleep(0.03); ser.write(P.encode_button(0, 0))
# 4) Stay silent for 0.5 s → the device should enter deadman (flags bit0 = 1), with no button left stuck
```

### 7.3 Disconnect safety (mandatory)

1. Start a tap with the left button held (`BUTTON 1` sent, `BUTTON 0` not yet sent);
2. `Ctrl-C` `aim2` (or just unplug the USB cable on the Mac side);
3. **The left button must release by itself within 300 ms**, and no displacement output follows.

### 7.4 Static regression possible inside the repo (already done)

The protocol core of `firmware/esp32_hid/esp32_hid.ino` (CRC, resync, sub-frame splitting,
deadman, STATUS bytes) has been compiled against stub headers on the host and run through 35
assertions, and cross-checked byte-for-byte against real frames produced by
[`src/dma/transport/protocol.py`](../../src/dma/transport/protocol.py) (including unpacking the
13-byte STATUS through `decode_status()`); **USB enumeration, HID report timing, the report rate
on real hardware, and the real button release of deadman can only be verified on the board**
(see §9).

---

## 8. Troubleshooting (FAQ)

### HID drops off / the device does not enumerate

| Symptom | Cause | Fix |
| --- | --- | --- |
| The Mac sees only a serial port, no mouse | USB Mode was set to `Hardware CDC and JTAG` | Change to `USB-OTG (TinyUSB)` + `USB CDC On Boot: Enabled`, re-flash |
| No device at all | The `UART` port (CP2102) is plugged in, not the native USB port | Move to the port silkscreened `USB` |
| No device at all | Charge-only cable / bad cable / Hub / KVM | Change the cable, connect directly |
| Enumerates, then disappears after tens of seconds | Power droop (long cable + clone board); or `usbmodem` on the host side is held by another program | Use a short cable, remove the Hub; check the holder with `lsof /dev/cu.usbmodem*` |
| Dead after macOS sleep/wake | USB re-enumeration, `aim2` reconnects via `reconnects` | After waking, confirm `reconnects` grows in `aim2 doctor` and `device.uptime_s` restarts from zero (a device reboot is normal) |

### Report rate below 1000 Hz

1. **First confirm which segment you are measuring**: `device.report_hz` is "the number of HID
   reports actually sent in the last 100 ms, converted to Hz", so reporting 0 when there is no
   displacement is normal; the host-side `report_hz` field is a uint16 and 1000 is not truncated.
2. The host-side `output.report_hz` (default 500) caps the send frame rate; the firmware emits
   reports at up to 1000 Hz, but sends no empty report when there is no new displacement.
3. The USB interrupt endpoint's `bInterval` sets the ceiling: this firmware uses core 3.x's
   `USBHIDMouse` (1 ms period). With a Hub/KVM in between, the host may downgrade it.
4. The serial side is not the bottleneck: 921600 bps ≈ 92 kB/s, and 500–1000 frames/s × 9–17 B
   ≈ 15 kB/s, a 6× margin.
5. CPU frequency must be 240 MHz; if WiFi/Bluetooth is enabled as well, the priority-20 report
   task can still be delayed by long critical sections in the radio tasks — **do not enable
   WiFi/BT on this bridge**.

### Stuck buttons (the most dangerous failure)

This firmware has three lines of defence; check them in order:

1. **deadman**: no valid frame for 300 ms → `release`. If the button still sticks, the device is
   still receiving frames (the host never stopped), or USB is already gone (the OS then releases
   buttons on device removal, but it is still worth keeping `REPORT_HZ` and not raising
   `DEADMAN_MS`).
2. **CRC storm**: 32 consecutive CRC errors clear the accumulator, but they do **not** release
   the button (releasing is deadman's job). With a bad cable and intermittent frames, deadman may
   never fire (an occasional good frame keeps the link alive) — that is a cable problem: change
   the cable and watch `device.crc_errors`.
3. **Host-side `FireCfg.tap_s = 30 ms`**: if `aim2` crashes after sending `BUTTON 1`, the device
   relies on deadman as the backstop. For extra safety, lower `DEADMAN_MS` to 150 ms (the cost: a
   host stall is more easily misread as a disconnect).

Other situations that can be mistaken for a "stuck button": the target game swallowing the
mouse-up by itself (lost foreground), HID state cached across a KVM switch, and sending `BUTTON`
frames that only press and never release.

### Cursor jitter / drifting displacement

- **Do not call `Mouse.*` anywhere else**: only `report_task` calls it in the whole file, and keep
  it that way while editing the code.
- If the accumulator cap `AXIS_CAP=508` is reduced, fast flicks get clipped (`queue_drops` will
  rise).
- The parse task sharing a core with USB (core 0) is deliberate; moving `report_task` to core 0 as
  well makes things noticeably worse.
- The host `planner`'s `max_per_report` and the firmware's `MAX_PER_REPORT` are both 127; never
  change only one side.

### Serial port not found / `aim2` reports "no serial device found"

- `Serial` landed on UART0 → `USB CDC On Boot` is not enabled (see the §2 matrix).
- The port name is not `usbmodem` (for example you used the CP2102 port) →
  `SerialLink.resolve_port()` matches `/dev/cu.usbmodem*` first, so a wrongly inserted port
  connects to the wrong device.
- On macOS it is held by another program (Arduino serial monitor, `screen`) → close it and retry.

### Flashing fails

- `A fatal error occurred: Failed to connect`: hold BOOT → tap RST → release BOOT, then upload
  again.
- The port was set to the UART port → select the native USB port instead (download mode also works
  over the native USB).
- Lower `upload_speed` to `115200`.

---

## 9. Items that can only be verified on the board (host regression cannot reach them)

1. **USB enumeration**: whether the composite-device descriptors and the HID report descriptor are
   accepted by macOS/Windows/Linux, and the actual `bInterval` value.
2. **Real report rate and jitter**: decided jointly by `Mouse.move()` blocking semantics + the
   TinyUSB FIFO + host polling; the only way to confirm is to probe D+/D- with an
   oscilloscope/logic analyser, or to sample timestamps with `raw input` on the controlled machine.
3. **µs-level button response**: the code path is "parse task notify → report task apply", µs-level
   in theory, but the real latency includes USB frame scheduling, so you must look at HID report
   timestamps on the target machine.
4. **deadman's real button release**: you need to observe "held → link down → auto-release within
   300 ms" on the OS side.
5. **921600 CDC throughput and the `SERIAL_RX_BUFSZ` boundary**: whether it overflows under extreme
   bursts (1000 frames/s for several seconds in a row).
6. **Long-run stability**: ≥ 8 h of continuous operation, no memory growth, `crc_errors` not
   drifting.
7. **Compatibility with the target game/system**: some games apply extra filtering and an
   acceleration curve to raw input from "non-physical mice"; that is a host-side calibration issue,
   but it can only be confirmed on real hardware.

---

## 10. Files

| File | Contents |
| --- | --- |
| `esp32_hid.ino` | The entire firmware implementation (single file, no third-party dependencies, comments explaining each section) |
| `platformio.ini` | PlatformIO project (the main env is usable; `esp32-s3-hwcdc-jtag` is a spec comparison only) |
| `README.md` | This document |
