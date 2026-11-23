"""Serial transport to the ESP32 HID bridge, with reconnect and device telemetry."""
from __future__ import annotations

import glob
import threading
import time
from typing import Optional

from ..core.config import OutputCfg
from ..core.metrics import Metrics
from . import protocol as P
from .link import LinkBase


class SerialLink(LinkBase):
    def __init__(self, cfg: OutputCfg, metrics: Metrics | None = None, port: str | None = None):
        super().__init__(cfg, metrics)
        self.port = port or cfg.port
        self._ser = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._seq = 0
        self._sent_mask = 0
        self._errors = 0
        self._reconnects = 0
        self._frames = 0
        self._last_hb = 0.0
        self._device: Optional[P.DeviceStatus] = None
        self._parser = P.StreamParser()
        self._last_ok_ts = 0.0
        self._error_flag = ""

    # ------------------------------------------------------------------
    def start(self) -> "SerialLink":
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="dma-link", daemon=True)
        self._thread.start()
        return self

    # ------------------------------------------------------------------
    @staticmethod
    def resolve_port() -> str:
        cands = sorted(
            glob.glob("/dev/cu.usbmodem*")
            + glob.glob("/dev/cu.wchusbserial*")
            + glob.glob("/dev/ttyUSB*")
            + glob.glob("/dev/ttyACM*")
            + glob.glob("COM[0-9]*")
        )
        if not cands:
            raise RuntimeError("no serial device found (usbmodem/wchusbserial/ttyUSB/ttyACM/COM)")
        return cands[0]

    def _open(self) -> bool:
        import serial

        try:
            port = self.resolve_port() if self.port in ("auto", "") else self.port
            self._ser = serial.Serial(
                port,
                int(self.cfg.baud),
                timeout=0.01,
                write_timeout=float(self.cfg.write_timeout_s),
            )
            self.port = port
            self._last_ok_ts = time.monotonic()
            self._error_flag = ""
            if self.metrics is not None:
                self.metrics.set("transport.connected", 1.0)
            return True
        except Exception as exc:
            self._error_flag = f"{type(exc).__name__}: {exc}"
            if self.metrics is not None:
                self.metrics.set("transport.connected", 0.0)
                self.metrics.inc("transport.open_errors")
            return False

    def _close_ser(self) -> None:
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        if self.metrics is not None:
            self.metrics.set("transport.connected", 0.0)

    # ------------------------------------------------------------------
    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def _write(self, frame: bytes) -> bool:
        t0 = time.perf_counter()
        try:
            self._ser.write(frame)  # type: ignore[union-attr]
            self._frames += 1
            self._errors = 0
            self._last_ok_ts = time.monotonic()
            if self.metrics is not None:
                self.metrics.observe("transport.write_ms", (time.perf_counter() - t0) * 1000.0)
                self.metrics.inc("transport.frames")
            return True
        except Exception as exc:
            self._errors += 1
            self._error_flag = f"{type(exc).__name__}: {exc}"
            if self.metrics is not None:
                self.metrics.inc("transport.write_errors")
            return False

    # ------------------------------------------------------------------
    def _read_available(self) -> None:
        ser = self._ser
        if ser is None:
            return
        try:
            waiting = ser.in_waiting
        except Exception:
            return
        if not waiting:
            return
        try:
            chunk = ser.read(waiting)
        except Exception:
            return
        for kind, payload, _seq in self._parser.feed(chunk):
            if kind == P.T_STATUS:
                st = P.decode_status(payload)
                if st is not None:
                    self._device = st
                    if self.metrics is not None:
                        self.metrics.set("device.uptime_s", st.uptime_ms / 1000.0)
                        self.metrics.set("device.crc_errors", float(st.crc_errors))
                        self.metrics.set("device.queue_drops", float(st.queue_drops))
                        self.metrics.set("device.report_hz", float(st.report_hz))
                        self.metrics.set("device.rx_frames", float(st.rx_frames))
                        self.metrics.set("device.deadman", float(st.flags & 0x01))
                        self.metrics.set("device.usb_ready", float((st.flags >> 1) & 0x01))

    # ------------------------------------------------------------------
    def _emit_motion(self, now: float) -> None:
        for dx, dy in self.planner.take(now):
            if self.cfg.protocol == "v1":
                frame = P.legacy_move(dx, dy, self._next_seq())
            else:
                frame = P.encode_move(dx, dy, self._next_seq())
            if not self._write(frame):
                break

    def _run(self) -> None:
        period = 1.0 / max(30, int(self.cfg.report_hz))
        next_tick = time.monotonic()
        while not self._stop.is_set():
            if self._ser is None:
                if not self._open():
                    time.sleep(max(0.2, float(self.cfg.reconnect_s)))
                    self._reconnects += 1
                    if self.metrics is not None:
                        self.metrics.inc("transport.reconnects")
                    continue
                next_tick = time.monotonic()

            now = time.monotonic()

            # 1. buttons are high priority and only sent on change
            mask = 1 if self._button else 0
            if mask != self._sent_mask:
                frame = (
                    P.legacy_button(mask, self._next_seq())
                    if self.cfg.protocol == "v1"
                    else P.encode_button(mask, self._next_seq())
                )
                if self._write(frame):
                    self._sent_mask = mask

            # 2. motion, rate limited by the planner
            self._emit_motion(now)

            # 3. heartbeat / ping
            if now - self._last_hb >= self.cfg.heartbeat_ms / 1000.0:
                if self.cfg.protocol == "v1":
                    self._write(P.legacy_heartbeat(self._next_seq()))
                else:
                    self._write(P.encode_ping(self._frames, self._next_seq()))
                self._last_hb = now

            # 4. drain device telemetry
            self._read_available()

            if self._errors >= 5:
                # port is wedged: close and let the loop reopen it
                self._close_ser()
                continue

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()

    # ------------------------------------------------------------------
    def ok(self) -> bool:
        return self._ser is not None and self._errors < 5 and not self.closed

    def status(self) -> dict:
        return {
            "ok": self.ok(),
            "port": self.port,
            "frames": self._frames,
            "errors": self._errors,
            "reconnects": self._reconnects,
            "crc_errors": self._parser.crc_errors,
            "device": None if self._device is None else {
                "uptime_s": self._device.uptime_ms / 1000.0,
                "rx_frames": self._device.rx_frames,
                "crc_errors": self._device.crc_errors,
                "queue_drops": self._device.queue_drops,
                "report_hz": self._device.report_hz,
                "flags": self._device.flags,
                "deadman": bool(self._device.flags & 0x01),
            },
            "detail": self._error_flag,
            "planner": self.planner.report(),
        }

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
