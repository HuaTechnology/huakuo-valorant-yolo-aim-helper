"""Transport abstraction: a rate limited HID report pump.

The link owns the *only* clock that matters on the output side: the HID report
period.  Control ticks push desired displacement into an accumulator; the link
thread wakes every ``1/report_hz`` seconds and emits at most one motion report,
one button report (only on change) and a heartbeat.  Consequences:

* there is no queue, therefore no queue depth, therefore no latency growth when
  the serial port stalls — the accumulator simply coalesces;
* the report rate is independent of the capture rate, so a 60 fps capture can
  still drive a 500 Hz mouse;
* a dead port is visible (``status()['reconnects']``, ``crc_errors``) instead of
  being a silent ``print`` in a worker thread.
"""
from __future__ import annotations

import time
from typing import List, Optional, Tuple

from ..core.config import OutputCfg
from ..core.metrics import Metrics
from .planner import MotionPlanner


class LinkBase:
    def __init__(self, cfg: OutputCfg, metrics: Metrics | None = None):
        self.cfg = cfg
        self.metrics = metrics
        self.planner = MotionPlanner(cfg)
        self._button = False
        self.closed = False

    # -- API ---------------------------------------------------------------
    def submit_motion(self, dx: float, dy: float) -> None:
        self.planner.add(dx, dy)

    def submit_button(self, pressed: bool) -> None:
        self._button = bool(pressed)

    @property
    def button(self) -> bool:
        return self._button

    def ok(self) -> bool:
        return not self.closed

    def status(self) -> dict:
        return {"ok": self.ok()}

    def close(self) -> None:
        self.closed = True


class MockLink(LinkBase):
    """Records reports instead of writing a serial port (sim / CI / dry run)."""

    def __init__(self, cfg: OutputCfg, metrics: Metrics | None = None, record: bool = True):
        super().__init__(cfg, metrics)
        self.reports: List[Tuple[float, int, int]] = []
        self.buttons: List[Tuple[float, bool]] = []
        self._record = record
        self._last_ts: Optional[float] = None

    def submit_motion(self, dx: float, dy: float) -> None:
        super().submit_motion(dx, dy)
        self.pump(time.monotonic())

    def submit_button(self, pressed: bool) -> None:
        if bool(pressed) != self._button:
            self.buttons.append((time.monotonic(), bool(pressed)))
        super().submit_button(pressed)

    def pump(self, now: float) -> List[Tuple[int, int]]:
        steps = self.planner.take(now)
        if self._record:
            for dx, dy in steps:
                self.reports.append((now, dx, dy))
        if self.metrics is not None:
            self.metrics.inc("transport.reports", len(steps))
        return steps

    def total(self) -> Tuple[int, int]:
        sx = sum(r[1] for r in self.reports)
        sy = sum(r[2] for r in self.reports)
        return sx, sy

    def reset_counts(self) -> None:
        self.reports.clear()
        self.buttons.clear()

    def status(self) -> dict:
        return {"ok": True, "mock": True, "reports": len(self.reports), "buttons": len(self.buttons)}


def build_link(cfg: OutputCfg, metrics: Metrics | None = None, *, force_mock: bool = False):
    if force_mock or cfg.dry_run:
        return MockLink(cfg, metrics)
    from .serial_link import SerialLink

    return SerialLink(cfg, metrics)
