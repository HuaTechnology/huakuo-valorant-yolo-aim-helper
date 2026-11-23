"""Sub-frame motion planning: turn per-tick displacement into HID reports.

Two problems this solves.

**Quantisation.** HID reports carry 8-bit deltas (``-127..127``).  The old code
rounded every tick independently, so the sub-pixel part of a 3.4 px correction was
thrown away every single frame.  Near the target — where the error *is* sub-pixel
— rounding becomes a dead band the controller can never close, which is why v1
had to ship with a 4 px dead zone.  Here the remainder is carried, so the mean
error converges to zero.

**Rate mismatch.** Capture runs at 60-240 Hz, the HID device at 500-1000 Hz.  A
single 100 px report per capture frame produces a visible teleport and gets
clamped by many game engines' input smoothing.  The planner spreads the same
total over the reports that fit in the elapsed time, so the engine sees a smooth
ramp and no per-report clamp is exceeded.
"""
from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import List, Tuple

from ..core.config import OutputCfg


@dataclass
class PlanStats:
    ticks: int = 0
    reports: int = 0
    clipped_reports: int = 0
    residual_x: float = 0.0
    residual_y: float = 0.0


class MotionPlanner:
    def __init__(self, cfg: OutputCfg):
        self.cfg = cfg
        # The control thread adds, the link thread takes: read-modify-write on
        # the accumulator is not atomic in CPython, so a lost sub-pixel update is
        # possible without this lock.  It is held for microseconds.
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        with getattr(self, "_lock", threading.Lock()):
            self._acc_x = 0.0
            self._acc_y = 0.0
            self._last_ts: float | None = None
            self.stats = PlanStats()

    # ------------------------------------------------------------------
    def add(self, dx: float, dy: float) -> None:
        """Accumulate one control tick worth of desired movement."""
        cap = 4.0 * self.cfg.max_per_report
        with self._lock:
            self._acc_x += float(dx) * self.cfg.gain
            self._acc_y += float(dy) * self.cfg.gain
            # bound the accumulator: never let a stalled link turn into a huge jump
            self._acc_x = max(-cap, min(cap, self._acc_x))
            self._acc_y = max(-cap, min(cap, self._acc_y))
            self.stats.ticks += 1

    @property
    def pending(self) -> Tuple[float, float]:
        with self._lock:
            return self._acc_x, self._acc_y

    # ------------------------------------------------------------------
    def take(self, now: float) -> List[Tuple[int, int]]:
        """Split the accumulator into the reports due since the last call."""
        with self._lock:
            return self._take_locked(now)

    def _take_locked(self, now: float) -> List[Tuple[int, int]]:
        hz = max(30, int(self.cfg.report_hz))
        period = 1.0 / hz
        if self._last_ts is None:
            self._last_ts = now - period
        elapsed = max(0.0, now - self._last_ts)
        n = int(round(elapsed * hz))
        n = max(1, min(n, 8))
        self._last_ts = now

        steps: List[Tuple[int, int]] = []
        max_per = max(1, int(self.cfg.max_per_report))
        for i in range(n):
            remaining = n - i
            ix = int(round(self._acc_x / remaining))
            iy = int(round(self._acc_y / remaining))
            if abs(ix) > max_per or abs(iy) > max_per:
                self.stats.clipped_reports += 1
                ix = max(-max_per, min(max_per, ix))
                iy = max(-max_per, min(max_per, iy))
            self._acc_x -= ix
            self._acc_y -= iy
            if ix or iy:
                steps.append((ix, iy))
                self.stats.reports += 1
        self.stats.residual_x = self._acc_x
        self.stats.residual_y = self._acc_y
        return steps

    # ------------------------------------------------------------------
    def flush(self, now: float | None = None) -> List[Tuple[int, int]]:
        """Drain whatever is left (used on stop / aim disable)."""
        with self._lock:
            ix = int(round(self._acc_x))
            iy = int(round(self._acc_y))
            self._acc_x -= ix
            self._acc_y -= iy
        return [(ix, iy)] if (ix or iy) else []

    def report(self) -> dict:
        with self._lock:
            return self._report_locked()

    def _report_locked(self) -> dict:
        return {
            "ticks": self.stats.ticks,
            "reports": self.stats.reports,
            "clipped": self.stats.clipped_reports,
            "residual": round(math.hypot(self.stats.residual_x, self.stats.residual_y), 3),
        }
