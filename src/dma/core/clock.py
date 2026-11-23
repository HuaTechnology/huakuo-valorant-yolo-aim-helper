"""Monotonic time helpers, rate meters and delay lines.

All timestamps in the pipeline are ``time.monotonic()`` seconds.  Wall clock is
only used for file names and log rotation.  Mixing the two is the classic source
of "negative latency" bugs in capture pipelines, so the rule is enforced here by
naming: anything called ``ts`` is monotonic.
"""
from __future__ import annotations

import math
import time
from collections import deque
from typing import Deque, Iterable, Tuple


def now() -> float:
    return time.monotonic()


def age_since(ts: float) -> float:
    return time.monotonic() - ts


class RateMeter:
    """Events per second over a sliding time window."""

    __slots__ = ("_ts", "_window")

    def __init__(self, window_s: float = 1.0):
        self._ts: Deque[float] = deque()
        self._window = float(window_s)

    def tick(self, ts: float | None = None) -> float:
        t = time.monotonic() if ts is None else ts
        self._ts.append(t)
        cut = t - self._window
        while self._ts and self._ts[0] < cut:
            self._ts.popleft()
        span = self._ts[-1] - self._ts[0]
        if span <= 1e-6:
            return float(len(self._ts))
        return (len(self._ts) - 1) / span

    @property
    def rate(self) -> float:
        if len(self._ts) < 2:
            return 0.0
        span = self._ts[-1] - self._ts[0]
        return (len(self._ts) - 1) / span if span > 1e-6 else 0.0

    def reset(self) -> None:
        self._ts.clear()


class FixedRate:
    """Deadline driven tick generator that does not drift.

    ``for _ in FixedRate(240):`` yields at 240 Hz; if a tick overruns the
    deadline the loop resynchronises instead of accumulating lag (accumulated
    lag turns into a burst of catch-up ticks, which the controller would see as
    a huge dt spike).
    """

    __slots__ = ("period", "_next", "_started", "late_ticks", "max_late_s")

    def __init__(self, hz: float):
        self.period = 1.0 / max(1.0, float(hz))
        self._next = 0.0
        self._started = False
        self.late_ticks = 0
        self.max_late_s = 0.0

    def __iter__(self):
        self._next = time.monotonic()
        self._started = True
        while True:
            self._next += self.period
            delay = self._next - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                late = -delay
                self.late_ticks += 1
                if late > self.max_late_s:
                    self.max_late_s = late
                if late > self.period:  # we are hopelessly behind: resync
                    self._next = time.monotonic()
            yield


class DelayLine:
    """History of (ts, x, y) with linear interpolation at ``ts - delay``.

    Used to recover the camera position at the instant the visible frame was
    actually produced: the frame in our hands is ``measure_offset`` seconds old,
    so our own accumulated displacement must be sampled at that same instant,
    otherwise the tracker interprets our own flicks as enemy motion.
    """

    __slots__ = ("_buf", "_maxlen")

    def __init__(self, maxlen: int = 512):
        self._buf: Deque[Tuple[float, float, float]] = deque(maxlen=maxlen)

    def push(self, ts: float, x: float, y: float) -> None:
        self._buf.append((ts, float(x), float(y)))

    def reset(self) -> None:
        self._buf.clear()

    def sample(self, ts: float) -> Tuple[float, float]:
        buf = self._buf
        if not buf:
            return 0.0, 0.0
        if ts >= buf[-1][0]:
            return buf[-1][1], buf[-1][2]
        if ts <= buf[0][0]:
            return buf[0][1], buf[0][2]
        # binary search would be nicer but the buffer is <= 512 entries and this
        # is called a few hundred times per second at most.
        lo, hi = 0, len(buf) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if buf[mid][0] <= ts:
                lo = mid
            else:
                hi = mid
        t0, x0, y0 = buf[lo]
        t1, x1, y1 = buf[lo + 1]
        span = t1 - t0
        if span <= 1e-9:
            return x1, y1
        a = (ts - t0) / span
        return x0 + (x1 - x0) * a, y0 + (y1 - y0) * a

    @property
    def span_s(self) -> float:
        if len(self._buf) < 2:
            return 0.0
        return self._buf[-1][0] - self._buf[0][0]


def smoothstep(edge0: float, edge1: float, x: float) -> float:
    if edge1 <= edge0:
        return 0.0 if x < edge0 else 1.0
    t = (x - edge0) / (edge1 - edge0)
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return t * t * (3.0 - 2.0 * t)


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else (hi if v > hi else v)


def hypot(x: float, y: float) -> float:
    return math.sqrt(x * x + y * y)


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def ema(prev: float, new: float, alpha: float) -> float:
    return prev + (new - prev) * alpha


def robust_median(values: Iterable[float]) -> float:
    vals = sorted(values)
    n = len(vals)
    if not n:
        return 0.0
    mid = n // 2
    return vals[mid] if n % 2 else 0.5 * (vals[mid - 1] + vals[mid])
