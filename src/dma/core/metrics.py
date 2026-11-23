"""Low overhead metrics primitives.

The v1 code printed ``fps / inference ms`` into a JPEG every two seconds and that
was the entire observability story.  You cannot operate a real-time control loop
like that: you need per-stage percentiles, counters that survive a restart, and a
machine readable dump a dashboard can poll.

Design notes
------------
* ``RollingStats`` is O(1) per sample: it keeps a fixed size mmap-like ring and
  computes percentiles on demand (only the dashboard asks, once per second).
* Everything is thread safe; the control thread must never block on metrics, so
  updates take a small lock and the arrays are written in place.
* ``snapshot()`` returns plain dicts so it can be ``json.dumps``-ed straight to
  disk or served over HTTP.
"""
from __future__ import annotations

import json
import math
import threading
import time
from typing import Dict, Iterable, List


class RollingStats:
    """Fixed window of float samples with percentile queries."""

    __slots__ = ("capacity", "_buf", "_n", "_idx", "_lock", "_sum", "_sumsq", "_min", "_max")

    def __init__(self, capacity: int = 240):
        self.capacity = max(2, int(capacity))
        self._buf: List[float] = [0.0] * self.capacity
        self._n = 0
        self._idx = 0
        self._lock = threading.Lock()
        self._sum = 0.0
        self._sumsq = 0.0
        self._min = math.inf
        self._max = -math.inf

    def add(self, value: float) -> None:
        v = float(value)
        if not math.isfinite(v):
            return
        with self._lock:
            if self._n == self.capacity:
                old = self._buf[self._idx]
                self._sum -= old
                self._sumsq -= old * old
            else:
                self._n += 1
            self._buf[self._idx] = v
            self._idx = (self._idx + 1) % self.capacity
            self._sum += v
            self._sumsq += v * v
            if v < self._min:
                self._min = v
            if v > self._max:
                self._max = v

    # -- readers -----------------------------------------------------------
    def _values(self) -> List[float]:
        with self._lock:
            if self._n < self.capacity:
                return self._buf[: self._n]
            return list(self._buf)

    @property
    def count(self) -> int:
        with self._lock:
            return self._n

    @property
    def last(self) -> float:
        with self._lock:
            if not self._n:
                return 0.0
            return self._buf[(self._idx - 1) % self.capacity]

    @property
    def mean(self) -> float:
        with self._lock:
            return self._sum / self._n if self._n else 0.0

    @property
    def std(self) -> float:
        with self._lock:
            if self._n < 2:
                return 0.0
            m = self._sum / self._n
            var = max(0.0, self._sumsq / self._n - m * m)
            return math.sqrt(var)

    def percentile(self, q: float) -> float:
        vals = self._values()
        if not vals:
            return 0.0
        vals.sort()
        if len(vals) == 1:
            return vals[0]
        pos = (len(vals) - 1) * max(0.0, min(1.0, q))
        lo = int(math.floor(pos))
        hi = min(lo + 1, len(vals) - 1)
        frac = pos - lo
        return vals[lo] * (1.0 - frac) + vals[hi] * frac

    def summary(self) -> Dict[str, float]:
        vals = self._values()
        if not vals:
            return {"n": 0, "mean": 0.0, "p50": 0.0, "p95": 0.0, "p99": 0.0, "max": 0.0}
        srt = sorted(vals)
        return {
            "n": len(srt),
            "mean": round(sum(srt) / len(srt), 3),
            "p50": round(self.percentile(0.50), 3),
            "p95": round(self.percentile(0.95), 3),
            "p99": round(self.percentile(0.99), 3),
            "max": round(srt[-1], 3),
            "min": round(srt[0], 3),
            "std": round(self.std, 3),
            "last": round(self.last, 3),
        }

    def reset(self) -> None:
        with self._lock:
            self._buf = [0.0] * self.capacity
            self._n = 0
            self._idx = 0
            self._sum = self._sumsq = 0.0
            self._min, self._max = math.inf, -math.inf


class Metrics:
    """Named counters, gauges and rolling histograms."""

    def __init__(self, default_capacity: int = 240):
        self._lock = threading.Lock()
        self._stats: Dict[str, RollingStats] = {}
        self._counters: Dict[str, float] = {}
        self._gauges: Dict[str, float] = {}
        self._capacity = default_capacity
        self._started = time.monotonic()

    # -- update ------------------------------------------------------------
    def observe(self, name: str, value: float, capacity: int | None = None) -> None:
        st = self._stats.get(name)
        if st is None:
            with self._lock:
                st = self._stats.get(name)
                if st is None:
                    st = RollingStats(capacity or self._capacity)
                    self._stats[name] = st
        st.add(value)

    def inc(self, name: str, amount: float = 1.0) -> None:
        with self._lock:
            self._counters[name] = self._counters.get(name, 0.0) + float(amount)

    def set(self, name: str, value: float) -> None:
        with self._lock:
            self._gauges[name] = float(value)

    def get_counter(self, name: str) -> float:
        with self._lock:
            return self._counters.get(name, 0.0)

    def gauge(self, name: str) -> float:
        with self._lock:
            return self._gauges.get(name, 0.0)

    # -- read --------------------------------------------------------------
    def stats_summary(self, prefix: str | Iterable[str] | None = None) -> Dict[str, Dict[str, float]]:
        names = list(self._stats)
        if isinstance(prefix, str):
            names = [n for n in names if n.startswith(prefix)]
        elif prefix is not None:
            want = set(prefix)
            names = [n for n in names if n in want]
        return {n: self._stats[n].summary() for n in sorted(names)}

    def counters(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._counters)

    def gauges(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._gauges)

    def snapshot(self) -> Dict[str, object]:
        return {
            "uptime_s": round(time.monotonic() - self._started, 3),
            "counters": {k: round(v, 3) for k, v in self.counters().items()},
            "gauges": {k: round(v, 6) for k, v in self.gauges().items()},
            "stats": self.stats_summary(),
        }

    def to_json(self) -> str:
        return json.dumps(self.snapshot(), ensure_ascii=False, separators=(",", ":"))

    def to_prometheus(self) -> str:
        """Minimal text exposition format: counters as _total, gauges raw."""
        lines: List[str] = []
        for name, value in sorted(self.counters().items()):
            key = f"dma_{name.replace('.', '_')}_total"
            lines.append(f"# TYPE {key} counter")
            lines.append(f"{key} {value:g}")
        for name, value in sorted(self.gauges().items()):
            key = f"dma_{name.replace('.', '_')}"
            lines.append(f"# TYPE {key} gauge")
            lines.append(f"{key} {value:g}")
        for name in sorted(self._stats):
            s = self._stats[name].summary()
            key = f"dma_{name.replace('.', '_')}"
            for q in ("p50", "p95", "p99"):
                lines.append(f"# TYPE {key}_{q} gauge")
                lines.append(f"{key}_{q} {s[q]:g}")
        return "\n".join(lines) + "\n"


class Stopwatch:
    """Context manager that records elapsed milliseconds into Metrics."""

    __slots__ = ("_m", "_name", "_t0")

    def __init__(self, metrics: Metrics | None, name: str):
        self._m = metrics
        self._name = name
        self._t0 = 0.0

    def __enter__(self) -> "Stopwatch":
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, *exc: object) -> None:
        if self._m is not None:
            self._m.observe(self._name, (time.perf_counter() - self._t0) * 1000.0)
