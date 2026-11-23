"""Dedicated capture thread with reconnect, timestamps and staleness tracking.

The only job of this thread is "read as fast as the card produces, keep the newest
frame, stamp it".  A downstream consumer that is slower than the card must never
cause a growing backlog: that backlog *is* input lag, and v1's ``cap.read()`` in
the main loop produced it whenever inference was slower than capture.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..core.config import CaptureCfg
from ..core.metrics import Metrics
from .sources import FrameSource


@dataclass(frozen=True, slots=True)
class FramePacket:
    ts: float
    seq: int
    frame: np.ndarray

    @property
    def shape(self):
        return self.frame.shape


class FrameGrabber:
    def __init__(
        self,
        source: FrameSource,
        cfg: CaptureCfg,
        metrics: Metrics | None = None,
        *,
        pace_fps: int = 0,
    ):
        self.source = source
        self.cfg = cfg
        self.metrics = metrics
        self.pace_fps = int(pace_fps)
        self._lock = threading.Lock()
        self._packet: Optional[FramePacket] = None
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._seq = 0
        self._fails = 0
        self.reconnects = 0
        self.read_errors = 0
        self.fps = 0.0
        self._t_prev = 0.0
        self._opened = False

    # ------------------------------------------------------------------
    def ensure_open(self) -> bool:
        if self._opened:
            return True
        self._opened = self.source.open()
        return self._opened

    def start(self) -> "FrameGrabber":
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="dma-grab", daemon=True)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    # ------------------------------------------------------------------
    def _open_with_retry(self) -> bool:
        if not self._opened:
            if self.source.open():
                self._opened = True
                return True
            return False
        return True

    def _run(self) -> None:
        period = 1.0 / self.pace_fps if self.pace_fps > 0 else 0.0
        next_tick = time.monotonic()
        while not self._stop.is_set():
            if not self._open_with_retry():
                if self.metrics is not None:
                    self.metrics.inc("capture.open_errors")
                time.sleep(max(0.2, float(self.cfg.reconnect_s)))
                self.reconnects += 1
                continue

            ok, frame = self.source.read()
            ts = time.monotonic()
            if not ok or frame is None:
                self._fails += 1
                self.read_errors += 1
                if self.metrics is not None:
                    self.metrics.inc("capture.read_errors")
                if self._fails >= 8:
                    # dead stream (unplugged card, HDMI mode change, game exit):
                    # close and reopen, do not keep a stale frame alive
                    self.source.close()
                    self._opened = False
                    self.reconnects += 1
                    self._fails = 0
                    if self.metrics is not None:
                        self.metrics.inc("capture.reconnects")
                    time.sleep(max(0.1, float(self.cfg.reconnect_s)))
                else:
                    time.sleep(0.002)
                continue

            self._fails = 0
            self._seq += 1
            with self._lock:
                self._packet = FramePacket(ts=ts, seq=self._seq, frame=frame)
            if self.metrics is not None:
                self.metrics.inc("capture.frames")
                if self._t_prev:
                    dt = ts - self._t_prev
                    if dt > 0:
                        inst = 1.0 / dt
                        self.fps = inst if self.fps == 0.0 else self.fps * 0.9 + inst * 0.1
                        self.metrics.observe("capture.interval_ms", dt * 1000.0)
                self._t_prev = ts
                self.metrics.set("capture.fps", self.fps)

            if period:
                next_tick += period
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()

    # ------------------------------------------------------------------
    def latest(self) -> Optional[FramePacket]:
        with self._lock:
            return self._packet

    @property
    def last_ts(self) -> float:
        with self._lock:
            pkt = self._packet
        return pkt.ts if pkt is not None else 0.0

    def stale_s(self) -> float:
        ts = self.last_ts
        return float("inf") if ts == 0.0 else time.monotonic() - ts

    def describe(self) -> dict:
        return {
            **self.source.describe(),
            "fps": round(self.fps, 2),
            "reconnects": self.reconnects,
            "read_errors": self.read_errors,
            "stale_s": round(self.stale_s(), 3),
        }
