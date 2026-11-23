"""Online identification of the end-to-end loop delay.

The plant, in screen coordinates, is a delayed integrator::

    e_k = p_k - c_{k-L}          (target position minus camera position)
    Δe_k = Δp_k - u_{k-m}        (m = L / frame_time)

so for the correct delay ``m`` the residual ``Δe_k + u_{k-m}`` contains only the
enemy's own motion.  For every other ``m`` it also contains our own movement, so
the residual explodes.  That is a one-line system identification problem and it
is solved here on *detection* events (not control ticks) because that is where
the delay is observable.

Why not use the per-stage timers only?  They measure grab→infer→write, roughly
5-15 ms.  They cannot see the two stages that dominate: the capture card /
display pipeline (1-3 frames) and the game engine applying the HID report on its
next input tick.  Those are only observable in closed loop, which is what this
estimator does.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Tuple

import numpy as np

from ..core.clock import clamp


@dataclass(frozen=True, slots=True)
class LatencyEstimate:
    loop_s: float
    frames: float
    confidence: float
    samples: int


class LatencyEstimator:
    """Passive identification from the closed loop; safe to run permanently."""

    def __init__(
        self,
        prior_s: float = 0.033,
        window: int = 45,
        min_conf: float = 0.2,
        max_lag: int = 10,
        smoothing: float = 0.25,
    ):
        self.prior_s = float(prior_s)
        self.loop_s = float(prior_s)
        self.window = max(8, int(window))
        self.min_conf = float(min_conf)
        self.max_lag = max(2, int(max_lag))
        self.smoothing = float(smoothing)
        self.confidence = 0.0
        self.frames = 0.0
        self.rejected = 0
        self._err: Deque[Tuple[float, float, float]] = deque(maxlen=self.window)
        self._win: Deque[Tuple[float, float]] = deque(maxlen=self.window)
        self._pending = [0.0, 0.0]

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._err.clear()
        self._win.clear()
        self._pending = [0.0, 0.0]
        self.confidence = 0.0
        self.rejected = 0

    def add_motion(self, dx: float, dy: float) -> None:
        """Called for every commanded displacement (control tick)."""
        self._pending[0] += float(dx)
        self._pending[1] += float(dy)

    def on_detection(self, ts: float, err_x: float, err_y: float) -> None:
        """Called once per *new* detection of the tracked target."""
        self._err.append((float(ts), float(err_x), float(err_y)))
        self._win.append((self._pending[0], self._pending[1]))
        self._pending = [0.0, 0.0]
        if len(self._err) >= 12:
            self._estimate()

    # ------------------------------------------------------------------
    def _estimate(self) -> None:
        err = list(self._err)
        win = list(self._win)
        n = len(err)
        if n < 12:
            return

        dts = [err[i][0] - err[i - 1][0] for i in range(1, n) if err[i][0] > err[i - 1][0]]
        if not dts:
            return
        frame_dt = float(np.median(dts))
        if frame_dt <= 1e-4:
            return

        max_lag = min(self.max_lag, max(2, n - 8))
        residuals: List[Tuple[int, float]] = []
        for m in range(0, max_lag + 1):
            vals: List[float] = []
            for k in range(1, n):
                j = k - m
                if j < 0:
                    break
                dex = err[k][1] - err[k - 1][1]
                dey = err[k][2] - err[k - 1][2]
                vals.append(abs(dex + win[j][0]) + abs(dey + win[j][1]))
            if len(vals) >= 10:
                residuals.append((m, float(np.median(vals))))
        if not residuals:
            return

        best_m, best_cost = min(residuals, key=lambda p: p[1])
        costs = [c for _, c in residuals]
        overall = float(np.median(costs))
        conf = 0.0 if overall <= 1e-6 else clamp(1.0 - best_cost / overall, 0.0, 1.0)

        self.frames = float(best_m)
        self.confidence = conf

        # --- sanity gates -------------------------------------------------
        # A real loop cannot be faster than one frame: the detector only reports
        # after capture + inference, and the command only lands on the next input
        # tick.  Letting the estimate collapse to ~0 turns the controller into a
        # plain P loop on a delayed integrator, i.e. a limit cycle around the
        # target — exactly the "oscillates but never hits" symptom.
        floor_s = max(0.020, frame_dt * 1.2)
        candidate = clamp(best_m * frame_dt, floor_s, 0.25)
        too_few = n < 40
        jump = abs(candidate - self.loop_s) > max(0.05, 0.6 * self.loop_s)
        if conf < self.min_conf or too_few or jump:
            self.rejected += 1
            self.confidence = conf * 0.5 if conf >= self.min_conf else conf
            return
        self.loop_s = float(clamp((1.0 - self.smoothing) * self.loop_s + self.smoothing * candidate,
                                  floor_s, 0.25))

    # ------------------------------------------------------------------
    def estimate(self) -> LatencyEstimate:
        return LatencyEstimate(
            loop_s=float(self.loop_s),
            frames=float(self.frames),
            confidence=float(self.confidence),
            samples=len(self._err),
        )

    def report(self, split: float = 0.5) -> dict:
        split = clamp(float(split), 0.0, 1.0)
        return {
            "loop_ms": round(self.loop_s * 1000.0, 2),
            "frames": round(self.frames, 2),
            "confidence": round(self.confidence, 3),
            "measure_ms": round(self.loop_s * split * 1000.0, 2),
            "apply_ms": round(self.loop_s * (1.0 - split) * 1000.0, 2),
            "lead_ms": round(self.loop_s * 1000.0, 2),
            "rejected": int(self.rejected),
        }
