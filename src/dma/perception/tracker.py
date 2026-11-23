"""Multi-object tracking with per-track Kalman filters.

Why this matters for aiming
---------------------------
The detector jitters by a few pixels per frame.  v1 fed that jitter straight into
the PID, so the derivative term amplified it and the controller spent its output
budget chasing noise (the "spring" everyone complains about).  A constant-velocity
Kalman filter gives the controller three things v1 never had:

1. a *filtered* target position (noise ~ sqrt(R/N) instead of R),
2. a *velocity* estimate with a known covariance, so lead compensation can be
   clamped to what the data actually supports,
3. an *identity*, so re-detections do not reset the integrator every time the
   class score flickers.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from ..core.config import TrackerCfg
from ..core.types import Box, Detection, Track

_EYE = np.eye(6, dtype=np.float64)


class KalmanBox:
    """State [cx, cy, vx, vy, w, h]; measurement [cx, cy, w, h]."""

    __slots__ = ("x", "P", "q", "r_pos", "r_size", "ts", "initialized")

    def __init__(
        self,
        box: Box,
        ts: float,
        accel_noise: float = 3000.0,
        meas_noise: float = 3.5,
        size_noise: float = 6.0,
    ):
        self.x = np.array([box.cx, box.cy, 0.0, 0.0, max(1.0, box.w), max(1.0, box.h)], dtype=np.float64)
        # Velocity starts at 0 with a deliberately loose prior: we would rather
        # learn the enemy's speed in three frames than drag a wrong assumption
        # along for a second.
        self.P = np.diag([meas_noise ** 2, meas_noise ** 2, 400.0 ** 2, 400.0 ** 2, size_noise ** 2, size_noise ** 2])
        # white-noise-acceleration model: q is sigma_a^2 in px^2/s^4
        self.q = float(accel_noise) ** 2
        self.r_pos = float(meas_noise) ** 2
        self.r_size = float(size_noise) ** 2
        self.ts = float(ts)
        self.initialized = False

    # -- matrices ----------------------------------------------------------
    @staticmethod
    def _F(dt: float) -> np.ndarray:
        F = np.eye(6)
        F[0, 2] = dt
        F[1, 3] = dt
        return F

    def _Q(self, dt: float) -> np.ndarray:
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        Q = np.zeros((6, 6))
        Q[0, 0] = Q[1, 1] = dt4 / 4.0
        Q[0, 2] = Q[2, 0] = Q[1, 3] = Q[3, 1] = dt3 / 2.0
        Q[2, 2] = Q[3, 3] = dt2
        Q[4, 4] = Q[5, 5] = dt2
        return Q * self.q

    # -- filter ------------------------------------------------------------
    def predict(self, ts: float) -> None:
        dt = max(1e-3, min(float(ts) - self.ts, 0.5))
        F = self._F(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self._Q(dt)
        self.x[4] = max(1.0, self.x[4])
        self.x[5] = max(1.0, self.x[5])
        self.ts = float(ts)

    def update(self, box: Box, score: float = 1.0) -> None:
        z = np.array([box.cx, box.cy, max(1.0, box.w), max(1.0, box.h)], dtype=np.float64)
        H = np.zeros((4, 6))
        H[0, 0] = H[1, 1] = H[2, 4] = H[3, 5] = 1.0
        # scale measurement noise with 1/score: a 0.3-confidence box is trusted less
        s = max(0.05, min(1.0, score))
        R = np.diag([self.r_pos, self.r_pos, self.r_size, self.r_size]) / (0.4 + 0.6 * s)
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:  # pragma: no cover - pathological
            return
        self.x = self.x + K @ y
        I_KH = _EYE - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T   # Joseph form: stays PSD
        self.initialized = True

    # -- accessors ---------------------------------------------------------
    @property
    def box(self) -> Box:
        cx, cy, _, _, w, h = self.x
        return Box(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)

    @property
    def vel(self) -> Tuple[float, float]:
        return float(self.x[2]), float(self.x[3])

    @property
    def sigma(self) -> float:
        return float(math.sqrt(max(0.0, self.P[0, 0] + self.P[1, 1])))


@dataclass
class _TrackState:
    tid: int
    cls_id: int
    cls_name: str
    kf: KalmanBox
    hits: int = 1
    misses: int = 0
    age: int = 1
    score: float = 1.0

    def to_public(self) -> Track:
        return Track(
            tid=self.tid,
            cls_id=self.cls_id,
            cls_name=self.cls_name,
            box=self.kf.box,
            vx=self.kf.vel[0],
            vy=self.kf.vel[1],
            sigma=self.kf.sigma,
            hits=self.hits,
            misses=self.misses,
            age=self.age,
            score=self.score,
            last_ts=self.kf.ts,
        )


class Tracker:
    """Greedy IoU association; deterministic and dependency free."""

    def __init__(self, cfg: TrackerCfg):
        self.cfg = cfg
        self._tracks: List[_TrackState] = []
        self._next_id = 1
        self._last_ts = 0.0

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1

    @property
    def count(self) -> int:
        return len(self._tracks)

    # ------------------------------------------------------------------
    def update(self, detections: Sequence[Detection], ts: float) -> List[Track]:
        cfg = self.cfg
        if not cfg.enabled:
            return [
                Track(
                    tid=self._next_id + i,
                    cls_id=d.cls_id,
                    cls_name=d.cls_name,
                    box=d.box,
                    vx=0.0,
                    vy=0.0,
                    sigma=10.0,
                    hits=1,
                    misses=0,
                    age=1,
                    score=d.score,
                    last_ts=ts,
                )
                for i, d in enumerate(detections)
            ]

        for trk in self._tracks:
            trk.kf.predict(ts)

        pairs: List[Tuple[float, int, int]] = []
        for ti, trk in enumerate(self._tracks):
            tbox = trk.kf.box
            for di, det in enumerate(detections):
                if det.cls_id != trk.cls_id:
                    continue
                iou = tbox.iou(det.box)
                if iou >= cfg.iou_thr:
                    pairs.append((iou, ti, di))
        pairs.sort(key=lambda p: (-p[0], p[1], p[2]))

        used_t: set[int] = set()
        used_d: set[int] = set()
        for _iou, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            trk = self._tracks[ti]
            det = detections[di]
            trk.kf.update(det.box, det.score)
            trk.hits += 1
            trk.age += 1
            trk.misses = 0
            trk.score = det.score
            used_t.add(ti)
            used_d.add(di)

        survivors: List[_TrackState] = []
        for ti, trk in enumerate(self._tracks):
            if ti in used_t:
                survivors.append(trk)
                continue
            trk.misses += 1
            trk.age += 1
            if trk.misses <= cfg.max_age:
                survivors.append(trk)

        for di, det in enumerate(detections):
            if di in used_d:
                continue
            kf = KalmanBox(det.box, ts, cfg.accel_noise, cfg.meas_noise, cfg.size_noise)
            survivors.append(
                _TrackState(
                    tid=self._next_id,
                    cls_id=det.cls_id,
                    cls_name=det.cls_name,
                    kf=kf,
                    hits=1,
                    misses=0,
                    age=1,
                    score=det.score,
                )
            )
            self._next_id += 1

        self._tracks = survivors
        self._last_ts = ts
        return [t.to_public() for t in survivors]

    # ------------------------------------------------------------------
    def predicted_box(self, track: Track, ts: float) -> Box:
        """Linear extrapolation of a published track (used for coasting)."""
        dt = max(0.0, float(ts) - float(track.last_ts))
        return track.box.translate(track.vx * dt, track.vy * dt)
