"""Replay a recorded session through a different controller (closed loop).

The trick that makes this possible: the enemy's *world* trajectory in a recording
does not depend on what our crosshair was doing.  Once the recording stores the
camera sample next to each detection, the world track can be reconstructed
exactly, and any controller can be flown through that same scenario offline —
same enemy, same latency, different law.  This is how a tuning change is validated
against real gameplay instead of against a synthetic strafe.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from ..control.controller import ControlInput, LegacyAim, PredictiveAim
from ..control.policy import TargetPolicy
from ..core.config import AimCfg, PolicyCfg, TrackerCfg
from ..core.types import Box, Detection
from ..perception.tracker import Tracker
from .recorder import RecordedSession


@dataclass
class ReplayResult:
    controller: str
    frames: int
    rms_px: float
    p95_px: float
    on_target: float
    jitter_px: float
    travel_px: float
    mean_latency_ms: float = 0.0

    def as_row(self) -> Dict[str, object]:
        return {
            "controller": self.controller,
            "frames": self.frames,
            "rms_px": round(self.rms_px, 2),
            "p95_px": round(self.p95_px, 2),
            "on_target": round(self.on_target, 3),
            "jitter_px": round(self.jitter_px, 2),
            "travel_px": round(self.travel_px, 1),
        }


def _session_settings(session: RecordedSession):
    """Rebuild the Settings the recording was made with (header, schema 2)."""
    from ..core.config import Settings, from_dict

    raw = (session.header or {}).get("settings")
    if isinstance(raw, dict):
        try:
            return from_dict(raw)
        except Exception:
            pass
    return Settings()


def _class_names(session: RecordedSession) -> dict:
    from ..tuning.calibration import class_names

    return class_names(session)


def replay(
    session: RecordedSession,
    controller_kind: str = "predictive",
    *,
    aim_cfg: AimCfg | None = None,
    tracker_cfg: TrackerCfg | None = None,
    control_hz: int = 240,
    head_ratio: float | None = None,
) -> ReplayResult:
    base = _session_settings(session)
    aim_cfg = aim_cfg or base.aim
    tracker_cfg = tracker_cfg or base.tracker
    head_ratio = base.policy.head_ratio if head_ratio is None else head_ratio
    names = _class_names(session)
    pol_cfg = PolicyCfg(fov_radius=1400.0, lock_radius=400.0, switch_margin=0.0,
                        top_mask=0.0, bottom_mask=0.0, head_ratio=head_ratio)
    frames = sorted(session.frames, key=lambda f: f["ts"])
    if len(frames) < 10:
        raise ValueError("session too short, at least 10 frames are required")

    tracker = Tracker(tracker_cfg)
    policy = TargetPolicy(pol_cfg, 1280, 720, head_mode=base.vision.head_mode)
    controller = PredictiveAim(aim_cfg) if controller_kind != "legacy" else LegacyAim(aim_cfg)

    center = (640.0, 360.0)
    cam = [0.0, 0.0]
    errors: List[float] = []
    applied: List[float] = []
    on_target = 0
    travel = 0.0
    latency_ms: List[float] = []

    inp_dx_sum = 0.0
    inp_dy_sum = 0.0
    latency = aim_cfg.latency_prior_s
    measure_offset = latency * aim_cfg.latency_split
    apply_s = latency * (1.0 - aim_cfg.latency_split)
    dt = 1.0 / max(30, control_hz)

    for frame in frames:
        ts = float(frame["ts"])
        cam_meas = tuple(frame.get("cam", (0.0, 0.0)))
        world: List[Detection] = []
        for x1, y1, x2, y2, cls_id, score in frame.get("dets", []):
            world.append(
                Detection(
                    box=Box(x1 + cam_meas[0], y1 + cam_meas[1], x2 + cam_meas[0], y2 + cam_meas[1]),
                    cls_id=int(cls_id),
                    cls_name=names.get(int(cls_id), "body"),
                    score=float(score),
                )
            )
        tracks = tracker.update(world, ts)
        target = policy.select(tracks, (cam[0], cam[1]), center, ts)
        if target is None:
            continue

        lead = latency + measure_offset
        inp = ControlInput(
            now=ts,
            dt=dt if controller_kind != "legacy" else max(dt, 1.0 / 60.0),
            world_xy=target.aim_world,
            vel_xy=(target.track.vx, target.track.vy),
            cam_now=(cam[0], cam[1]),
            cam_meas=(cam_meas[0], cam_meas[1]),
            center=center,
            age_s=measure_offset,
            apply_s=apply_s,
            sigma=target.track.sigma,
        )
        out = controller.step(inp)
        cam[0] += out.dx
        cam[1] += out.dy
        inp_dx_sum, inp_dy_sum = out.dx, out.dy
        travel += math.hypot(out.dx, out.dy)
        applied.append(math.hypot(out.dx, out.dy))

        err_x = target.aim_world[0] - cam[0] - center[0]
        err_y = target.aim_world[1] - cam[1] - center[1]
        errors.append(math.hypot(err_x, err_y))
        # screen_box is already in screen coordinates (policy subtracted cam)
        if target.screen_box.translate(-inp_dx_sum, -inp_dy_sum).contains(center[0], center[1]):
            on_target += 1
        latency_ms.append(lead * 1000.0)

    arr = np.asarray(errors) if errors else np.zeros(1)
    return ReplayResult(
        controller=controller_kind,
        frames=len(frames),
        rms_px=float(np.sqrt(np.mean(arr ** 2))),
        p95_px=float(np.percentile(arr, 95)),
        on_target=on_target / max(1, len(errors)),
        jitter_px=float(np.std(applied)) if applied else 0.0,
        travel_px=travel,
        mean_latency_ms=float(np.mean(latency_ms)) if latency_ms else 0.0,
    )


def replay_compare(session: RecordedSession, **kw) -> List[Dict[str, object]]:
    return [replay(session, k, **kw).as_row() for k in ("legacy", "predictive")]
