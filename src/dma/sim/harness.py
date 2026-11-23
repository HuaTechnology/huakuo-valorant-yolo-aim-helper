"""Closed-loop simulator: the regression gate for the control law.

Nothing here touches a camera or a serial port, yet it runs the *real* tracker,
policy, controller and fire logic against a plant with the measured properties of
the physical rig:

* capture at ``detect_hz`` with gaussian detection noise (measured ~2-3 px);
* loop latency ``latency_s`` split into "frame is stale" and "command is late";
* HID quantisation to integer counts with a carried remainder;
* occasional dropouts (the detector misses a frame, the tracker coasts).

That makes "did this change make aim better?" a number instead of an opinion, and
it is what lets ``aim2 autotune`` search parameters offline.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence

import numpy as np

from ..control.controller import ControlInput, LegacyAim, PredictiveAim
from ..control.fire import FireController, ImpactPreview
from ..control.policy import TargetPolicy
from ..core.config import AimCfg, FireCfg, PolicyCfg, TrackerCfg
from ..core.types import Box, Detection, Track
from ..perception.tracker import Tracker
from .world import Scenario


@dataclass(frozen=True)
class SimConfig:
    control_hz: int = 240
    detect_hz: int = 60
    latency_s: float = 0.033
    noise_px: float = 2.5
    dropout: float = 0.04
    seed: int = 11
    quantize: bool = True
    use_tracker: bool = True
    apply_s_override: float = 0.0   # simulate a wrong latency estimate
    debug: bool = False


@dataclass
class SimMetrics:
    controller: str
    scenario: str
    rms_px: float = 0.0
    p95_px: float = 0.0
    median_px: float = 0.0
    first_acq_s: float = 0.0
    tracking_rms_px: float = 0.0
    on_target: float = 0.0
    overshoot_px: float = 0.0
    max_dev_px: float = 0.0
    recovery_s: float = float("nan")
    jitter_px: float = 0.0
    shots: int = 0
    hits: int = 0
    saturated: int = 0
    ticks: int = 0

    @property
    def hit_rate(self) -> float:
        return self.hits / self.shots if self.shots else 0.0

    def as_row(self) -> Dict[str, float | str]:
        return {
            "controller": self.controller,
            "scenario": self.scenario,
            "rms_px": round(self.rms_px, 2),
            "p95_px": round(self.p95_px, 2),
            "track_rms": round(self.tracking_rms_px, 2),
            "first_acq_s": round(self.first_acq_s, 3),
            "on_target": round(self.on_target, 3),
            "max_dev_px": round(self.max_dev_px, 1),
            "recovery_s": (round(self.recovery_s, 3) if self.recovery_s == self.recovery_s else "nan"),
            "jitter_px": round(self.jitter_px, 2),
            "shots": self.shots,
            "hits": self.hits,
            "hit_rate": round(self.hit_rate, 3),
        }


def _default_policy() -> PolicyCfg:
    return PolicyCfg(fov_radius=1400.0, lock_radius=400.0, switch_margin=0.0, top_mask=0.0, bottom_mask=0.0)


def simulate(
    scenario: Scenario,
    controller_kind: str = "predictive",
    *,
    sim: SimConfig | None = None,
    aim_cfg: AimCfg | None = None,
    fire_cfg: FireCfg | None = None,
    tracker_cfg: TrackerCfg | None = None,
) -> SimMetrics:
    sim = sim or SimConfig()
    aim_cfg = aim_cfg or AimCfg()
    tracker_cfg = tracker_cfg or TrackerCfg()
    fire_cfg = fire_cfg or FireCfg(mode="auto", cooldown_s=0.12, dwell_s=0.03)

    dt = 1.0 / sim.control_hz
    detect_every = max(1, int(round(sim.control_hz / max(1, sim.detect_hz))))
    half = max(0, int(round(sim.latency_s * 0.5 / dt)))
    # at least one tick of application delay: a command written into the plant
    # schedule at index k would otherwise be overwritten before it is integrated
    apply_lag = max(1, half)
    m_measure = half
    ticks = int(round(scenario.duration_s * sim.control_hz))
    center = (scenario.cx0, scenario.cy0)

    tracker = Tracker(tracker_cfg)
    policy = TargetPolicy(_default_policy(), scenario.width, scenario.height)
    controller = PredictiveAim(aim_cfg) if controller_kind != "legacy" else LegacyAim(aim_cfg)
    fire = FireController(fire_cfg)

    cam: List[float] = [0.0] * (ticks + apply_lag + 4)
    cam_y: List[float] = [0.0] * (ticks + apply_lag + 4)
    schedule_x: List[float] = [0.0] * (ticks + apply_lag + 4)
    schedule_y: List[float] = [0.0] * (ticks + apply_lag + 4)
    residual = [0.0, 0.0]

    rng = np.random.default_rng(sim.seed)
    errors: List[float] = []
    applied: List[float] = []
    acq_t: Optional[float] = None
    max_after_acq = 0.0
    on_target = 0
    metric_ticks = 0
    hits = 0

    tracks: List[Track] = []
    target = None
    cam_meas = (0.0, 0.0)

    for k in range(ticks):
        t = k * dt
        # plant integration: commands issued ``apply_lag`` ticks ago land now,
        # and the camera keeps whatever it already accumulated
        cam[k] = (cam[k - 1] if k else 0.0) + schedule_x[k]
        cam_y[k] = (cam_y[k - 1] if k else 0.0) + schedule_y[k]

        pose = scenario.pose(t)
        true_screen = Box(
            pose.cx - pose.w / 2 - cam[k],
            pose.cy - pose.h / 2 - cam_y[k],
            pose.cx + pose.w / 2 - cam[k],
            pose.cy + pose.h / 2 - cam_y[k],
        )
        # error is measured to the *aim point* (the head point for body-only
        # models), not to the body centre: the controller is supposed to put the
        # crosshair there, so measuring the body centre would score a perfect
        # headshot as a 40 px miss.
        aim_y = pose.cy - pose.h * 0.39
        err = math.hypot(pose.cx - cam[k] - center[0], aim_y - cam_y[k] - center[1])
        if pose.visible:
            errors.append(err)
            metric_ticks += 1

        # ---- detection at detect_hz -----------------------------------
        if k % detect_every == 0 and k >= m_measure:
            t_meas = t - m_measure * dt
            p_meas = scenario.pose(t_meas)
            cx_meas = cam[k - m_measure]
            cy_meas = cam_y[k - m_measure]
            cam_meas = (cx_meas, cy_meas)
            if p_meas.visible and rng.random() >= sim.dropout:
                nx = rng.normal(0.0, sim.noise_px) if sim.noise_px else 0.0
                ny = rng.normal(0.0, sim.noise_px) if sim.noise_px else 0.0
                # ``p_meas`` is already a world coordinate; the tracker works in
                # the world frame, so no camera offset is added here (adding it
                # twice is the classic way to build a positive feedback loop)
                box = Box(
                    p_meas.cx - p_meas.w / 2 + nx,
                    p_meas.cy - p_meas.h / 2 + ny,
                    p_meas.cx + p_meas.w / 2 + nx,
                    p_meas.cy + p_meas.h / 2 + ny,
                )
                dets = [Detection(box=box, cls_id=0, cls_name="body", score=0.9)]
            else:
                dets = []
            tracks = tracker.update(dets, t_meas) if sim.use_tracker else _pseudo_tracks(dets, t_meas)
            target = policy.select(tracks, cam_meas, center, t)
            if target is None:
                controller.reset()

        if target is None:
            fire.update(None, None, 0.0, t)
            applied.append(0.0)
            continue

        lead = sim.latency_s + (t - (k - k % detect_every) * dt)
        inp = ControlInput(
            now=t,
            dt=dt,
            world_xy=target.aim_world,
            vel_xy=(target.track.vx, target.track.vy),
            cam_now=(cam[k], cam_y[k]),
            cam_meas=cam_meas,
            center=center,
            age_s=lead - sim.latency_s * 0.5,
            apply_s=(sim.apply_s_override or sim.latency_s * 0.5),
            sigma=target.track.sigma,
            coasting=target.coasting,
        )
        out = controller.step(inp)
        applied.append(math.hypot(out.dx, out.dy))

        # ---- HID quantisation with carried remainder ------------------
        if sim.quantize:
            residual[0] += out.dx
            residual[1] += out.dy
            qx, qy = int(round(residual[0])), int(round(residual[1]))
            residual[0] -= qx
            residual[1] -= qy
        else:
            qx, qy = out.dx, out.dy
        idx = min(k + apply_lag, len(schedule_x) - 1)
        schedule_x[idx] += qx
        schedule_y[idx] += qy

        # ---- fire ------------------------------------------------------
        box_impact = true_screen.translate(0.0, 0.0)
        preview = ImpactPreview(
            impact_xy=center,
            target_xy=(true_screen.cx, true_screen.cy),
            box=box_impact,
            residual=math.hypot(true_screen.cx - center[0], true_screen.cy - center[1]),
        )
        decision = fire.update(target, preview, target.track.sigma, t)
        if decision.fire:
            if box_impact.contains(center[0], center[1]):
                hits += 1

        if sim.debug and k % 24 == 0:
            print(
                f"k={k:4d} t={t:5.2f} p={pose.cx:7.1f} cam={cam[k]:7.1f} err={err:7.1f} "
                f"aim={target.aim_world[0]:7.1f} vx={target.track.vx:7.1f} u={out.dx:7.2f} "
                f"lead={lead*1000:4.1f}ms kp={out.kp:.2f} mode={out.mode.value}"
            )
        if pose.visible and true_screen.contains(center[0], center[1]):
            on_target += 1
        if acq_t is None and err <= aim_cfg.settle_px:
            acq_t = t
        if acq_t is not None and pose.visible:
            max_after_acq = max(max_after_acq, err)

    arr = np.asarray(errors) if errors else np.zeros(1)
    warm = arr[len(arr) // 3:] if len(arr) > 6 else arr
    jitter = float(np.std(applied[len(applied) // 2:])) if applied else 0.0
    metrics = SimMetrics(
        controller=controller_kind,
        scenario=scenario.name,
        rms_px=float(np.sqrt(np.mean(arr ** 2))),
        p95_px=float(np.percentile(arr, 95)),
        median_px=float(np.median(arr)),
        first_acq_s=acq_t if acq_t is not None else float("nan"),
        tracking_rms_px=float(np.sqrt(np.mean(warm ** 2))),
        on_target=on_target / max(1, metric_ticks),
        overshoot_px=float(max_after_acq),
        max_dev_px=float(np.max(arr)),
        recovery_s=_recovery(arr, 1.0 / sim.control_hz),
        jitter_px=jitter,
        shots=fire.shots,
        hits=hits,
        ticks=ticks,
    )
    return metrics


def _recovery(dev: np.ndarray, dt: float) -> float:
    """Mean time from a large deviation peak back to <10 px.

    This is the metric that matters after a flick or a dash: how long the assist
    stays off target because it kept pushing in the old direction.
    """
    if len(dev) < 20:
        return float("nan")
    times: List[float] = []
    i = 0
    n = len(dev)
    while i < n - 1:
        if dev[i] > 100 and dev[i] >= dev[max(0, i - 4):i + 5].max():
            j = i + 1
            while j < n and dev[j] >= 10.0:
                j += 1
            if j < n:
                times.append((j - i) * dt)
            i = j
        else:
            i += 1
    return float(np.mean(times)) if times else float("nan")


def _pseudo_tracks(dets: Sequence[Detection], ts: float) -> List[Track]:
    return [
        Track(
            tid=i + 1,
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
        for i, d in enumerate(dets)
    ]


def compare(
    scenarios: Sequence[str] | None = None,
    *,
    sim: SimConfig | None = None,
    aim_cfg: AimCfg | None = None,
    kinds: Sequence[str] = ("legacy", "predictive"),
) -> List[Dict[str, float | str]]:
    from .world import build

    names = list(scenarios or ["strafe_mid", "strafe_far", "jitter", "dash", "peek", "micro_drift"])
    rows: List[Dict[str, float | str]] = []
    for name in names:
        for kind in kinds:
            sc = build(name)
            # the v1 loop ran at capture rate with no sub-frame control
            cfg = sim or SimConfig()
            if kind == "legacy":
                cfg = SimConfig(
                    control_hz=cfg.detect_hz,
                    detect_hz=cfg.detect_hz,
                    latency_s=cfg.latency_s,
                    noise_px=cfg.noise_px,
                    dropout=cfg.dropout,
                    seed=cfg.seed,
                    quantize=cfg.quantize,
                    use_tracker=False,
                )
            rows.append(simulate(sc, kind, sim=cfg, aim_cfg=aim_cfg).as_row())
    return rows
