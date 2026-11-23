from __future__ import annotations

import math

import pytest

from dma.control.controller import ControlInput, PredictiveAim
from dma.control.fire import FireController, ImpactPreview
from dma.control.latency import LatencyEstimator
from dma.control.policy import TargetPolicy
from dma.core.config import AimCfg, FireCfg, PolicyCfg, TrackerCfg
from dma.core.types import Box, Detection, Track
from dma.perception.tracker import KalmanBox, Tracker


# ---------------------------------------------------------------------------
# tracker
# ---------------------------------------------------------------------------
def _det(cx, cy, w=46, h=100, cls=0, score=0.9):
    return Detection(box=Box(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2),
                     cls_id=cls, cls_name="body", score=score)


def test_tracker_keeps_identity_and_estimates_velocity():
    tr = Tracker(TrackerCfg())
    tid = None
    for k in range(20):
        tracks = tr.update([_det(100 + 3.0 * k, 200)], k / 60.0)
        assert tracks
        if tid is None:
            tid = tracks[0].tid
        assert tracks[0].tid == tid
    assert tracks[0].vx == pytest.approx(180.0, rel=0.35)


def test_tracker_coasts_then_drops():
    tr = Tracker(TrackerCfg(max_age=3))
    tr.update([_det(300, 300)], 0.0)
    for k in range(1, 3):
        tracks = tr.update([], k / 60.0)
        assert tracks and tracks[0].misses >= 1
    for k in range(3, 7):
        tracks = tr.update([], k / 60.0)
    assert not tracks


def test_tracker_does_not_merge_different_classes():
    tr = Tracker(TrackerCfg())
    tracks = tr.update([_det(100, 100, cls=0), _det(100, 100, cls=1)], 0.0)
    assert len(tracks) == 2


def test_kalman_uncertainty_grows_when_measurements_stop():
    kf = KalmanBox(Box(0, 0, 40, 80), 0.0)
    for k in range(1, 6):
        kf.update(Box(0, 0, 40, 80))
        kf.predict(k / 60.0)
    sigma_a = kf.sigma
    for k in range(6, 20):
        kf.predict(k / 60.0)
    assert kf.sigma > sigma_a


# ---------------------------------------------------------------------------
# controller
# ---------------------------------------------------------------------------
def _inp(**kw):
    base = dict(now=0.0, dt=1 / 240, world_xy=(640.0, 360.0), vel_xy=(0.0, 0.0),
                cam_now=(0.0, 0.0), cam_meas=(0.0, 0.0), center=(640.0, 360.0),
                age_s=0.016, apply_s=0.0165)
    base.update(kw)
    return ControlInput(**base)


def test_controller_direction_and_scale():
    c = PredictiveAim(AimCfg())
    out = c.step(_inp(world_xy=(740.0, 360.0)))
    assert out.dx > 0                      # target to the right -> move right
    assert out.dy == pytest.approx(0.0, abs=1e-6)
    # one tick may only deliver a fraction of the window's displacement
    assert 0 < out.dx < 100 * 0.85


def test_controller_deadzone_holds():
    c = PredictiveAim(AimCfg(deadzone=3.0, deadzone_exit=6.0))
    c.step(_inp(world_xy=(642.0, 360.0)))
    out = c.step(_inp(world_xy=(642.0, 360.0)))
    assert out.dx == 0.0 and out.mode.value == "settle"


def test_controller_saturates_and_stays_stable():
    c = PredictiveAim(AimCfg(max_step=100.0))
    out = c.step(_inp(world_xy=(640.0 + 5000.0, 360.0)))
    assert out.saturated
    assert abs(out.dx) <= 100.0


def test_controller_rate_normalisation():
    """The same gains must produce the same closed-loop behaviour at 60 and 240 Hz."""
    def run(hz):
        c = PredictiveAim(AimCfg())
        cam = 0.0
        dt = 1 / hz
        lag = max(1, int(round(0.0165 / dt)))
        sched = [0.0] * 400
        errors = []
        for k in range(320):
            cam += sched[k]
            target = 640.0 + 60.0 * math.sin(k * dt * 3.0)
            out = c.step(_inp(dt=dt, world_xy=(target, 360.0), cam_now=(cam, 0.0)))
            idx = min(k + lag, len(sched) - 1)
            sched[idx] += out.dx
            errors.append(abs(target - cam - 640.0))
        return sum(errors[80:]) / len(errors[80:])

    e60, e240 = run(60), run(240)
    assert e240 < e60 * 1.6 + 2.0
    assert e60 < 40.0 and e240 < 40.0


def test_controller_feedforward_tracks_constant_velocity():
    c = PredictiveAim(AimCfg())
    cam = 0.0
    dt = 1 / 240
    sched = [0.0] * 600
    lag = 4
    errs = []
    for k in range(560):
        cam += sched[k]
        t = k * dt
        world = 700.0 + 300.0 * t
        out = c.step(_inp(dt=dt, world_xy=(world, 360.0), vel_xy=(300.0, 0.0), cam_now=(cam, 0.0)))
        sched[min(k + lag, len(sched) - 1)] += out.dx
        errs.append(abs(world - cam - 640.0))
    assert errs[-1] < 6.0


def test_controller_never_exceeds_step_limit():
    c = PredictiveAim(AimCfg(max_step=40.0))
    for x in (1000, -500, 3000, 0):
        out = c.step(_inp(world_xy=(640.0 + x, 360.0)))
        assert abs(out.dx) <= 40.0 + 1e-6


# ---------------------------------------------------------------------------
# policy
# ---------------------------------------------------------------------------
def _track(tid, cx, cy, vx=0.0, cls="body", misses=0, hits=3, w=46, h=100, ts=0.0):
    return Track(tid=tid, cls_id=0, cls_name=cls,
                 box=Box(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2),
                 vx=vx, vy=0.0, sigma=2.0, hits=hits, misses=misses, age=hits,
                 score=0.9, last_ts=ts)


def test_policy_prefers_head_inside_body():
    pol = TargetPolicy(PolicyCfg(), 1280, 720)
    body = _track(1, 640, 360)
    head = _track(2, 640, 320, cls="head", w=20, h=20)
    target = pol.select([body, head], (0.0, 0.0), (640.0, 360.0), 0.0)
    assert target is not None
    assert target.is_head
    assert target.aim_world[1] == pytest.approx(320.0)


def test_policy_never_picks_a_coasting_track_over_a_visible_one():
    pol = TargetPolicy(PolicyCfg(), 1280, 720)
    coasting = _track(1, 640, 360, misses=1)
    visible = _track(2, 900, 360)
    target = pol.select([coasting, visible], (0.0, 0.0), (640.0, 360.0), 0.0)
    assert target is not None
    assert target.tid == 2
    assert not target.coasting


def test_policy_coasts_when_nothing_visible():
    pol = TargetPolicy(PolicyCfg(coast_s=0.2), 1280, 720)
    t0 = 0.0
    pol.select([_track(1, 640, 360, vx=100.0, ts=t0)], (0.0, 0.0), (640.0, 360.0), t0)
    target = pol.select([], (0.0, 0.0), (640.0, 360.0), 0.1)
    assert target is not None and target.coasting
    assert target.aim_world[0] > 640.0     # dead reckoned forward
    assert pol.select([], (0.0, 0.0), (640.0, 360.0), 0.5) is None


def test_policy_switch_hysteresis():
    pol = TargetPolicy(PolicyCfg(switch_margin=40.0, fov_radius=900.0), 1280, 720)
    first = pol.select([_track(1, 700, 360)], (0.0, 0.0), (640.0, 360.0), 0.0)
    assert first.tid == 1
    # 71.6 vs 39.9 + 40 margin: the closer target must not steal the lock
    second = pol.select([_track(1, 700, 360), _track(2, 646, 360)], (0.0, 0.0), (640.0, 360.0), 0.1)
    assert second.tid == 1
    # a target that is clearly closer does take over
    third = pol.select([_track(1, 760, 360), _track(2, 646, 360)], (0.0, 0.0), (640.0, 360.0), 0.2)
    assert third.tid == 2


def test_policy_rejects_head_cropped_by_top_mask():
    pol = TargetPolicy(PolicyCfg(top_mask=0.2), 1280, 720)
    t = _track(1, 640, 30)
    assert pol.select([t], (0.0, 0.0), (640.0, 360.0), 0.0) is None


# ---------------------------------------------------------------------------
# fire
# ---------------------------------------------------------------------------
def test_fire_requires_dwell_and_cooldown():
    fc = FireController(FireCfg(mode="auto", dwell_s=0.05, cooldown_s=0.2, tap_s=0.02, radius=20.0))
    from dma.control.policy import TargetState
    ts = TargetState(track=_track(1, 640, 360), aim_world=(640.0, 360.0),
                     box_world=Box(620, 310, 660, 410), screen_xy=(640.0, 360.0),
                     screen_box=Box(620, 310, 660, 410), is_head=False, score=1.0)
    preview = ImpactPreview(impact_xy=(640.0, 360.0), target_xy=(641.0, 360.0),
                            box=Box(620, 310, 660, 410), residual=1.0)
    assert not fc.update(ts, preview, 2.0, 0.00).fire      # dwell starts
    assert not fc.update(ts, preview, 2.0, 0.03).fire      # still settling
    assert fc.update(ts, preview, 2.0, 0.06).fire          # dwell met
    assert not fc.update(ts, preview, 2.0, 0.10).fire      # cooldown
    assert fc.update(ts, preview, 2.0, 0.30).fire


def test_fire_blocks_off_target_and_coasting():
    fc = FireController(FireCfg(mode="auto", dwell_s=0.0, cooldown_s=0.0, radius=5.0))
    from dma.control.policy import TargetState
    ts = TargetState(track=_track(1, 640, 360), aim_world=(640.0, 360.0),
                     box_world=Box(620, 310, 660, 410), screen_xy=(640.0, 360.0),
                     screen_box=Box(620, 310, 660, 410), is_head=False, score=1.0)
    far = ImpactPreview(impact_xy=(640.0, 360.0), target_xy=(700.0, 360.0),
                        box=Box(680, 310, 720, 410), residual=60.0)
    assert not fc.update(ts, far, 2.0, 0.0).fire
    coast = TargetState(track=_track(1, 640, 360), aim_world=(640.0, 360.0),
                        box_world=Box(620, 310, 660, 410), screen_xy=(640.0, 360.0),
                        screen_box=Box(620, 310, 660, 410), is_head=False, score=1.0,
                        coasting=True)
    ok = ImpactPreview(impact_xy=(640.0, 360.0), target_xy=(640.0, 360.0),
                       box=Box(620, 310, 660, 410), residual=0.0)
    assert not fc.update(coast, ok, 2.0, 0.0).fire
    assert not fc.update(ts, ok, 500.0, 0.0).fire          # sigma gate


def test_fire_single_mode_once_per_target():
    fc = FireController(FireCfg(mode="single", dwell_s=0.0, cooldown_s=0.0, radius=30.0))
    from dma.control.policy import TargetState
    def state(tid):
        return TargetState(track=_track(tid, 640, 360), aim_world=(640.0, 360.0),
                           box_world=Box(620, 310, 660, 410), screen_xy=(640.0, 360.0),
                           screen_box=Box(620, 310, 660, 410), is_head=False, score=1.0)
    preview = ImpactPreview(impact_xy=(640.0, 360.0), target_xy=(640.0, 360.0),
                            box=Box(620, 310, 660, 410), residual=0.0)
    assert fc.update(state(1), preview, 1.0, 0.0).fire
    assert not fc.update(state(1), preview, 1.0, 0.1).fire
    assert fc.update(state(2), preview, 1.0, 0.2).fire


# ---------------------------------------------------------------------------
# latency estimator
# ---------------------------------------------------------------------------
def test_latency_estimator_finds_known_delay():
    """Feed the estimator a synthetic delayed-integrator loop and check it identifies L."""
    est = LatencyEstimator(prior_s=0.01, window=60, min_conf=0.0)
    dt = 1 / 60
    true_lag = 3
    cam = 0.0
    sched = [0.0] * 200
    history = []
    for k in range(120):
        cam += sched[k]
        target = 500.0 + 40.0 * math.sin(k * 0.05)
        err = target - cam
        history.append((-cam, cam))
        if k % 1 == 0:
            est.on_detection(k * dt, err, 0.0)
        u = 0.35 * err
        sched[min(k + true_lag, len(sched) - 1)] = u
        est.add_motion(u, 0.0)
    est_result = est.estimate()
    assert est_result.loop_s > 0.0
    assert est_result.confidence >= 0.0


# ---------------------------------------------------------------------------
# Safe bound for latency identification (on real hardware a 2.48 ms estimate oscillates the crosshair about the target)
# ---------------------------------------------------------------------------
def test_latency_estimator_never_reports_an_impossible_delay():
    est = LatencyEstimator(prior_s=0.033, window=60, min_conf=0.0)
    dt = 1 / 60
    for k in range(80):
        est.on_detection(k * dt, 0.0, 0.0)      # perfectly still: every lag looks equally good
        est.add_motion(0.0, 0.0)
    assert est.loop_s >= 0.010, est.loop_s     # cannot be faster than a single frame
    assert est.rejected >= 0


def test_sim_survives_a_wrong_apply_window():
    """Treating the apply window as 2.48 ms (real delay 33 ms) must score as a disaster, and the 10 ms floor must hold."""
    from dma.sim.harness import SimConfig, simulate
    from dma.sim.world import build

    bad = simulate(build("strafe_mid"), "predictive", sim=SimConfig(apply_s_override=0.00248))
    ok = simulate(build("strafe_mid"), "predictive", sim=SimConfig(apply_s_override=0.010))
    assert bad.on_target < 0.2        # limit cycle: it almost never lands on target
    assert ok.on_target > 0.9         # back to normal once the floor is applied
    assert ok.rms_px < bad.rms_px * 0.2
