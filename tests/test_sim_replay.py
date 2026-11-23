from __future__ import annotations

import math

import pytest

from dma.sim.harness import SimConfig, compare, simulate
from dma.sim.world import SCENARIOS, build
from dma.telemetry.recorder import SessionRecorder, load_session, summarize
from dma.telemetry.replay import replay


def test_all_scenarios_run():
    for name in SCENARIOS:
        m = simulate(build(name), "predictive", sim=SimConfig())
        assert m.ticks > 0
        assert math.isfinite(m.tracking_rms_px)
        assert m.tracking_rms_px < 90.0, name


def test_predictive_beats_legacy_on_tracking():
    """The regression gate for the whole control rewrite."""
    rows = compare(["strafe_mid", "strafe_far", "peek"])
    by = {(r["scenario"], r["controller"]): r for r in rows}
    for scenario in ("strafe_mid", "strafe_far", "peek"):
        old = by[(scenario, "legacy")]
        new = by[(scenario, "predictive")]
        # overall RMS includes the re-acquisition transient, which is where the
        # v1 law loses: it has no tracking, no coasting and no delay compensation
        assert new["rms_px"] < old["rms_px"], (scenario, old, new)
        assert new["on_target"] >= old["on_target"], (scenario, old, new)


def test_sim_is_deterministic():
    a = simulate(build("strafe_mid"), "predictive", sim=SimConfig(seed=5))
    b = simulate(build("strafe_mid"), "predictive", sim=SimConfig(seed=5))
    assert a.rms_px == pytest.approx(b.rms_px)


def test_sim_latency_matters():
    """With no latency the same law must track better than with 66 ms."""
    fast = simulate(build("strafe_far"), "predictive", sim=SimConfig(latency_s=0.0))
    slow = simulate(build("strafe_far"), "predictive", sim=SimConfig(latency_s=0.066))
    assert fast.tracking_rms_px <= slow.tracking_rms_px


def test_sim_quantisation_does_not_break_convergence():
    m = simulate(build("micro_drift"), "predictive", sim=SimConfig(quantize=True))
    assert m.tracking_rms_px < 15.0


def test_record_and_replay(tmp_path):
    rec = SessionRecorder(tmp_path, settings=None, model_info={"classes": {"0": "body"}})
    # fabricate a short recording: a target strafing in screen space, no camera motion
    for k in range(120):
        from dma.core.types import Box, Detection, DetectionFrame

        x = 640 + 120 * math.sin(k * 0.08)
        det = Detection(box=Box(x - 23, 310, x + 23, 410), cls_id=0, cls_name="body", score=0.9)
        rec.frame(DetectionFrame(frame_id=k, capture_ts=k / 60.0, detections=(det,), infer_ms=4.0),
                  cam=(0.0, 0.0))
        rec.tick(k / 60.0, (0.0, 0.0), (1.0, 0.0), 1, (2.0, 0.0), "track", {"loop_ms": 33.0})
    rec.close()

    session = load_session(rec.path)
    assert session.frames and session.ticks
    summary = summarize(session)
    assert summary["frames"] == 120
    result = replay(session, "predictive")
    assert result.frames == 120
    assert result.rms_px < 40.0
