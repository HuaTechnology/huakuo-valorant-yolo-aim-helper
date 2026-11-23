from __future__ import annotations

import time

from dma.capture.sources import SyntheticSource
from dma.core.config import Settings, set_fields
from dma.core.metrics import Metrics
from dma.core.types import Health
from dma.engine.runtime import Engine
from dma.sim.fake_detector import SyntheticDetector
from dma.transport.link import MockLink


def _engine(**patches) -> Engine:
    cfg = set_fields(
        Settings(),
        {
            "runtime.control_hz": 240,
            "output.report_hz": 1000,
            "output.dry_run": True,
            "fire.mode": "auto",
            **patches,
        },
    )
    source = SyntheticSource(cfg.capture.width, cfg.capture.height, 200)
    metrics = Metrics()
    link = MockLink(cfg.output, metrics)
    return Engine(cfg, metrics=metrics, source=source, link=link, detector=SyntheticDetector())


def _spin(engine: Engine, seconds: float = 1.2, aim: bool = True) -> None:
    engine.start()
    engine.set_aim_enabled(aim)
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        engine.step()
        time.sleep(1.0 / 240.0)
    engine.stop()


def test_engine_end_to_end():
    eng = _engine()
    _spin(eng, 1.2)
    snap = eng.snapshot()
    assert snap.health in (Health.OK, Health.DEGRADED)
    assert eng.grabber.fps > 20
    assert eng.metrics.get_counter("capture.frames") > 10
    assert eng.metrics.get_counter("perception.dets") >= 0
    # the synthetic scene contains a target, so the loop must have commanded motion
    assert eng.metrics.get_counter("transport.reports") > 0
    assert snap.frame is not None


def test_engine_aim_disabled_commands_nothing():
    eng = _engine()
    _spin(eng, 0.8, aim=False)
    assert eng.metrics.get_counter("transport.reports") == 0


def test_engine_snapshot_contract():
    eng = _engine()
    _spin(eng, 1.0)
    snap = eng.snapshot()
    assert snap.latency["loop_ms"] >= 0
    # A lower bound proving the control thread is running, not a performance benchmark: on a
    # shared CI runner each step() costs far more than 1/240 wall-clock, roughly 60 Hz measured.
    assert snap.fps_control > 20
    assert isinstance(snap.metrics, dict)
    assert all(hasattr(s, "name") for s in snap.stages)
    assert "aim.cam_x" in snap.metrics.get("gauges", {})


def test_engine_hot_reload():
    eng = _engine()
    _spin(eng, 0.5)
    new = set_fields(eng.settings, {"aim.kp_far": 0.7, "fire.dwell_s": 0.05})
    notes = eng.apply(new)
    assert isinstance(notes, list)
    assert eng.settings.aim.kp_far == 0.7
    assert eng.controller.cfg.kp_far == 0.7
    assert eng.fire.cfg.dwell_s == 0.05


def test_engine_describe_is_json_safe():
    import json

    eng = _engine()
    _spin(eng, 0.4)
    json.dumps(eng.describe(), default=str)


def test_engine_recovers_from_dead_source():
    """A stale capture stream must degrade, never raise."""
    eng = _engine()
    eng.grabber.source.close()
    eng.start()
    time.sleep(0.3)
    eng.step()
    assert eng.snapshot().health in (Health.DEGRADED, Health.OK, Health.BOOT)
    eng.stop()
