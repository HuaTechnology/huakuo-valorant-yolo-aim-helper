from __future__ import annotations

import json

import pytest

from dma.core.config import (
    SCHEMA_VERSION,
    ConfigError,
    Settings,
    diff,
    from_dict,
    get_field,
    load,
    save,
    set_fields,
    to_dict,
    validate,
)


def test_defaults_are_valid():
    s = Settings()
    validate(s)
    assert s.schema_version == SCHEMA_VERSION
    assert s.aim.controller == "predictive"
    assert s.output.protocol == "v2"


def test_roundtrip(tmp_path):
    s = Settings(profile="test")
    path = save(s, tmp_path / "s.json")
    again = load(path)
    assert to_dict(again)["aim"] == to_dict(s)["aim"]
    assert again.profile == "test"


def test_v1_migration(tmp_path):
    legacy = {
        "model_path": "models/model.onnx",
        "model_conf": 0.18,
        "SINK_GAIN": 3.0,
        "max_step": 112.0,
        "kp_far": 0.45,
        "kd": 0.39,
        "ki": 0.2,
        "FIRE_RADIUS": 16.0,
        "FIRE_MODE": "single",
        "FOV_RADIUS": 550.0,
        "COAST_FRAMES": 6,
        "HEAD_RATIO": 0.1386,
        "ema_alpha_near": 0.55,   # v1 internal -> ignored with a warning
        "PREDICT_FRAMES": 2,
    }
    path = tmp_path / "config.json"
    path.write_text(json.dumps(legacy), encoding="utf-8")
    s = load(path)
    assert s.vision.model.endswith("model.onnx")
    assert s.vision.conf == pytest.approx(0.18)
    # SINK_GAIN is a unit conversion (mouse counts per screen pixel), so it is
    # carried over; the v1 PID gains are not (different control law).
    assert s.output.gain == pytest.approx(3.0)
    assert any("PID" in w for w in s.warnings)
    assert s.aim.max_step == pytest.approx(112.0)
    # v1 PID gains must not leak into the v2 law
    assert s.aim.kp_far == pytest.approx(Settings().aim.kp_far)
    assert s.fire.radius == pytest.approx(16.0)
    assert s.policy.head_ratio == pytest.approx(0.1386)
    assert s.policy.coast_s == pytest.approx(6 / 60)
    assert any("v1" in w for w in s.warnings)
    assert any("PREDICT_FRAMES" in w for w in s.warnings)


def test_unknown_key_warns_but_strict_raises():
    raw = {"schema_version": 2, "aim": {"nope": 1.0}}
    s = from_dict(raw)
    assert any("nope" in w for w in s.warnings)
    with pytest.raises(ConfigError):
        from_dict(raw, strict=True)


def test_out_of_range_rejected():
    with pytest.raises(ConfigError):
        set_fields(Settings(), {"aim.kp_far": 5.0})
    with pytest.raises(ConfigError):
        set_fields(Settings(), {"aim.kp_near": -1.0})


def test_enum_rejected():
    with pytest.raises(ConfigError):
        set_fields(Settings(), {"fire.mode": "burst"})


def test_cross_field_validation():
    with pytest.raises(ConfigError):
        set_fields(Settings(), {"aim.near_px": 300.0, "aim.far_px": 100.0})


def test_set_fields_is_atomic():
    s = Settings()
    new = set_fields(s, {"aim.kp_far": 0.6, "policy.fov_radius": 400.0})
    assert s.aim.kp_far != 0.6          # original untouched
    assert new.aim.kp_far == 0.6
    assert new.policy.fov_radius == 400.0


def test_set_unknown_field():
    with pytest.raises(ConfigError):
        set_fields(Settings(), {"aim.unknown": 1.0})


def test_get_field():
    assert get_field(Settings(), "output.report_hz") == 500
    with pytest.raises(ConfigError):
        get_field(Settings(), "output.nope")


def test_atomic_save_leaves_no_tmp(tmp_path):
    path = save(Settings(), tmp_path / "s.json")
    assert path.exists()
    assert not list(tmp_path.glob("*.tmp"))


def test_diff():
    a = Settings()
    b = set_fields(a, {"aim.kp_far": 0.9})
    changes = diff(a, b)
    assert changes["aim.kp_far"] == (a.aim.kp_far, 0.9)


def test_type_coercion_from_json():
    raw = {"schema_version": 2, "output": {"report_hz": "250", "dry_run": "true"},
           "vision": {"providers": ["CoreML"], "target_classes": "body,head"}}
    s = from_dict(raw)
    assert s.output.report_hz == 250
    assert s.output.dry_run is True
    assert s.vision.providers == ("CoreML",)
    assert s.vision.target_classes == ("body", "head")
