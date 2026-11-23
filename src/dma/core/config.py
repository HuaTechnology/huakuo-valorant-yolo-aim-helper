"""Versioned configuration (schema v2) with validation, migration and atomic save.

v1 was a flat dict of 33 magic numbers loaded with ``cfg[k] = v if k in cfg``.
The failure mode of that design is well known: a typo silently keeps the old
value, a value out of range silently poisons the control loop, and rolling back
a bad tuning session is impossible.  v2 therefore splits the flat namespace into
sections, keeps **one** registry of field metadata that drives both validation
and the generated UI, and upgrades old files instead of ignoring them.

    settings.json
    ├── schema_version: 2
    ├── capture   … device / resolution / fps
    ├── vision    … model, thresholds, ROI, class mapping
    ├── tracker   … Kalman + association
    ├── aim       … control law
    ├── policy    … target selection
    ├── fire      … trigger discipline
    ├── output    … protocol, report rate, gain
    └── runtime   … rates, watchdogs, logging, recording

Anything unknown in the file is reported (``Settings.warnings``) rather than
dropped silently, so operators can see that they are editing a stale profile.
"""
from __future__ import annotations

import json
import os
import tempfile
import typing
from dataclasses import dataclass, field, fields, replace
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Tuple

SCHEMA_VERSION = 2

REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_PATH = REPO_ROOT / "configs" / "settings.json"
LEGACY_PATH = REPO_ROOT / "config.json"


class ConfigError(ValueError):
    """Raised when a configuration file or a runtime patch is not acceptable."""


# ---------------------------------------------------------------------------
# sections
# ---------------------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class CaptureCfg:
    device: int = 0
    width: int = 1280
    height: int = 720
    fps: int = 120
    fourcc: str = "MJPG"
    buffer_size: int = 1
    warmup_s: float = 2.0
    reconnect_s: float = 1.0


@dataclass(frozen=True, slots=True)
class VisionCfg:
    model: str = "models/model.onnx"
    conf: float = 0.35
    iou: float = 0.45
    max_det: int = 32
    letterbox: str = "auto"          # auto | on | off
    providers: Tuple[str, ...] = ("CoreML", "CPU")
    intra_threads: int = 2
    roi: str = "off"                 # off | center | target
    head_mode: str = "auto"          # auto | class | ratio
    roi_size: int = 512
    target_classes: Tuple[str, ...] = ("body", "head", "st", "person", "enemy", "target")
    ignore_classes: Tuple[str, ...] = ("teammate", "ai", "crosshair")


@dataclass(frozen=True, slots=True)
class TrackerCfg:
    enabled: bool = True
    iou_thr: float = 0.25
    max_age: int = 12
    accel_noise: float = 3000.0      # sigma_a in px/s^2 (manoeuvre capability)
    meas_noise: float = 3.5          # px, detector position sigma
    size_noise: float = 6.0


@dataclass(frozen=True, slots=True)
class AimCfg:
    controller: str = "predictive"   # predictive | legacy
    kp_near: float = 0.50
    kp_far: float = 0.85
    near_px: float = 26.0
    far_px: float = 260.0
    ki: float = 0.05
    kd: float = 0.10
    integral_limit: float = 60.0
    integral_gate: float = 70.0
    lead_gain: float = 0.9
    lead_max: float = 70.0
    latency_prior_s: float = 0.033     # 60 fps capture + display + input ≈ 2 frames
    latency_split: float = 0.5
    latency_window: int = 45
    latency_min_conf: float = 0.5     # low-confidence estimates are always rejected (collapsing to 2 ms sets the crosshair oscillating)
    max_step: float = 140.0
    max_accel: float = 0.0
    deadzone: float = 1.5
    deadzone_exit: float = 5.0
    output_ema: float = 0.9
    settle_px: float = 7.0


@dataclass(frozen=True, slots=True)
class PolicyCfg:
    fov_radius: float = 550.0
    lock_radius: float = 160.0
    switch_margin: float = 55.0
    coast_s: float = 0.18
    head_bonus: float = 28.0
    top_mask: float = 0.08
    bottom_mask: float = 0.12
    head_ratio: float = 0.11
    prefer_head: bool = True
    min_hits: int = 1


@dataclass(frozen=True, slots=True)
class FireCfg:
    mode: str = "single"             # off | single | auto
    radius: float = 9.0
    inset: float = 0.06
    dwell_s: float = 0.04
    cooldown_s: float = 0.12
    tap_s: float = 0.03
    max_sigma: float = 80.0


@dataclass(frozen=True, slots=True)
class OutputCfg:
    protocol: str = "v2"             # v1 | v2
    port: str = "auto"
    baud: int = 921600
    report_hz: int = 500
    gain: float = 1.0
    max_per_report: int = 127
    heartbeat_ms: int = 100
    write_timeout_s: float = 0.02
    dry_run: bool = False


@dataclass(frozen=True, slots=True)
class RuntimeCfg:
    control_hz: int = 240
    preview_fps: int = 30        # UI preview frame rate, 0 = no image, numbers only
    watchdog_ms: float = 400.0
    max_infer_age_s: float = 0.25    # one slow frame is not a fault; 0.12 makes the status light flicker with load
    log_level: str = "INFO"
    log_dir: str = "out/logs"
    record: bool = True      # record every session by default: evidence when things break (~100 KB/min)
    record_dir: str = "out/telemetry"
    metrics_interval_s: float = 1.0


@dataclass(frozen=True, slots=True)
class Settings:
    schema_version: int = SCHEMA_VERSION
    profile: str = "default"
    capture: CaptureCfg = field(default_factory=CaptureCfg)
    vision: VisionCfg = field(default_factory=VisionCfg)
    tracker: TrackerCfg = field(default_factory=TrackerCfg)
    aim: AimCfg = field(default_factory=AimCfg)
    policy: PolicyCfg = field(default_factory=PolicyCfg)
    fire: FireCfg = field(default_factory=FireCfg)
    output: OutputCfg = field(default_factory=OutputCfg)
    runtime: RuntimeCfg = field(default_factory=RuntimeCfg)

    # not part of the file; filled by the loader
    warnings: Tuple[str, ...] = ()

    # -- helpers -----------------------------------------------------------
    def clone(self, **patches: Any) -> "Settings":
        return set_fields(self, patches)

    def section(self, name: str) -> Any:
        if not hasattr(self, name) or name in ("warnings", "schema_version", "profile"):
            raise ConfigError(f"unknown config section: {name}")
        return getattr(self, name)


SECTIONS = ("capture", "vision", "tracker", "aim", "policy", "fire", "output", "runtime")


# ---------------------------------------------------------------------------
# field registry — single source of truth for validation + generated UI
# ---------------------------------------------------------------------------
@dataclass(frozen=True, slots=True)
class FieldMeta:
    group: str
    label: str
    lo: float
    hi: float
    step: float
    unit: str = ""
    help: str = ""


FIELDS: Dict[str, FieldMeta] = {}


def _f(group, key, label, lo, hi, step, unit="", help=""):
    FIELDS[key] = FieldMeta(group, label, lo, hi, step, unit, help)


_f("capture", "capture.device", "Capture device index", 0, 8, 1)
_f("capture", "capture.width", "Capture width", 320, 1920, 16, "px")
_f("capture", "capture.height", "Capture height", 240, 1080, 16, "px")
_f("capture", "capture.fps", "Requested frame rate", 30, 240, 10, "Hz", "value negotiated with the capture card, the HUD shows the effective rate")
_f("capture", "capture.buffer_size", "Driver buffer frames", 1, 8, 1, "", "1 = lowest latency")
_f("capture", "capture.warmup_s", "Warm-up time", 0.0, 8.0, 0.5, "s")
_f("capture", "capture.reconnect_s", "Reconnect interval after stream loss", 0.2, 10.0, 0.2, "s")

_f("vision", "vision.conf", "Confidence threshold", 0.05, 0.90, 0.01)
_f("vision", "vision.iou", "NMS IoU", 0.10, 0.90, 0.01)
_f("vision", "vision.max_det", "Maximum detections", 1, 300, 1)
_f("vision", "vision.intra_threads", "Inference threads", 1, 8, 1)
_f("vision", "vision.roi_size", "ROI edge length", 256, 1280, 32, "px", "centre-crop inference, keeps distant targets resolvable")

_f("tracker", "tracker.iou_thr", "Association IoU threshold", 0.05, 0.80, 0.01)
_f("tracker", "tracker.max_age", "Frame loss tolerance", 1, 40, 1, "frames")
_f("tracker", "tracker.accel_noise", "Manoeuvre capability σa", 100.0, 12000.0, 50.0, "px/s²",
   "assumed acceleration ceiling; higher tracks faster but shakes more")
_f("tracker", "tracker.meas_noise", "Measurement noise", 0.5, 20.0, 0.5, "px")
_f("tracker", "tracker.size_noise", "Size noise", 0.5, 30.0, 0.5, "px")

_f("aim", "aim.kp_near", "Near proportional gain", 0.0, 1.5, 0.01, "", "point-blank trim, lower is steadier")
_f("aim", "aim.kp_far", "Far proportional gain", 0.0, 1.5, 0.01, "", "flick, higher is faster, too high shakes")
_f("aim", "aim.near_px", "Near band start", 2.0, 120.0, 1.0, "px")
_f("aim", "aim.far_px", "Far band end", 40.0, 600.0, 5.0, "px")
_f("aim", "aim.ki", "Integral gain", 0.0, 0.8, 0.01, "", "removes the steady-state tracking error")
_f("aim", "aim.kd", "Derivative damping", 0.0, 1.0, 0.01)
_f("aim", "aim.integral_limit", "Integral limit (anti-windup)", 0.0, 300.0, 5.0, "px")
_f("aim", "aim.integral_gate", "Integral gate", 0.0, 200.0, 5.0, "px", "integrates only while the error is below this")
_f("aim", "aim.lead_gain", "Lead strength", 0.0, 1.5, 0.05, "×", "1.0 = fully compensates link latency")
_f("aim", "aim.lead_max", "Lead clamp", 0.0, 250.0, 5.0, "px", "maximum feed-forward per tick, suppresses sprint/teleport mispredictions")
_f("aim", "aim.latency_prior_s", "Latency prior", 0.0, 0.20, 0.002, "s", "default used before latency identification converges")
_f("aim", "aim.latency_split", "Latency split (measurement side)", 0.2, 0.8, 0.05, "×", "share of the total latency that is stale frame")
_f("aim", "aim.latency_window", "Latency identification window", 10, 200, 5, "tick")
_f("aim", "aim.latency_min_conf", "Latency identification confidence gate", 0.0, 1.0, 0.05)
_f("aim", "aim.max_step", "Maximum step per frame", 10.0, 400.0, 5.0, "px")
_f("aim", "aim.max_accel", "Maximum acceleration per frame (0=off)", 0.0, 200.0, 2.0, "px/tick")
_f("aim", "aim.deadzone", "Dead band inner radius", 0.0, 20.0, 0.5, "px")
_f("aim", "aim.deadzone_exit", "Dead band exit radius", 0.0, 40.0, 0.5, "px")
_f("aim", "aim.output_ema", "Output smoothing", 0.0, 1.0, 0.01, "", "1.0 = no smoothing")
_f("aim", "aim.settle_px", "Settled radius", 1.0, 40.0, 0.5, "px")

_f("policy", "policy.fov_radius", "Field of view radius", 80.0, 1200.0, 10.0, "px")
_f("policy", "policy.lock_radius", "Extrapolation drift limit", 20.0, 500.0, 5.0, "px",
   "how far a lost target may be extrapolated before it is dropped, stops chasing ghosts")
_f("policy", "policy.switch_margin", "Target switch hysteresis", 0.0, 300.0, 5.0, "px")
_f("policy", "policy.coast_s", "Target invisibility tolerance", 0.0, 0.6, 0.01, "s")
_f("policy", "policy.head_bonus", "Head preference bonus", 0.0, 120.0, 2.0, "px")
_f("policy", "policy.top_mask", "Top ignore fraction", 0.0, 0.40, 0.01)
_f("policy", "policy.bottom_mask", "Bottom ignore fraction", 0.0, 0.45, 0.01)
_f("policy", "policy.head_ratio", "Head fraction (no head class)", 0.0, 0.40, 0.005, "", "a model with a head class needs no calibration")
_f("policy", "policy.min_hits", "Minimum hits before selecting", 1, 6, 1, "frames")

_f("fire", "fire.radius", "Fire gate radius (fallback)", 0.0, 60.0, 0.5, "px")
_f("fire", "fire.inset", "Hitbox inset", 0.0, 0.40, 0.01, "×", "larger is more conservative; OR-ed with the fire radius")
_f("fire", "fire.dwell_s", "Settled dwell time", 0.0, 0.30, 0.005, "s")
_f("fire", "fire.cooldown_s", "Shot interval", 0.02, 0.60, 0.005, "s")
_f("fire", "fire.tap_s", "Tap duration", 0.005, 0.15, 0.005, "s")
_f("fire", "fire.max_sigma", "State uncertainty limit", 1.0, 300.0, 1.0, "px")

_f("output", "output.baud", "Serial baud rate", 115200, 3000000, 1, "bps")
_f("output", "output.report_hz", "HID report rate", 60, 1000, 10, "Hz")
_f("output", "output.gain", "Mouse gain", 0.2, 3.0, 0.05, "×")
_f("output", "output.max_per_report", "Maximum step per report", 1, 127, 1, "counts", "the HID int8 limit is 127")
_f("output", "output.heartbeat_ms", "Heartbeat period", 20, 1000, 10, "ms")
_f("output", "output.write_timeout_s", "Write timeout", 0.002, 0.2, 0.002, "s")

_f("runtime", "runtime.control_hz", "Control rate", 30, 1000, 10, "Hz")
_f("runtime", "runtime.preview_fps", "Preview frame rate (0=off)", 0, 60, 1, "fps", "affects only how the UI looks, never the aim")
_f("runtime", "runtime.watchdog_ms", "Watchdog timeout", 50.0, 3000.0, 50.0, "ms")
_f("runtime", "runtime.max_infer_age_s", "Maximum inference staleness", 0.01, 0.5, 0.005, "s")
_f("runtime", "runtime.metrics_interval_s", "Metrics refresh period", 0.2, 10.0, 0.2, "s")

ENUMS: Dict[str, Tuple[str, ...]] = {
    "capture.fourcc": ("MJPG", "YUYV"),
    "vision.letterbox": ("auto", "on", "off"),
    "vision.roi": ("off", "center", "target"),
    "vision.head_mode": ("auto", "class", "ratio"),
    "aim.controller": ("predictive", "legacy"),
    "fire.mode": ("off", "single", "auto"),
    "output.protocol": ("v1", "v2"),
    "runtime.log_level": ("DEBUG", "INFO", "WARNING", "ERROR"),
}


# ---------------------------------------------------------------------------
# (de)serialisation
# ---------------------------------------------------------------------------
def _hints(cls) -> Dict[str, Any]:
    try:
        return typing.get_type_hints(cls)
    except Exception:  # pragma: no cover - exotic forward refs
        return {}


_HINT_CACHE: Dict[type, Dict[str, Any]] = {}


def _coerce(value: Any, typ: Any, where: str) -> Any:
    origin = typing.get_origin(typ)
    if origin is tuple:  # Tuple[str, ...]
        if isinstance(value, str):
            items = [s.strip() for s in value.split(",") if s.strip()]
        elif isinstance(value, Iterable):
            items = [str(v) for v in value]
        else:
            raise ConfigError(f"{where}: expected a list of strings, got {value!r}")
        return tuple(items)
    if typ is bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return bool(value)
        text = str(value).strip().lower()
        if text in ("1", "true", "yes", "on"):
            return True
        if text in ("0", "false", "no", "off"):
            return False
        raise ConfigError(f"{where}: expected a bool, got {value!r}")
    if typ is int:
        try:
            return int(round(float(value)))
        except (TypeError, ValueError) as exc:
            raise ConfigError(f"{where}: expected an int, got {value!r}") from exc
    if typ is float:
        try:
            return float(value)
        except (TypeError, ValueError) as exc:
            raise ConfigError(f"{where}: expected a number, got {value!r}") from exc
    if typ is str:
        return str(value)
    return value


def _defaults_for(cls) -> Dict[str, Any]:
    return {f.name: f.default for f in fields(cls)}


def to_dict(settings: Settings) -> Dict[str, Any]:
    out: Dict[str, Any] = {"schema_version": settings.schema_version, "profile": settings.profile}
    for name in SECTIONS:
        sec = getattr(settings, name)
        out[name] = {
            f.name: (list(v) if isinstance(v, tuple) else v)
            for f, v in ((f, getattr(sec, f.name)) for f in fields(sec))
        }
    return out


def from_dict(raw: Mapping[str, Any], *, strict: bool = False) -> Settings:
    """Build Settings from a possibly partial / older mapping.

    Unknown keys produce a warning (or raise in strict mode) instead of being
    dropped silently: a stale profile that references a removed knob is an
    operator error worth surfacing.
    """
    warnings: list[str] = []
    if int(raw.get("schema_version", 1)) != SCHEMA_VERSION:
        raw = migrate_v1(raw, warnings)

    kwargs: Dict[str, Any] = {}
    if "profile" in raw:
        kwargs["profile"] = str(raw["profile"])

    for name in SECTIONS:
        cls = type(getattr(Settings(), name))
        hints = _HINT_CACHE.setdefault(cls, _hints(cls))
        base = _defaults_for(cls)
        section_raw = raw.get(name, {})
        if not isinstance(section_raw, Mapping):
            raise ConfigError(f"{name}: expected a mapping, got {type(section_raw).__name__}")
        values: Dict[str, Any] = {}
        for key, value in section_raw.items():
            if key not in base:
                msg = f"unknown field {name}.{key} (ignored)"
                if strict:
                    raise ConfigError(msg)
                warnings.append(msg)
                continue
            values[key] = _coerce(value, hints.get(key, Any), f"{name}.{key}")
        kwargs[name] = cls(**{**base, **values})

    known = {"schema_version", "profile", *SECTIONS}
    for key in raw:
        if key not in known:
            msg = f"unknown config section {key} (ignored)"
            if strict:
                raise ConfigError(msg)
            warnings.append(msg)

    settings = Settings(**kwargs)
    settings = replace(settings, warnings=tuple(warnings))
    validate(settings)
    return settings


# ---------------------------------------------------------------------------
# migration
# ---------------------------------------------------------------------------
_V1_MAP = {
    "model_path": "vision.model",
    "model_conf": "vision.conf",
    "capture_w": "capture.width",
    "capture_h": "capture.height",
    "capture_fps": "capture.fps",
    "SINK_GAIN": "output.gain",
    "max_step": "aim.max_step",
    "FIRE_RADIUS": "fire.radius",
    "FIRE_COOLDOWN": "fire.cooldown_s",
    "FIRE_TAP_S": "fire.tap_s",
    "FIRE_MODE": "fire.mode",
    "FOV_RADIUS": "policy.fov_radius",
    "LOCK_RADIUS": "policy.lock_radius",
    "COAST_FRAMES": "policy.coast_s",
    "SWITCH_JUMP": "policy.switch_margin",
    "TOP_MASK": "policy.top_mask",
    "BOTTOM_MASK": "policy.bottom_mask",
    "HEAD_RATIO": "policy.head_ratio",
    # v1 EMA/PID internals have no direct v2 counterpart: the predictive
    # controller replaces them.  Recorded as a warning, never silently used.
}


def migrate_v1(raw: Mapping[str, Any], warnings: list[str] | None = None) -> Dict[str, Any]:
    """Upgrade a flat v1 ``config.json`` into the sectioned v2 layout."""
    warnings = warnings if warnings is not None else []
    if "schema_version" in raw and int(raw["schema_version"]) >= SCHEMA_VERSION:
        return dict(raw)

    out: Dict[str, Any] = {"schema_version": SCHEMA_VERSION, "profile": "migrated-v1"}
    for key, value in raw.items():
        path = _V1_MAP.get(key)
        if path is None:
            if key in _V1_INTERNAL:
                warnings.append(f"v1 field {key} is taken over by the predictive controller in v2 (not migrated)")
            continue
        section, name = path.split(".", 1)
        out.setdefault(section, {})[name] = value

    # v1 recorded coast as frames; convert with the 60 fps the loop ran at.
    if "COAST_FRAMES" in raw:
        try:
            out["policy"]["coast_s"] = round(float(raw["COAST_FRAMES"]) / 60.0, 3)
        except (TypeError, ValueError):
            pass
    # v1 ran 60 Hz and had no delay compensation at all; keep the measured
    # prior instead of trusting its "1 frame" claim.
    out.setdefault("aim", {})["latency_prior_s"] = 0.033
    warnings.append(
        "Migrated from the v1 config.json; the v1 PID gains were not imported (not comparable with the v2"
        " control law), aim.* uses the v2 defaults. SINK_GAIN maps to output.gain (counts/screen pixel); measure it with aim2 calibrate gain"
    )
    return out


_V1_INTERNAL = {
    # v1 PID gains are not comparable with the v2 law (different structure and
    # rate normalisation), so they are reported instead of imported: carrying
    # them over would look like "your tuning was kept" while actually detuning.
    "kp_far", "kp_mid", "kp_near", "kp_touch", "kd", "ki",
    "integral_limit", "integral_gate", "deadzone", "accel",
    "ema_alpha_near",
    "ema_alpha_far",
    "far_alpha_th",
    "alpha_band",
    "PREDICT_FRAMES",
    "PREDICT_ALPHA",
    "PREDICT_CLAMP",
}


# ---------------------------------------------------------------------------
# validation
# ---------------------------------------------------------------------------
def validate(settings: Settings) -> None:
    problems: list[str] = []
    for path, meta in FIELDS.items():
        value = get_field(settings, path)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            if not (meta.lo <= float(value) <= meta.hi):
                problems.append(f"{path}={value} outside range [{meta.lo}, {meta.hi}]")
    for path, allowed in ENUMS.items():
        value = str(get_field(settings, path))
        if value not in allowed:
            problems.append(f"{path}={value!r} invalid, allowed {allowed}")
    if settings.aim.near_px >= settings.aim.far_px:
        problems.append("aim.near_px must be smaller than aim.far_px")
    if settings.aim.deadzone_exit < settings.aim.deadzone:
        problems.append("aim.deadzone_exit must not be smaller than aim.deadzone")
    if settings.policy.lock_radius < 20:
        problems.append("policy.lock_radius too small (<20 px)")
    if settings.capture.width % 2 or settings.capture.height % 2:
        problems.append("capture width and height must be even (encoder requirement)")
    if problems:
        raise ConfigError("invalid configuration:\n  - " + "\n  - ".join(problems))


# ---------------------------------------------------------------------------
# field access
# ---------------------------------------------------------------------------
def get_field(settings: Settings, path: str) -> Any:
    section, _, name = path.partition(".")
    if not name:
        raise ConfigError(f"field path must look like section.field, got {path!r}")
    sec = getattr(settings, section, None)
    if sec is None or not hasattr(sec, name):
        raise ConfigError(f"unknown field {path}")
    return getattr(sec, name)


def set_fields(settings: Settings, patches: Mapping[str, Any], *, validate_after: bool = True) -> Settings:
    """Return a new Settings with dotted-path patches applied (atomic)."""
    grouped: Dict[str, Dict[str, Any]] = {}
    for path, value in patches.items():
        section, _, name = path.partition(".")
        sec = getattr(settings, section, None)
        if sec is None or not hasattr(sec, name):
            raise ConfigError(f"unknown field {path}")
        cls = type(sec)
        hints = _HINT_CACHE.setdefault(cls, _hints(cls))
        grouped.setdefault(section, {})[name] = _coerce(value, hints.get(name, Any), path)

    kwargs: Dict[str, Any] = {"warnings": settings.warnings}
    for section, values in grouped.items():
        kwargs[section] = replace(getattr(settings, section), **values)
    new = replace(settings, **kwargs)
    if validate_after:
        validate(new)
    return new


def public_fields() -> Iterable[Tuple[str, FieldMeta]]:
    return sorted(FIELDS.items())


# ---------------------------------------------------------------------------
# disk IO
# ---------------------------------------------------------------------------
def load(path: os.PathLike | str | None = None) -> Settings:
    """Load settings, upgrading v1 files (including legacy ``config.json``)."""
    p = Path(path) if path else DEFAULT_PATH
    if not p.exists():
        if p == DEFAULT_PATH and LEGACY_PATH.exists():
            raw = json.loads(LEGACY_PATH.read_text(encoding="utf-8"))
            migrated = from_dict(raw)
            return replace(migrated, warnings=migrated.warnings + (f"source: {LEGACY_PATH.name} (v1)",))
        return Settings()
    raw = json.loads(p.read_text(encoding="utf-8"))
    if not isinstance(raw, Mapping):
        raise ConfigError(f"{p}: the top level must be a mapping")
    return from_dict(raw)


def save(settings: Settings, path: os.PathLike | str | None = None) -> Path:
    """Atomic write: temp file + fsync + rename, so a crash never leaves a
    truncated profile behind."""
    p = Path(path) if path else DEFAULT_PATH
    p.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(to_dict(settings), ensure_ascii=False, indent=2, sort_keys=False)
    fd, tmp = tempfile.mkstemp(dir=str(p.parent), prefix=p.name + ".", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(payload)
            fh.write("\n")
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, p)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    return p


def diff(a: Settings, b: Settings) -> Dict[str, Tuple[Any, Any]]:
    """Field level diff, used for hot-reload audit logs and profile review."""
    out: Dict[str, Tuple[Any, Any]] = {}
    da, db = to_dict(a), to_dict(b)
    for section in SECTIONS:
        for key, old in da[section].items():
            new = db[section].get(key)
            if old != new:
                out[f"{section}.{key}"] = (old, new)
    return out
