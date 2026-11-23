"""Calibration helpers: model sanity, head point, latency.

The three numbers that decide whether the system can hit anything are the class
map (do we even know what a head is?), the head point (where inside the box do we
aim?) and the loop latency (how far ahead must we predict?).  v1 had all three as
hand-entered magic values.  Here each one has a procedure that either measures it
or explains exactly which measurement is missing.
"""
from __future__ import annotations

import statistics
import time
from typing import Dict, List, Optional, Tuple

from ..control.latency import LatencyEstimator
from ..core.config import Settings
from ..telemetry.recorder import RecordedSession


def class_names(session: RecordedSession) -> Dict[int, str]:
    model = (session.header or {}).get("model") or {}
    raw = model.get("classes") or {}
    out: Dict[int, str] = {}
    for k, v in raw.items():
        try:
            out[int(k)] = str(v)
        except (TypeError, ValueError):
            continue
    return out


def model_report(detector) -> Dict[str, object]:
    """Validate that the loaded model can actually serve this pipeline."""
    info = detector.describe()
    names = {int(k): str(v) for k, v in info["classes"].items()}
    targets = {names.get(int(i), "") for i in info["targets"]}
    heads = {names.get(int(i), "") for i in info["heads"]}
    errors: List[str] = []
    warnings: List[str] = []
    if not targets:
        errors.append("no shootable classes: check vision.target_classes and the model metadata")
    if "head" not in heads and detector.cfg.head_mode != "ratio":
        warnings.append("model has no head class: head locking relies on a hand-tuned policy.head_ratio"
                        " (set vision.head_mode to ratio if ratio head locking is intended)")
    if any(n in ("teammate", "ai") for n in targets):
        errors.append("teammate/bot classes are listed as shootable: the aim would pull onto teammates")
    fmt = info["format"]
    if fmt == "yolov8" and detector.letterbox_mode == "stretch":
        errors.append("a yolov8 model with stretch preprocessing biases the boxes; use letterbox")
    return {
        "model": info["model"],
        "input": info["input"],
        "format": fmt,
        "providers": info["providers"],
        "targets": sorted(t for t in targets if t),
        "heads": sorted(h for h in heads if h),
        "letterbox": info["letterbox"],
        "errors": errors,
        "warnings": warnings,
        "ok": not errors,
    }


def head_ratio_from_session(session: RecordedSession) -> Optional[Tuple[float, int]]:
    """Estimate where the head sits inside the body box, from recorded data."""
    names = class_names(session)
    head_ids = {k for k, v in names.items() if v.lower() == "head"}
    body_ids = {k for k, v in names.items() if v.lower() in ("body", "st", "person", "enemy")}
    if not head_ids or not body_ids:
        return None
    ratios: List[float] = []
    for frame in session.frames:
        dets = frame.get("dets") or []
        heads = [d for d in dets if int(d[4]) in head_ids]
        bodies = [d for d in dets if int(d[4]) in body_ids]
        for head in heads:
            hx = (head[0] + head[2]) / 2.0
            hy = (head[1] + head[3]) / 2.0
            for body in bodies:
                bx1, by1, bx2, by2 = body[0], body[1], body[2], body[3]
                if bx1 <= hx <= bx2 and by1 <= hy <= by2:
                    h = max(1.0, by2 - by1)
                    ratios.append((hy - by1) / h)
                    break
    if len(ratios) < 20:
        return None
    return float(statistics.median(ratios)), len(ratios)


def latency_from_estimator(est: LatencyEstimator) -> Dict[str, object]:
    e = est.estimate()
    return {
        "loop_ms": round(e.loop_s * 1000.0, 2),
        "frames": round(e.frames, 2),
        "confidence": round(e.confidence, 3),
        "samples": e.samples,
        "verdict": (
            "trustworthy" if e.confidence >= 0.35 and e.samples >= 30
            else "not enough samples: keep tracking a moving target for a while"
        ),
    }


def suggest(settings: Settings, report: Dict[str, object]) -> List[str]:
    tips: List[str] = []
    if not report.get("heads") and settings.vision.head_mode != "ratio":
        tips.append(
            f"the model has no head class; head locking relies on policy.head_ratio={settings.policy.head_ratio};"
            "set vision.head_mode to ratio for ratio head locking, otherwise load a model with a head class"
        )
    if settings.policy.head_ratio <= 0.02 or settings.policy.head_ratio >= 0.3:
        tips.append("policy.head_ratio looks suspicious (0.08~0.16 is the usual range)")
    if settings.output.report_hz < 125:
        tips.append("output.report_hz is low: the HID report rate sets sub-frame smoothness, aim for >=250")
    if settings.runtime.control_hz < 120:
        tips.append("runtime.control_hz is low: law is rate normalised, but a higher rate shrinks quantisation steps")
    if settings.output.gain > 1.2:
        tips.append(
            f"output.gain={settings.output.gain}: v2 output is already a screen-pixel displacement, so the"
            "multiplier goes straight into the loop gain (>1.2 pushes kp past the stability boundary); keep 1.0"
        )
    if settings.vision.conf < 0.2:
        tips.append(f"vision.conf={settings.vision.conf} is low, HUD portraits and similar false positives increase; "
                    "try 0.30")
    if settings.aim.latency_prior_s <= 0:
        tips.append("aim.latency_prior_s is 0: nothing is predicted until the online identification converges")
    return tips


def wait_for_latency(engine, seconds: float = 6.0, interval: float = 0.5) -> Dict[str, object]:
    """Run the engine for a while and return the delay identification result."""
    engine.set_aim_enabled(True)
    engine.set_fire_enabled(False)
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        engine.step()
        time.sleep(1.0 / max(30, engine.settings.runtime.control_hz))
    return latency_from_estimator(engine.latency)


def measure_counts_per_px(engine, seconds: float = 6.0, probe_counts: int = 160) -> Dict[str, object]:
    """Measure how many mouse counts move the view by one screen pixel.

    This is the one calibration that cannot be guessed: it depends on the game's
    sensitivity, the mouse DPI and the capture resolution.  Everything else in
    the loop is dimensionless, so a wrong value shows up as "slow / never
    sticks" (too small) or "overshoots" (too large).

    Procedure: find a slow target, remember its screen position, send a known
    number of counts with the controller disabled, then measure how far it moved.
    """
    import time

    t_start = time.monotonic()
    anchor = None          # (ts, screen_xy, speed)
    probe_sent = 0
    moved = None
    engine.set_aim_enabled(False)
    engine.set_fire_enabled(False)

    while time.monotonic() - t_start < seconds:
        engine.step()
        snap = engine.snapshot()
        target = snap.target
        now = time.monotonic()
        if probe_sent == 0:
            if target is not None and target.track.speed < 40.0 and target.track.hits >= 2:
                if anchor is None:
                    anchor = (now, target.screen_xy)
                elif now - anchor[0] > 0.25:
                    engine.link.planner.add(float(probe_counts), 0.0)
                    for step in engine.link.planner.take(now):
                        engine.link.submit_motion(float(step[0]) / max(1e-6, engine.settings.output.gain), 0.0)
                    probe_sent = probe_counts
                    anchor = (now, target.screen_xy)
        else:
            if anchor is not None and now - anchor[0] > 0.45 and target is not None:
                dx = target.screen_xy[0] - anchor[1][0]
                if abs(dx) >= 3.0:
                    moved = dx
                    break
                if now - anchor[0] > 1.2:
                    break
        time.sleep(0.5 / max(30, engine.settings.runtime.control_hz))

    if moved is None or probe_sent == 0:
        return {
            "ok": False,
            "reason": "no stable target found on screen, or the shift was never observed",
            "hint": "make sure the game runs (not paused / menu) with a target near the crosshair, then retry",
            "probe_counts": probe_sent,
        }

    # moving the mouse right shifts the scene left, so the measured dx is negative
    counts = abs(probe_sent) / max(0.1, abs(moved))
    return {
        "ok": True,
        "probe_counts": probe_sent,
        "scene_shift_px": round(moved, 1),
        "counts_per_px": round(counts, 3),
    }
