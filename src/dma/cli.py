"""``aim2`` — the single entry point for running, diagnosing and tuning.

Design rules for this CLI:

* every subcommand works without hardware (``--sim`` / ``--dry-run``), so CI can
  exercise the same code path an operator uses;
* nothing needs environment variables or a hand-edited file to be useful;
* machine readable output (``--json``) for anything a dashboard or a test might
  consume, human tables otherwise.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from .core import __version__ as VERSION
from .core.config import (
    DEFAULT_PATH,
    SCHEMA_VERSION,
    ConfigError,
    Settings,
    diff,
    get_field,
    load,
    save,
    set_fields,
    to_dict,
    validate,
)
from .core.logging import setup as setup_logging
from .core.metrics import Metrics


# ---------------------------------------------------------------------------
# output helpers
# ---------------------------------------------------------------------------
def _table(rows: Sequence[Dict[str, Any]], columns: Sequence[str]) -> str:
    if not rows:
        return "(no data)"
    widths = {c: len(c) for c in columns}
    for row in rows:
        for c in columns:
            widths[c] = max(widths[c], len(_fmt(row.get(c))))
    head = "  ".join(c.ljust(widths[c]) for c in columns)
    sep = "  ".join("-" * widths[c] for c in columns)
    body = "\n".join("  ".join(_fmt(r.get(c)).ljust(widths[c]) for c in columns) for r in rows)
    return f"{head}\n{sep}\n{body}"


def _fmt(v: Any) -> str:
    if v is None:
        return "-"
    if isinstance(v, float):
        return f"{v:.2f}"
    return str(v)


def _print_json(obj: Any) -> None:
    print(json.dumps(obj, ensure_ascii=False, indent=2, default=str))


# ---------------------------------------------------------------------------
# doctor
# ---------------------------------------------------------------------------
def cmd_doctor(args: argparse.Namespace) -> int:
    settings = load(args.settings)
    checks: List[Dict[str, Any]] = []

    def add(name: str, ok: bool, detail: str = "") -> None:
        checks.append({"check": name, "ok": ok, "detail": detail})

    add("python", sys.version_info >= (3, 10), sys.version.split()[0])
    try:
        import onnxruntime as ort

        add("onnxruntime", True, f"{ort.__version__} providers={','.join(ort.get_available_providers())}")
    except Exception as exc:
        add("onnxruntime", False, str(exc))
    try:
        import cv2

        add("opencv", True, cv2.__version__)
    except Exception as exc:
        add("opencv", False, str(exc))
    try:
        import serial

        add("pyserial", True, getattr(serial, "VERSION", "?"))
    except Exception as exc:
        add("pyserial", False, str(exc))

    detector = None
    try:
        from .perception.detector import OnnxDetector
        from .tuning.calibration import model_report, suggest

        t0 = time.perf_counter()
        detector = OnnxDetector(settings.vision)
        report = model_report(detector)
        add("model", bool(report["ok"]), f"{report['model']} {report['input']} {report['format']} {time.perf_counter()-t0:.2f}s")
        add("classes", True, f"targets={report['targets']} heads={report['heads']}")
        for msg in report["errors"]:
            add("error", False, msg)
        for msg in report["warnings"]:
            add("warning", True, msg)
        for tip in suggest(settings, report):
            add("suggestion", True, tip)
    except Exception as exc:
        add("model", False, f"{type(exc).__name__}: {exc}")

    if settings.output.dry_run or args.dry_run:
        add("serial", True, "dry-run: serial port skipped")
    else:
        try:
            from .transport.serial_link import SerialLink

            port = SerialLink.resolve_port()
            add("serial", True, port)
        except Exception as exc:
            add("serial", False, str(exc))

    if args.probe and detector is not None:
        try:
            import cv2
            import numpy as np

            frame = np.full((settings.capture.height, settings.capture.width, 3), 24, np.uint8)
            cv2.rectangle(frame, (500, 300), (560, 420), (60, 60, 200), -1)
            res = detector.bench([frame], runs=args.probe_runs)
            add("inference benchmark", True, f"p50={res['p50_ms']}ms p95={res['p95_ms']}ms fps={res['fps']}")
        except Exception as exc:
            add("inference benchmark", False, str(exc))

    ok = all(c["ok"] for c in checks if c["check"] not in ("suggestion", "warning"))
    if args.json:
        _print_json({"ok": ok, "schema_version": SCHEMA_VERSION, "checks": checks})
    else:
        print(_table(checks, ["check", "ok", "detail"]))
        print(f"\nverdict: {'pass' if ok else 'blocked'}")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# models
# ---------------------------------------------------------------------------
def cmd_models(args: argparse.Namespace) -> int:
    from .perception.detector import OnnxDetector
    from .tuning.calibration import model_report

    if args.action == "list":
        paths = sorted(Path("models").glob("*.onnx"))
        rows = []
        for p in paths:
            try:
                det = OnnxDetector(Settings().vision.__class__(model=str(p), conf=0.3))
                rep = model_report(det)
                rows.append({"model": p.name, "input": rep["input"], "format": rep["format"],
                             "targets": ",".join(rep["targets"]), "heads": ",".join(rep["heads"]),
                             "ok": rep["ok"]})
            except Exception as exc:
                rows.append({"model": p.name, "input": "-", "format": "-", "targets": "-", "heads": "-",
                             "ok": f"ERR {exc}"})
        if args.json:
            _print_json(rows)
        else:
            print(_table(rows, ["model", "input", "format", "targets", "heads", "ok"]))
        return 0

    path = args.path
    det = OnnxDetector(Settings().vision.__class__(model=path, conf=0.3))
    if args.action == "inspect":
        rep = model_report(det)
        if args.json:
            _print_json(rep)
        else:
            for k, v in rep.items():
                print(f"{k:12s} {v}")
        return 0 if rep["ok"] else 1

    # bench
    import numpy as np

    frame = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)
    res = det.bench([frame], runs=args.runs)
    if args.json:
        _print_json(res)
    else:
        print(_table([res], list(res)))
    return 0


# ---------------------------------------------------------------------------
# sim / bench
# ---------------------------------------------------------------------------
def cmd_sim(args: argparse.Namespace) -> int:
    from .sim.harness import SimConfig, simulate
    from .sim.world import SCENARIOS, build

    settings = load(args.settings)
    sim = SimConfig(
        control_hz=args.control_hz,
        detect_hz=args.detect_hz,
        latency_s=args.latency_ms / 1000.0,
        noise_px=args.noise,
    )
    names = [args.scenario] if args.scenario else list(SCENARIOS)
    kinds = ["legacy", "predictive"] if args.compare else [args.controller]
    rows = []
    for name in names:
        for kind in kinds:
            cfg = sim
            if kind == "legacy":
                cfg = SimConfig(control_hz=sim.detect_hz, detect_hz=sim.detect_hz,
                                latency_s=sim.latency_s, noise_px=sim.noise_px,
                                dropout=sim.dropout, seed=sim.seed, use_tracker=False)
            m = simulate(build(name, width=settings.capture.width, height=settings.capture.height),
                         kind, sim=cfg, aim_cfg=settings.aim)
            rows.append(m.as_row())
    if args.json:
        _print_json(rows)
    else:
        print(_table(rows, ["scenario", "controller", "rms_px", "p95_px", "track_rms",
                            "first_acq_s", "on_target", "recovery_s", "jitter_px"]))
    return 0


def cmd_bench(args: argparse.Namespace) -> int:
    """Regression gate: the predictive law must beat v1 on every scenario."""
    from .sim.harness import compare

    rows = compare()
    failures = []
    by_key = {(r["scenario"], r["controller"]): r for r in rows}
    for scenario in {r["scenario"] for r in rows}:
        old = by_key.get((scenario, "legacy"))
        new = by_key.get((scenario, "predictive"))
        if not old or not new:
            continue
        # Gate on end-to-end quality, not on steady-state dither:
        #  * rms  — includes the acquisition transient, where v1 has no tracking
        #  * on_target — the metric the player actually feels
        #  * jitter — must not get noticeably noisier
        if isinstance(new["rms_px"], (int, float)) and isinstance(old["rms_px"], (int, float)):
            if new["rms_px"] > old["rms_px"] * 1.02:
                failures.append(f"{scenario}: RMS {new['rms_px']} > v1 {old['rms_px']}")
        if isinstance(new["on_target"], (int, float)) and isinstance(old["on_target"], (int, float)):
            if new["on_target"] < old["on_target"] - 0.01:
                failures.append(f"{scenario}: on-target ratio {new['on_target']} < v1 {old['on_target']}")
        if isinstance(new["jitter_px"], (int, float)) and isinstance(old["jitter_px"], (int, float)):
            if new["jitter_px"] > old["jitter_px"] + 1.0:
                failures.append(f"{scenario}: jitter {new['jitter_px']} > v1 {old['jitter_px']}")

    if args.json:
        _print_json({"rows": rows, "failures": failures})
    else:
        print(_table(rows, ["scenario", "controller", "rms_px", "on_target", "recovery_s", "jitter_px"]))
        print()
        print("regression gate: " + ("pass" if not failures else "fail\n  " + "\n  ".join(failures)))
    return 0 if not failures else 2


# ---------------------------------------------------------------------------
# autotune / replay / record
# ---------------------------------------------------------------------------
def cmd_autotune(args: argparse.Namespace) -> int:
    from .tuning.autotune import autotune

    settings = load(args.settings)
    result = autotune(settings.aim, passes=args.passes)
    if args.write:
        import dataclasses

        updated = settings.clone(**{f"aim.{k}": v for k, v in dataclasses.asdict(result.aim).items()})
        save(updated, args.settings)
    payload = result.as_dict()
    if args.json:
        _print_json(payload)
    else:
        print(f"baseline={payload['baseline_cost']} best={payload['cost']} improvement={payload['improvement']*100:.1f}%")
        print(_table(result.rows, ["scenario", "track_rms", "jitter", "on_target", "recovery_s"]))
        if args.write:
            print(f"\nwritten to {args.settings or DEFAULT_PATH}")
    return 0


def cmd_replay(args: argparse.Namespace) -> int:
    from .telemetry.recorder import load_session, summarize
    from .telemetry.replay import replay

    session = load_session(args.path)
    if args.summary:
        _print_json(summarize(session))
        return 0
    rows = [replay(session, k).as_row() for k in (["legacy", "predictive"] if args.compare else [args.controller])]
    if args.json:
        _print_json(rows)
    else:
        print(_table(rows, ["controller", "frames", "rms_px", "p95_px", "on_target", "jitter_px"]))
    return 0


def cmd_record(args: argparse.Namespace) -> int:
    return _run_engine(args, record=True)


# ---------------------------------------------------------------------------
# run
# ---------------------------------------------------------------------------
def cmd_run(args: argparse.Namespace) -> int:
    return _run_engine(args, record=args.record)


def _link_selfcheck(engine, counts: int = 20) -> None:
    """Send a tiny nudge at startup and verify the board actually received it.

    Without this, "no mouse movement" is ambiguous: the wire, the aim gate and the
    controller all live behind the same symptom.  Twenty counts is invisible in a
    menu and unmistakable in the log.
    """
    import time as _t

    link = engine.link
    status = getattr(link, "status", lambda: {})()
    # the serial port opens asynchronously (open + reconnect inside the link thread); do not write it off before it is up
    for _ in range(30):
        if status.get("mock") or status.get("ok"):
            break
        _t.sleep(0.05)
        status = getattr(link, "status", lambda: {})()
    if status.get("mock"):
        print("   link self-check: dry-run / synthetic link, nothing written to the serial port")
        return
    if not status.get("ok", False):
        print(f"!! link self-check failed: {status.get('detail') or 'serial port not open'}")
        return
    # the device answers a STATUS frame only every 100 ms; wait for its first word before asking whether it got it
    for _ in range(20):
        if (status.get("device") or {}).get("rx_frames") is not None:
            break
        _t.sleep(0.05)
        status = getattr(link, "status", lambda: {})()
    before = (status.get("device") or {}).get("rx_frames")
    link.submit_motion(float(counts), 0.0)
    _t.sleep(0.35)
    after = ((link.status() or {}).get("device") or {}).get("rx_frames")
    if before is None or after is None:
        print(f"   link self-check: connected to {status.get('port')} (no telemetry from the device, receipt unconfirmed)")
        return
    delta = after - before
    if delta > 0:
        print(f"   link self-check: the board saw {delta} frames, {counts} counts delivered (the cursor should twitch once)")
    else:
        print(f"!! link self-check: the board did not answer (rx {before}→{after}), check serial port/firmware")


def _run_engine(args: argparse.Namespace, record: bool = False) -> int:
    from .capture.sources import SyntheticSource, VideoSource
    from .engine.runtime import Engine
    from .transport.link import MockLink, build_link

    settings = load(args.settings)
    patches: Dict[str, Any] = {}
    if args.dry_run:
        patches["output.dry_run"] = True
    if args.hz:
        patches["runtime.control_hz"] = args.hz
    if args.report_hz:
        patches["output.report_hz"] = args.report_hz
    if record:
        patches["runtime.record"] = True
    if patches:
        settings = set_fields(settings, patches)

    metrics = Metrics()
    source = None
    detector = None
    pace = 0
    if args.sim:
        source = SyntheticSource(settings.capture.width, settings.capture.height, 60)
        from .sim.fake_detector import SyntheticDetector

        detector = SyntheticDetector()
        pace = 0
    elif args.video:
        source = VideoSource(args.video)
        pace = 60

    link = MockLink(settings.output, metrics) if (args.sim or args.dry_run) else build_link(settings.output, metrics)
    engine = Engine(settings, metrics=metrics, source=source, link=link, detector=detector, pace_fps=pace)

    hotkeys = None
    if not args.no_hotkeys:
        try:
            from pynput import keyboard

            def on_press(key):
                try:
                    if key == keyboard.Key.space:
                        engine.set_aim_enabled(not engine.aim_enabled)
                    elif key.char and key.char.lower() == "f":
                        engine.set_fire_enabled(not engine.fire_enabled)
                except AttributeError:
                    pass

            hotkeys = keyboard.Listener(on_press=on_press)
            hotkeys.start()
        except Exception:
            hotkeys = None

    engine.start()
    if args.aim:
        engine.set_aim_enabled(True)
    if args.fire:
        engine.set_fire_enabled(True)

    if not args.quiet:
        print(
            f"=== aim {'ON ' if engine.aim_enabled else 'OFF'} | fire {settings.fire.mode} | "
            f"model {engine.detector.model_name} | head {settings.vision.head_mode} | "
            f"gain {settings.output.gain} | protocol {settings.output.protocol} | control {settings.runtime.control_hz}Hz ==="
        )
        if not engine.aim_enabled:
            print("!! aim is disabled: start with --aim, or press space while running (the terminal needs Accessibility permission)")
        if settings.vision.head_mode == "ratio" and getattr(engine.detector, "names", None) is not None:
            print(f"   head ratio policy.head_ratio={settings.policy.head_ratio} (used when the model has no head class)")
        print("   For study and research only: do not use it in any competitive/ranked context (see DISCLAIMER.md)")
        _link_selfcheck(engine)

    t0 = time.monotonic()
    last_print = 0.0
    try:
        from .core.clock import FixedRate

        for _ in FixedRate(settings.runtime.control_hz):
            snap = engine.step()
            now = time.monotonic()
            if now - last_print >= 0.5 and not args.quiet:
                last_print = now
                u = snap.output
                extra = f" u=({u.dx:5.1f},{u.dy:5.1f})" if u else " u=0(no output)"
                print(f"\r{snap.top_line()}{extra}", end="", flush=True)
            if args.duration and now - t0 >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print()
        engine.stop()
        if hotkeys is not None:
            hotkeys.stop()
        if not args.quiet:
            _print_json(engine.describe())
    return 0


# ---------------------------------------------------------------------------
# selftest: run a real closed loop and answer "why it does not move / why it does not fire"
# ---------------------------------------------------------------------------
def cmd_selftest(args: argparse.Namespace) -> int:
    import time

    from .core.clock import FixedRate
    from .engine.runtime import Engine
    from .transport.link import build_link

    settings = load(args.settings)
    if args.no_aim:
        settings = set_fields(settings, {"fire.mode": "off"})
    metrics = Metrics()
    link = build_link(settings.output, metrics, force_mock=args.dry_run or args.sim)
    source = None
    detector = None
    if args.sim:
        from .capture.sources import SyntheticSource
        from .sim.fake_detector import SyntheticDetector

        source = SyntheticSource(settings.capture.width, settings.capture.height, 120)
        detector = SyntheticDetector()
    engine = Engine(settings, source=source, link=link, detector=detector, metrics=metrics)
    engine.start()
    if not args.no_aim:
        engine.set_aim_enabled(True)
        engine.set_fire_enabled(settings.fire.mode != "off")

    t0 = time.monotonic()
    ticks = 0
    snap = None
    try:
        for _ in FixedRate(settings.runtime.control_hz):
            snap = engine.step()
            ticks += 1
            if time.monotonic() - t0 >= args.seconds:
                break
    finally:
        describe = engine.describe()
        engine.stop()

    counters = metrics.counters()
    stats = metrics.stats_summary()
    dets = counters.get("perception.det_objects", 0.0)
    target_ticks = counters.get("policy.target_ticks", 0.0)
    reports = counters.get("transport.reports", 0.0)
    frames = counters.get("capture.frames", 0.0)
    fire_reasons = {k.split("fire.reason.")[-1]: int(v) for k, v in counters.items() if k.startswith("fire.reason.")}

    findings: List[str] = []
    if frames < args.seconds * 10:
        findings.append(f"capture frame rate too low ({frames/args.seconds:.1f} fps): capture card / permissions / game output")
    if dets == 0:
        findings.append("no detection box for the whole run: model / thresholds / frame content")
    if target_ticks == 0 and dets > 0:
        findings.append("detections but never a selected target: FOV / masks / classes / unconfirmed track")
    if reports == 0 and target_ticks > 0:
        findings.append("target locked but the controller outputs nothing: gain is 0 or the dead band is too wide")
    if args.no_aim:
        findings.append("this test disabled aim and only verified perception and link")
    if fire_reasons.get("no-target", 0) > ticks * 0.8:
        findings.append("fire was judged no-target for most of the run (usually aim disabled or no lock)")
    if fire_reasons.get("already-fired", 0) > ticks * 0.5:
        findings.append("single mode: a shot has already been fired, this is normal")
    if snap is not None and snap.health.value != "ok":
        findings.append(f"health {snap.health.value}: {snap.health_detail}")

    payload = {
        "ok": not findings or all("normal" in f for f in findings),
        "seconds": args.seconds,
        "ticks": ticks,
        "aim_enabled": False if args.no_aim else True,
        "health": describe["health"],
        "health_detail": describe["detail"],
        "capture_fps": round(frames / max(1e-6, args.seconds), 1),
        "infer_ms": stats.get("perception.infer_ms", {}).get("p50", 0.0),
        "dets_total": int(dets),
        "target_ticks": int(target_ticks),
        "target_ratio": round(target_ticks / max(1, ticks), 3),
        "motion_reports": int(reports),
        "error_p50_px": stats.get("aim.err_px", {}).get("p50", 0.0),
        "latency": describe["latency"],
        "fire_reasons": fire_reasons,
        "findings": findings,
    }
    if args.json:
        _print_json(payload)
    else:
        print(f"duration {args.seconds:.0f}s / {ticks} tick | health {payload['health']} {payload['health_detail']}")
        print(f"capture {payload['capture_fps']} fps | inference p50 {payload['infer_ms']} ms | detections {payload['dets_total']}")
        print(f"lock ratio {payload['target_ratio']*100:.0f}% | motion reports {payload['motion_reports']} | error p50 {payload['error_p50_px']} px")
        print(f"latency {payload['latency'].get('loop_ms')} ms (confidence {payload['latency'].get('confidence')})")
        print(f"fire reasons: {fire_reasons or '{}'}")
        print("\nverdict: " + ("normal" if not findings else "\n  - " + "\n  - ".join(findings)))
    return 0


# ---------------------------------------------------------------------------
# link-test: talk to the firmware and tell whether the peer is v2 / v1 / not running
# ---------------------------------------------------------------------------
def cmd_link_test(args: argparse.Namespace) -> int:
    import time

    import serial

    from .transport import protocol as P
    from .transport.serial_link import SerialLink

    try:
        port = args.port or SerialLink.resolve_port()
    except Exception as exc:
        _print_json({"ok": False, "error": str(exc)})
        return 1

    parser = P.StreamParser()
    seen: Dict[str, int] = {"bytes": 0, "frames": 0, "status": 0, "move": 0, "other": 0}

    try:
        ser = serial.Serial(port, int(args.baud), timeout=0.1, write_timeout=0.05)
    except Exception as exc:
        _print_json({"ok": False, "port": port, "error": f"{type(exc).__name__}: {exc}"})
        return 1

    try:
        # never reset the board just by looking at it
        ser.dtr = False
        ser.rts = False
        ser.reset_input_buffer()

        def drain(seconds: float) -> None:
            t_end = time.monotonic() + seconds
            while time.monotonic() < t_end:
                chunk = ser.read(ser.in_waiting or 1)
                if not chunk:
                    continue
                seen["bytes"] += len(chunk)
                for kind, payload, _seq in parser.feed(chunk):
                    seen["frames"] += 1
                    if kind == P.T_STATUS:
                        seen["status"] += 1
                        nonlocal_device = P.decode_status(payload)
                        if nonlocal_device is not None:
                            seen["last_status"] = nonlocal_device  # type: ignore[assignment]
                    elif kind == P.T_MOVE:
                        seen["move"] += 1
                    else:
                        seen["other"] += 1

        drain(0.8)                                    # passive listen: v2 firmware reports every 100 ms
        ser.write(P.encode_ping(seen["frames"], 1))
        ser.flush()
        drain(0.6)
        sys.stdout.flush()

        if not seen["frames"]:
            # maybe it is a v1 firmware: it only listens, so a 5 px nudge is the
            # only way to prove it is alive (harmless, it is 5 px)
            if args.poke:
                for _ in range(3):
                    ser.write(P.legacy_move(5, 0, seen["bytes"] & 0xFF))
                    ser.flush()
                    time.sleep(0.05)
            ser.write(P.legacy_heartbeat(1))
            ser.flush()
            drain(0.4)
    finally:
        try:
            ser.close()
        except Exception:
            pass

    st = seen.get("last_status")
    if seen["status"]:
        verdict = "v2 firmware online"
    elif seen["frames"]:
        verdict = "v2 frame format but an unexpected type"
    elif seen["bytes"]:
        verdict = "bytes present but not v2 frames (baud rate or firmware generation mismatch)"
    elif args.poke:
        verdict = "no telemetry; sent +5px three times (a twitching cursor on Windows means a v1 firmware is alive)"
    else:
        verdict = "peer silent: it may only listen (v1), the firmware may not be running, or it needs flashing"

    payload = {
        "ok": bool(seen["status"]),
        "port": port,
        "baud": int(args.baud),
        "bytes": seen["bytes"],
        "frames": seen["frames"],
        "status_frames": seen["status"],
        "crc_errors": parser.crc_errors,
        "verdict": verdict,
        "device": None if st is None else {
            "uptime_s": st.uptime_ms / 1000.0,
            "rx_frames": st.rx_frames,
            "crc_errors": st.crc_errors,
            "queue_drops": st.queue_drops,
            "report_hz": st.report_hz,
            "deadman": bool(st.flags & 0x01),
            "usb_ready": bool(st.flags & 0x02),
            "hid_stalled": bool(st.flags & 0x04),
        },
    }
    if args.json:
        _print_json(payload)
    else:
        print(f"port {port} @ {args.baud}")
        print(f"received {payload['bytes']} bytes / {payload['frames']} frames / STATUS {payload['status_frames']} frames / CRC errors {parser.crc_errors}")
        if payload["device"]:
            d = payload["device"]
            print(f"device: uptime={d['uptime_s']:.1f}s rx={d['rx_frames']} crc={d['crc_errors']} "
                  f"drops={d['queue_drops']} {d['report_hz']}Hz deadman={d['deadman']} "
                  f"usb_ready={d.get('usb_ready')} hid_stalled={d.get('hid_stalled')}")
        print(f"verdict: {verdict}")
    return 0 if payload["ok"] else 3


# ---------------------------------------------------------------------------
# config
# ---------------------------------------------------------------------------
def cmd_config(args: argparse.Namespace) -> int:
    settings = load(args.settings)
    if args.action == "show":
        if args.json:
            _print_json(to_dict(settings))
        else:
            for section in ("capture", "vision", "tracker", "aim", "policy", "fire", "output", "runtime"):
                print(f"[{section}]")
                for key, value in to_dict(settings)[section].items():
                    print(f"  {section}.{key} = {value}")
            if settings.warnings:
                print("\nwarnings:")
                for w in settings.warnings:
                    print("  -", w)
        return 0
    if args.action == "validate":
        try:
            validate(settings)
        except ConfigError as exc:
            print(str(exc))
            return 1
        print(f"configuration valid (schema {settings.schema_version}, profile={settings.profile})")
        return 0
    if args.action == "get":
        print(get_field(settings, args.key))
        return 0
    if args.action == "set":
        value: Any = args.value
        try:
            value = json.loads(args.value)
        except json.JSONDecodeError:
            pass
        updated = set_fields(settings, {args.key: value})
        path = save(updated, args.settings)
        print(f"{args.key}: {get_field(settings, args.key)} -> {get_field(updated, args.key)}  ({path})")
        return 0
    if args.action == "migrate":
        path = save(settings, args.settings)
        print(f"migrated/normalised and written to {path} (schema {SCHEMA_VERSION})")
        return 0
    if args.action == "diff":
        other = load(args.other)
        changes = diff(settings, other)
        if not changes:
            print("no differences")
            return 0
        for key, (old, new) in changes.items():
            print(f"{key}: {old} -> {new}")
        return 0
    raise SystemExit(f"unknown action {args.action}")


# ---------------------------------------------------------------------------
# calibrate
# ---------------------------------------------------------------------------
def cmd_calibrate(args: argparse.Namespace) -> int:
    settings = load(args.settings)
    if args.action == "model":
        from .perception.detector import OnnxDetector
        from .tuning.calibration import model_report, suggest

        det = OnnxDetector(settings.vision)
        rep = model_report(det)
        _print_json({**rep, "suggestions": suggest(settings, rep)})
        return 0 if rep["ok"] else 1

    if args.action == "latency":
        from .capture.sources import SyntheticSource
        from .engine.runtime import Engine
        from .transport.link import MockLink
        from .tuning.calibration import wait_for_latency

        metrics = Metrics()
        source = SyntheticSource(settings.capture.width, settings.capture.height, 60)
        engine = Engine(settings, metrics=metrics, source=source, link=MockLink(settings.output, metrics))
        engine.start()
        try:
            print("collecting 8 seconds of data for latency identification (a target must be visible)...")
            result = wait_for_latency(engine, seconds=args.seconds)
        finally:
            engine.stop()
        _print_json(result)
        return 0

    if args.action == "gain":
        from .capture.sources import SyntheticSource
        from .engine.runtime import Engine
        from .transport.link import build_link
        from .tuning.calibration import measure_counts_per_px

        metrics = Metrics()
        link = build_link(settings.output, metrics, force_mock=args.dry_run)
        engine = Engine(settings, metrics=metrics, link=link)
        engine.start()
        try:
            result = measure_counts_per_px(engine, seconds=args.seconds, probe_counts=args.probe)
        finally:
            engine.stop()
        payload = {**result, "hint": f"aim2 config set output.gain {result.get('counts_per_px', 1.0)}"}
        if args.json:
            _print_json(payload)
        else:
            for k, v in payload.items():
                print(f"{k:16s} {v}")
        return 0 if result.get("ok") else 1

    if args.action == "head":
        from .telemetry.recorder import load_session
        from .tuning.calibration import class_names, head_ratio_from_session

        session = load_session(args.path)
        names = class_names(session)
        got = head_ratio_from_session(session)
        if got is None:
            _print_json({"ok": False, "classes": names,
                         "detail": "the session never had head and body detections at the same time, calibration not possible"})
            return 1
        ratio, n = got
        _print_json({"ok": True, "head_ratio": round(ratio, 4), "samples": n,
                     "hint": f"aim2 config set policy.head_ratio {round(ratio, 4)}"})
        return 0

    raise SystemExit(f"unknown action {args.action}")


# ---------------------------------------------------------------------------
# parser
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="aim2", description="industrial-grade visual aim system (v2)")
    p.add_argument("--settings", default=None, help=f"configuration file path (default {DEFAULT_PATH})")
    p.add_argument("--json", action="store_true", help="machine readable output")
    sub = p.add_subparsers(dest="cmd", required=True)

    d = sub.add_parser("doctor", help="environment and model health check")
    d.add_argument("--probe", action="store_true", help="also run an inference benchmark")
    d.add_argument("--probe-runs", type=int, default=20)
    d.add_argument("--dry-run", action="store_true", help="skip the serial port check")
    d.set_defaults(func=cmd_doctor)

    m = sub.add_parser("models", help="model registry")
    m.add_argument("action", choices=["list", "inspect", "bench"])
    m.add_argument("path", nargs="?")
    m.add_argument("--runs", type=int, default=20)
    m.set_defaults(func=cmd_models)

    s = sub.add_parser("sim", help="closed-loop simulation")
    s.add_argument("--scenario")
    s.add_argument("--controller", default="predictive", choices=["predictive", "legacy"])
    s.add_argument("--compare", action="store_true")
    s.add_argument("--control-hz", type=int, default=240, dest="control_hz")
    s.add_argument("--detect-hz", type=int, default=60, dest="detect_hz")
    s.add_argument("--latency-ms", type=float, default=33.0, dest="latency_ms")
    s.add_argument("--noise", type=float, default=2.5)
    s.set_defaults(func=cmd_sim)

    b = sub.add_parser("bench", help="regression gate: the new control law must beat v1")
    b.set_defaults(func=cmd_bench)

    a = sub.add_parser("autotune", help="offline auto-tuning")
    a.add_argument("--passes", type=int, default=2)
    a.add_argument("--write", action="store_true", help="write back to the configuration file")
    a.set_defaults(func=cmd_autotune)

    r = sub.add_parser("replay", help="replay a recorded session")
    r.add_argument("path")
    r.add_argument("--controller", default="predictive", choices=["predictive", "legacy"])
    r.add_argument("--compare", action="store_true")
    r.add_argument("--summary", action="store_true")
    r.set_defaults(func=cmd_replay)

    run = sub.add_parser("run", help="start the engine")
    run.add_argument("--sim", action="store_true", help="synthetic frames, no hardware needed")
    run.add_argument("--video", help="use a video file as the frame source")
    run.add_argument("--dry-run", action="store_true", help="never write to the serial port")
    run.add_argument("--duration", type=float, default=None, help="run duration in seconds")
    run.add_argument("--aim", action="store_true", help="enable aim at startup")
    run.add_argument("--fire", action="store_true", help="enable fire at startup")
    run.add_argument("--hz", type=int, default=0, help="override the control rate")
    run.add_argument("--report-hz", type=int, default=0, dest="report_hz")
    run.add_argument("--record", action="store_true")
    run.add_argument("--no-hotkeys", action="store_true")
    run.add_argument("--quiet", action="store_true")
    run.set_defaults(func=cmd_run)

    rec = sub.add_parser("record", help="record a session")
    rec.add_argument("--sim", action="store_true")
    rec.add_argument("--video")
    rec.add_argument("--dry-run", action="store_true")
    rec.add_argument("--duration", type=float, default=10.0)
    rec.add_argument("--aim", action="store_true")
    rec.add_argument("--fire", action="store_true")
    rec.add_argument("--hz", type=int, default=0)
    rec.add_argument("--report-hz", type=int, default=0, dest="report_hz")
    rec.add_argument("--no-hotkeys", action="store_true")
    rec.add_argument("--quiet", action="store_true")
    rec.set_defaults(func=cmd_record)

    st = sub.add_parser("selftest", help="run a real closed loop and print a diagnosis")
    st.add_argument("--seconds", type=float, default=8.0)
    st.add_argument("--no-aim", action="store_true", help="verify perception and link only, never move the mouse")
    st.add_argument("--dry-run", action="store_true")
    st.add_argument("--sim", action="store_true", help="synthetic frames, no camera permission needed")
    st.set_defaults(func=cmd_selftest)

    lt = sub.add_parser("link-test", help="talk to the ESP32 firmware, identify its generation and the link")
    lt.add_argument("--port", default=None, help="auto-discovered by default")
    lt.add_argument("--baud", type=int, default=921600)
    lt.add_argument("--poke", action="store_true", help="send a +5px nudge to probe a listen-only v1 firmware")
    lt.set_defaults(func=cmd_link_test)

    c = sub.add_parser("config", help="configuration management")
    c.add_argument("action", choices=["show", "get", "set", "validate", "migrate", "diff"])
    c.add_argument("key", nargs="?")
    c.add_argument("value", nargs="?")
    c.add_argument("--other")
    c.set_defaults(func=cmd_config)

    cal = sub.add_parser("calibrate", help="calibration")
    cal.add_argument("action", choices=["model", "latency", "head", "gain"])
    cal.add_argument("--probe", type=int, default=160, help="mouse counts sent by gain calibration")
    cal.add_argument("--path")
    cal.add_argument("--seconds", type=float, default=8.0)
    cal.set_defaults(func=cmd_calibrate)

    g = sub.add_parser("gui", help="open the desktop console (PySide6)")
    g.add_argument("--sim", action="store_true")
    g.add_argument("--dry-run", action="store_true")
    g.add_argument("--aim", action="store_true")
    g.set_defaults(func=lambda a: __import__("dma.desktop.app", fromlist=["main"]).main(
        (["--sim"] if a.sim else []) + (["--dry-run"] if a.dry_run else []) + (["--aim"] if a.aim else [])))

    v = sub.add_parser("version", help="version")
    v.set_defaults(func=lambda a: (print(f"aim2 {VERSION} (schema {SCHEMA_VERSION})"), 0)[1])

    # allow ``aim2 <cmd> --json`` as well as ``aim2 --json <cmd>``
    for action in sub.choices.values():
        action.add_argument("--json", action="store_true", default=argparse.SUPPRESS,
                            help="machine readable output")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if getattr(args, "settings", None) is None:
        args.settings = None
    setup_logging(level="INFO", log_dir=None, console=True)
    try:
        return int(args.func(args) or 0)
    except ConfigError as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
