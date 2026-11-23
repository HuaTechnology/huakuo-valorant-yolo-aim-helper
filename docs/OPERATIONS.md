<div align="right">

[简体中文](zh-CN/OPERATIONS.md) | **English**

</div>

# Operations Handbook (OPERATIONS)

> ⚠️ **For learning, research and engineering study only.**
> Do not use this project in any competitive or ranked setting. Automated aiming
> violates the terms of service of most online games and may lead to permanent bans.
> You are solely responsible for how you use it.

> Code baseline: `src/dma`, schema 2, `aim2 2.0.0`. Defaults come from `src/dma/core/config.py`.
> This document targets the operations scenario "get the system running, keep an eye on it, locate problems when they happen, and be able to roll back"; for principles see [ARCHITECTURE](./ARCHITECTURE.md), for parameter semantics see [TUNING](./TUNING.md).

---

## 1. Installation and first launch

### 1.1 Dependencies

| Item | Requirement | Check |
|---|---|---|
| Python | ≥ 3.10 (`cmd_doctor` checks `sys.version_info >= (3,10)`) | `aim2 doctor` |
| venv | `.venv` at the repository root; `pip install -e .` or `PYTHONPATH=src` | `python -c "import dma"` |
| Python packages | core dependencies from `pyproject.toml` (numpy/opencv-python/onnxruntime/pyserial); optional `[ui]` PySide6, `[hotkeys]` pynput, `[dev]` pytest. `requirements.txt` is the more complete pinned list (includes torch/ultralytics) | the onnxruntime/opencv/pyserial items of `aim2 doctor` |
| Model | `models/*.onnx` | `aim2 models list` |
| Camera permission | macOS: System Settings → Privacy & Security → Camera, tick your terminal/iTerm/IDE | check first when there is no picture |
| Accessibility permission | needed for global hotkeys (`pynput`) | check when hotkeys do nothing |
| Serial port | `/dev/cu.usbmodem*`, `/dev/cu.wchusbserial*`, `/dev/ttyUSB*`, `/dev/ttyACM*`, `COM*` | `aim2 doctor` |

Install (an editable install is recommended so you get the `aim2` console script):

```bash
cd ~/dma
python3 -m venv .venv
.venv/bin/python -m pip install -U pip
.venv/bin/python -m pip install -r requirements.txt     # or: pip install -e ".[ui,hotkeys,dev]"
.venv/bin/python -m pip install -e .                    # generates .venv/bin/aim2
```

`[project.scripts]` in `pyproject.toml` defines `aim2 = dma.cli:main`; if this `.venv` was not installed editable, `aim2` does not exist, use the module form instead:

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python -m dma.cli --help
```

It is recommended to add an alias in `~/.zshrc` (when the console script is not installed):

```bash
export DMA_HOME="$HOME/dma"
alias aim2='PYTHONPATH="$DMA_HOME/src" "$DMA_HOME/.venv/bin/python" -m dma.cli'
```

`make` targets (`Makefile`): `make test` / `make bench` / `make sim` / `make doctor` / `make run` / `make autotune`, internally also `.venv/bin/python` + `PYTHONPATH=src`.

### 1.2 Configuration sources and first migration

- Default path: `configs/settings.json` (`core/config.py::DEFAULT_PATH`).
- Legacy path: `config.json` (`LEGACY_PATH`).
- On the first `load()`:
  - `configs/settings.json` exists → read it directly;
  - it does not exist but `config.json` does → migrate v1 **in memory** (`migrate_v1`), nothing is written;
  - neither exists → use the dataclass defaults.

For a fresh install that should not inherit v1 parameters, use the example configuration shipped with the repository:

```bash
cp configs/settings.example.json configs/settings.json
```

The example configuration is pure schema 2 defaults (`vision.model` points to `models/model.onnx`, `output.gain=1.0`). If you migrate from v1, the model path and the conversion factor carry over the old values; you can only pick one of the two as a starting point.

Key migration results (measured on the current `config.json`):

| v1 field | v2 field | Current value | Note |
|---|---|---|---|
| `SINK_GAIN` | `output.gain` | 3.0 | v1 user dragged it to max |
| `max_step` | `aim.max_step` | 112.08 | |
| `kp_far` | `aim.kp_far` | 0.45 | |
| `kp_mid`/`kp_near`/`kp_touch` | `aim.kp_near` | 0.7297 | **the last one written is `kp_touch`**; after migration `kp_near` holds the v1 point-blank gain and must be re-checked |
| `kd`/`ki` | `aim.kd`/`aim.ki` | 0.388 / 0.2 | |
| `COAST_FRAMES` | `policy.coast_s` | 0.1 | converted at 60 fps (6/60) |
| `HEAD_RATIO` | `policy.head_ratio` | 0.1386 | |
| `FIRE_RADIUS` | `fire.radius` | 16.0 | |
| `PREDICT_*` / `ema_*` | — | dropped | the predictive controller takes over; written into warnings |
| `latency_prior_s` | forced to 0.033 | 0.033 | the v1 "1 frame" claim is not trusted |

`aim2 config show` prints these warnings. **Persist a schema 2 baseline before the first live run**:

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli config migrate
PYTHONPATH=src .venv/bin/python -m dma.cli config validate
PYTHONPATH=src .venv/bin/python -m dma.cli config show > out/config_baseline.txt
```

`config migrate` writes the currently effective configuration to `configs/settings.json`; from then on v2 only reads and writes that file, and `config.json` stays untouched (used for rollback).

### 1.3 Model selection

| Model | Suitable for | Note |
|---|---|---|
| YOLOv8 320×320 export (dataclass default) | has `head`/`teammate`, can lock the head and ignore teammates | `yolov8` output, letterbox mandatory; verify with `aim2 models inspect` |
| End-to-end 320×320 export (example) | only the `st` body class, measured adequate at 60 fps | no head/teammate; head locking relies on `head_ratio`; team modes suffer from teammates |
| YOLOv5 640×640 export | has head/body, 640 input | `yolov5` output; inference at 640 is slower |

Switching:

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli config set vision.model YOLOv8 320×320 export
PYTHONPATH=src .venv/bin/python -m dma.cli models inspect YOLOv8 320×320 export --json
# vision.* changes require a process restart (Engine.apply only prints a hint, it does not rebuild the detector)
```

### 1.4 Protocol and firmware

The host supports two frame formats:

| `output.protocol` | Frame format | Device telemetry | Suitable for |
|---|---|---|---|
| `v2` (default) | `0xA5` + version/type + seq + len + payload + CRC8/ATM | yes (`STATUS`) | v2 firmware |
| `v1` | `0xAA` 10 bytes + XOR | no | legacy firmware |

**The host's `output.protocol` must match the firmware**; on a mismatch the symptom is "the serial port opens, bytes are written, the game does not move". The firmware lives in `firmware/esp32_hid/` (`esp32_hid.ino` + `platformio.ini` + `README.md`) and only does "serial frame → 1000 Hz HID"; STATUS is 13 bytes of `"<IHHHHB"`, consistent with `protocol.STATUS_LEN=13`. Flashing settings are in README §2 (must be `USB-OTG (TinyUSB)` + `USB CDC On Boot: Enabled`).

---

## 2. `run_mac.sh` and `aim2`

### 2.1 `run_mac.sh`

The script hardcodes `PROJ="$HOME/dma"`, first `cd`s to the repository root, sets `PYTHONPATH=src`, then dispatches by mode. The default mode is `v2`.

**v2 entry points (recommended)**

| Mode | What it actually does | Hardware | Note |
|---|---|---|---|
| `./run_mac.sh run` | `python -m dma.cli run --aim` (further arguments are passed through verbatim) | capture card + serial | logs are tee'd to `out/logs/` |
| `./run_mac.sh run-dry` | `run --aim --dry-run` | capture card (serial optional) | MockLink, no serial writes; safe integration testing |
| `./run_mac.sh gui` | `python -m dma.cli gui --aim` | capture card + serial (or add `--sim`) | PySide6 console |
| `./run_mac.sh sim` | `run --sim --aim --dry-run` | none | synthetic frames + `SyntheticDetector`, full loop |
| `./run_mac.sh doctor` | `doctor --probe` | serial (a missing port is reported as a blocking item) | environment/model/serial check-up + inference benchmark |
| `./run_mac.sh selftest` | `selftest` | capture card + serial | 8-second closed-loop self-test with a verdict |
| `./run_mac.sh bench` | `bench` | none | 6-scenario regression gate; exits 2 on failure |

All paths in `run_mac.sh` are relative to the script itself (`$(dirname "$0")`), so it works from any clone directory;
the Python interpreter prefers `.venv/bin/python`, falls back to `python3`, and can be overridden with `PYTHON=`.

`scripts/aim2` is the equivalent no-install entry point (sets `PYTHONPATH` automatically, no arguments means `gui --aim`).

---|---|---|---|
| `./run_mac.sh gui` | `launchctl asuser` into the Aqua session, runs `src/dma/main.py` → `qt_gui.py` | yes (can use `MAC_PW`) | v1 PySide6 GUI, does not drive the v2 Engine |
| `./run_mac.sh tune` | `src/dma/gui.py` → `qt_gui.py` (live tuning GUI) | yes | v1 |
| `./run_mac.sh tune-demo` | `gui.py --demo`, synthetic frames | no | v1 |
| `./run_mac.sh headless` | stubs `cv2.imshow`, runs `main.py` | no | v1 without a window |
| `./run_mac.sh stop` | `pkill -f "dma.cli run"` and `pkill -f "dma/main.py"` | no | stops v2/v1 together |

Notes:

- `run_mac.sh` `shift`s away the first argument, so `./run_mac.sh run --duration 30` passes `--duration 30` through to the CLI.
- `v2` already carries `--aim`; to run without aiming use the CLI directly, or toggle with the space bar after startup.
- If `gui` needs `--sim`, add it yourself: `./run_mac.sh gui --sim`.
- The log tee only captures stdout/stderr text; the JSONL log still has to be enabled explicitly per §4.2.

### 2.2 `aim2` subcommands

Common to all subcommands: `--settings PATH` (global, placed before the subcommand), `--json` (before or after the subcommand). Without arguments the default configuration is used.

| Subcommand | Purpose | Hardware needed | Typical command |
|---|---|---|---|
| `version` | prints `aim2 2.0.0 (schema 2)` | no | `aim2 version` |
| `doctor` | environment/model/class/serial check-up; `--probe` adds the inference benchmark | needed for the serial check (`--dry-run` skips it) | `aim2 doctor --dry-run --json` |
| `models list` | scans `models/*.onnx`, loads each and reports format/classes | no | `aim2 models list` |
| `models inspect PATH` | details of a single model | no | `aim2 models inspect YOLOv8 320×320 export --json` |
| `models bench PATH` | benchmark of a single model (the CLI uses random 1280×720 frames; the 2560×1600 numbers in the report are in `docs/MEASUREMENTS.md`) | no | `aim2 models bench end-to-end 320×320 export --runs 20` |
| `sim` | closed-loop simulation; `--compare` runs legacy/predictive as well | no | `aim2 sim --scenario strafe_mid --json` |
| `bench` | regression gate: 6 scenarios compared against v1, exits 2 on failure | no | `aim2 bench` |
| `autotune` | offline coordinate descent; `--write` writes back the `aim` section | no | `aim2 autotune --passes 2 --write` |
| `replay PATH` | replays a session; `--summary` only the summary; `--compare` two controllers | no | `aim2 replay out/telemetry/session_x.jsonl.gz --summary` |
| `run` | starts the Engine; `--sim` uses synthetic frames + `SyntheticDetector` (no onnxruntime/camera needed), `--video` uses a video; the rest are `--dry-run/--record/--aim/--fire/--duration/--hz/--report-hz/--no-hotkeys/--quiet` | capture card + serial by default; `--sim` needs neither | `aim2 run --dry-run --duration 20` |
| `record` | recording (shortcut for `run --record`, 10 seconds by default) | same as above | `aim2 record --sim --duration 10` |
| `config show/get/set/validate/migrate/diff` | configuration management | no | `aim2 config set aim.kp_far 0.8` |
| `calibrate model/latency/head` | calibration helpers | `head` needs none; `latency` uses a synthetic source; `model` needs none | `aim2 calibrate head --path ...` |

Exit codes:

| Code | Meaning |
|---|---|
| 0 | success |
| 1 | `doctor` has blocking items / `models inspect` model not OK / `calibrate head` not enough samples |
| 2 | `bench` regression failure / configuration error (`ConfigError`) / argparse usage error |
| 130 | Ctrl-C |

### 2.3 Desktop console (PySide6)

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python -m dma.desktop.app            # live hardware
PYTHONPATH=src .venv/bin/python -m dma.desktop.app --sim      # synthetic frames, no hardware
PYTHONPATH=src .venv/bin/python -m dma.desktop.app --dry-run  # no serial writes
# or:
./run_mac.sh gui            # live hardware; needs sudo/launchctl
./run_mac.sh gui --sim      # synthetic frames
```

Structure of `desktop/app.py`:

| Component | Purpose |
|---|---|
| `EngineThread` (QThread) | runs `Engine.run()` in its own thread so UI stalls do not affect control |
| `MainWindow._tick` (QTimer 33 ms) | only reads `Engine.snapshot()` and refreshes preview/HUD/status lights |
| `ParamRow` | generated automatically from `core.config.FIELDS`; range/unit/help text come from the registry |
| `_on_param` | `set_fields` validation → `Engine.apply` hot reload; on failure the widget value is rolled back and the log panel gets an entry |
| `PreviewWidget` | draws detection boxes, track dashed boxes, target box (red), crosshair, dead-band circle |
| Top bar | health state, aim/fire toggles, model dropdown (display only, switching models needs a restart), save/reload configuration |
| HUD | `cap/ctl/inf/err/mode/lat(conf)/det/trk/fire`, same source as `Snapshot.top_line` |

Notes:

- The desktop app calls `setup_logging(console=False, log_dir=None)`: it **writes neither to the console nor to disk**; the log panel only shows the UI's own configuration/restart hints, not engine logs.
- When you change `vision.*`/`capture.*`/`output.port|baud`, `Engine.apply` only returns a "restart needed" hint and does not rebuild the detector/capture card/serial port.
- Closing the window calls `engine.stop()` and waits 1 second for `EngineThread`.
- The model dropdown only affects the next start; do not expect to switch models while running.

### 2.4 Recommended startup order

```bash
cd ~/dma
export PYTHONPATH=src

# 1) check-up (does not touch the serial port)
.venv/bin/python -m dma.cli doctor --dry-run

# 2) configuration baseline (first time)
.venv/bin/python -m dma.cli config migrate
.venv/bin/python -m dma.cli config validate

# 3) hardware-free smoke test: synthetic frames + SyntheticDetector + MockLink (no onnxruntime needed)
.venv/bin/python -m dma.cli run --sim --duration 10 --no-hotkeys

# 4) dry run: real capture card + MockLink, no aiming
.venv/bin/python -m dma.cli run --dry-run --duration 20 --no-hotkeys

# 5) live hardware (no firing yet)
.venv/bin/python -m dma.cli run --aim --no-hotkeys
# when you need firing, drop --no-hotkeys and toggle with the F key; or pass --fire directly

# 6) regression/unit tests (do not touch hardware)
.venv/bin/python -m pytest -q
```

`run_mac.sh run` equals step 5 and adds `--aim` automatically; `run_mac.sh sim` equals step 3; `run_mac.sh gui` opens the desktop console.

`aim2 run` enables global hotkeys by default: space toggles aiming, `f` toggles firing (`cli._run_engine`). Without the accessibility permission the hotkey listener fails but does not block the run.

`aim2 run` refreshes `Snapshot.top_line()` in the terminal every 0.5 seconds, and prints `engine.describe()` on exit (health, model, capture, link, latency, fire count, frame size).

---

## 3. Health states

`Health` has five states (`core/types.py`); `Engine._evaluate` only produces `OK/DEGRADED`:

| State | Meaning | Aiming/firing allowed | Typical scene |
|---|---|---|---|
| `BOOT` | constructed, not started | no | the instant of startup |
| `OK` | capture/inference/link are all healthy | yes | normal operation |
| `DEGRADED` | some stage is unhealthy, `health_detail` gives the reason | **no** (`can_aim=False`) | see the table below |
| `FAILED` | reserved for unrecoverable errors; **currently no assignment path** | no | do not rely on it for alerts |
| `STOPPED` | after `Engine.stop()` | no | clean exit |

The four `DEGRADED` reasons and their handling:

| `health_detail` | Trigger (code) | Threshold (default) | What to check first | Handling |
|---|---|---|---|---|
| `capture stream interrupted` | `grabber.stale_s()*1000 > runtime.watchdog_ms` | 400 ms | whether the capture card has an HDMI signal; camera permission; whether OBS/another program has it; USB power/cable | replug the card/change port; close the program holding it; `aim2 doctor` for the camera; returns to `OK` automatically once recovered |
| `waiting for the first inference` | `det is None` | — | whether the model loaded; whether the inference thread crashed; whether capture just started | wait 1–2 seconds for warm-up; if it persists check the log `inference failed` / `perception.errors` |
| `inference result stale` | `now - det.capture_ts > runtime.max_infer_age_s` | 120 ms | whether `perception.infer_ms` p95 spiked; whether capture fps dropped to 0; whether the control thread is blocked | lower the resolution/use a smaller model; check CPU/GPU usage; see whether `control.hz` dropped |
| `output link down` | `not link.ok()` | port not open or 5 consecutive write errors | `link.status()["detail"]` (exception type); whether the serial port is taken; whether the firmware matches | replug the ESP32; `aim2 doctor`; check `output.protocol` against the firmware |

Stage states (`Snapshot.stages`) are a separate set of criteria:

| StageStatus | Source | ok condition | detail |
|---|---|---|---|
| `capture` | `grabber.stale_s() < 0.2` | a new frame within 200 ms | capture fps |
| `inference` | `det` non-empty and `now - capture_ts < max_infer_age_s` | results are fresh | inference ms |
| `link` | `link.status()["ok"]` | port open and errors <5 | port name or `mock` |

Notes:

- `DEGRADED` only suppresses aiming/firing, it does not stop capture/inference/link; so the HUD still shows detection boxes, which helps with diagnosis.
- In the first few hundred milliseconds after startup `capture stream interrupted` (no first frame yet) or `waiting for the first inference` is common, not a fault.
- `FAILED` has no code path; alerts should be based on `DEGRADED` + `stages` + counters, not on waiting for `FAILED`.

---

## 4. Logs

### 4.1 Console

`aim2` calls `core.logging.setup(level="INFO", log_dir=None, console=True)` at startup (`cli.main`), so **the CLI currently only writes to the console, not to a file**. The desktop app's `desktop/app.py::main` calls `setup_logging(level="INFO", log_dir=None, console=False)`, which **writes neither a file nor the console**; logs only go to the UI's own message panel (and only configuration/restart hints). Format (`ConsoleFormatter`):

```
HH:MM:SS L src                     msg key=value
```

Common log sources (the `src` field): `dma.engine`, `dma.link` (if present), `dma.cli`.

### 4.2 JSONL file (must be enabled explicitly)

`core/logging.setup` supports `out/logs/aim.jsonl`, rotated at 8 MiB × 3. The CLI does not pass `log_dir`, so to get a file use a wrapper script:

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python - <<'PY'
from dma.core.config import load
from dma.core.logging import setup
from dma.engine.runtime import Engine

setup("INFO", "out/logs")          # returns the log path; file name defaults to aim.jsonl
engine = Engine(load())
try:
    engine.run(duration_s=60)
finally:
    engine.stop()
PY
```

Fields (`JsonFormatter`):

| Field | Meaning |
|---|---|
| `ts` | `record.created` (epoch seconds, float) |
| `t` | `HH:MM:SS` local time |
| `lvl` | `DEBUG/INFO/WARNING/ERROR` |
| `src` | logger name (stage) |
| `msg` | human-readable message |
| `exc` | exception stack (only with `exc_info`) |
| other | structured fields from `extra=`, flattened as-is |

Common events and extras:

| msg | src | extras | Meaning |
|---|---|---|---|
| `engine started` | `dma.engine` | `model`, `control_hz` | startup parameters |
| `health changed` | `dma.engine` | `health`, `detail` | logged only when the state changes (`_set_health`) |
| `frame size` | `dma.engine` | `w`, `h` | first time/resolution change; also clears the latency line |
| `aim toggled` | `dma.engine` | `enabled` | space bar/CLI |
| `fire toggled` | `dma.engine` | `enabled` | `f`/CLI |
| `config hot reload` | `dma.engine` | `changed` | number of diffs from `Engine.apply` |
| `field changed` | `dma.engine` | `field`, `old`, `new` | one entry per field |
| `inference failed` | `dma.engine` | `err` | rate-limited to 1 second (`StageLogger.throttled`) |
| `engine stopped` | `dma.engine` | `shots` | fire count |

Operations grep examples:

```bash
grep -c '"health":"degraded"' out/logs/aim.jsonl
grep '"detail":"inference result stale"' out/logs/aim.jsonl | tail
grep '"msg":"field changed"' out/logs/aim.jsonl | tail -20
```

### 4.3 Session recording (a separate set of files)

`aim2 run --record` / `aim2 record` additionally write `out/telemetry/session_YYYYmmdd_HHMMSS.jsonl.gz` (`SessionRecorder`):

| record `type` | Content |
|---|---|
| `header` | `ts` (epoch), `schema`, the complete `settings`, the `model` description |
| `frame` | `capture_ts`, `frame_id`, `infer_ms`, `cam` samples, `dets` (source box + cls_id + score) |
| `tick` | control `ts`, `cam`, `u` (dx/dy), `tid`, `err`, `mode`, `lat` (loop_ms/confidence/frames) |
| `fire` | `ts`, `fire`, `reason`, `dist` |

View the summary:

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli replay out/telemetry/session_x.jsonl.gz --summary
```

Note: `replay` hardcodes the class name of every detection to `"body"` (`telemetry/replay.py`), so a replay cannot reproduce head-class fusion; it is good for comparing control laws/parameters, not for verifying the head-lock policy.

---

## 5. Metrics

`Metrics` (`core/metrics.py`) has three kinds:

| Kind | Method | Meaning |
|---|---|---|
| Histogram | `observe(name, v)` / `stats_summary()` | fixed window (240 samples by default), outputs `n/mean/p50/p95/p99/min/max/std/last` |
| Counter | `inc(name)` / `counters()` | monotonic accumulation, reset on restart |
| Gauge | `set(name, v)` / `gauges()` | current value |

Main metrics (from the call sites in `Engine`/`OnnxDetector`/`SerialLink`/`FireController`):

| Metric | Kind | Meaning | Healthy reading |
|---|---|---|---|
| `capture.fps` | gauge | capture thread EMA frame rate | about 60 in game; 120 on the desktop |
| `capture.frames` | counter | successful frame reads | keeps growing |
| `capture.read_errors` | counter | failed reads | occasional is acceptable; keep growing → check card/cable |
| `capture.open_errors` | counter | failed opens | occasional during startup is acceptable |
| `capture.reconnects` | counter | reopens after stream loss | 0 is best; growth means an unstable signal |
| `capture.interval_ms` | hist | frame interval | p95 near 1000/fps; a spike means dropped frames |
| `perception.infer_ms` | hist | pure inference of `sess.run` | 4–6 ms for the 320 models (measured) |
| `perception.total_ms` | hist | preprocessing + inference + postprocessing | a few ms more than infer |
| `perception.pre_ms` / `post_ms` | hist | preprocessing/postprocessing | used to locate which stage is slow |
| `perception.dets` | gauge | detections in the latest frame | 0 may mean no target/nothing detected |
| `perception.errors` | counter | inference exceptions | should be 0 |
| `control.hz` | gauge | actual control thread frequency | close to `runtime.control_hz` (240) |
| `aim.err_px` | hist | control error (pixels) | p50/p95 show tracking quality; scene-dependent |
| `aim.kp` | hist | currently scheduled kp | used to confirm the scheduling range |
| `aim.cam_x` / `cam_y` | gauge | accumulated commanded displacement | world-frame state |
| `aim.saturated` | counter | output saturation count | grows with big flicks; continuous growth means `max_step` too small or kp too large |
| `aim.hold_ticks` | counter | ticks held inside the dead band | growing while stationary is normal |
| `tracker.count` | gauge | current number of tracks | 1–5 in multi-target scenes; a surge means misdetection/poor association |
| `transport.frames` | counter | successful serial write frames | keeps growing |
| `transport.write_ms` | hist | duration of a single write | p95 should be far below `write_timeout_s=20 ms` |
| `transport.write_errors` | counter | write exceptions | should be 0 |
| `transport.reconnects` | counter | serial reopen count | 0 is best |
| `transport.connected` | gauge | whether the port is open | 1 |
| `device.rx_frames` | gauge | frames received by the device (STATUS) | keeps growing |
| `device.crc_errors` | gauge | CRC errors on the device side | should be 0; growth → check cable/baud rate |
| `device.queue_drops` | gauge | device queue drops | should be 0; growth means the report rate is too high/the firmware is busy |
| `device.report_hz` | gauge | device report rate (STATUS) | compare against `output.report_hz` |
| `fire.shots` | counter | fire count | matches `shots` |
| `fire.reason.<reason>` | counter | count per fire-decision reason | look at the distribution, not only shots |
| `health.<state>` | counter | health state transitions | frequent transitions mean an unstable link/capture |

Inference-stage baseline (this machine, CoreML, 2560×1600 input frames, `aim2 models bench`; see `docs/MEASUREMENTS.md`):

| Model | p50 | p95 |
|---|---|---|
| YOLOv8 320×320 export | 4.1 ms | 7.9 ms |
| End-to-end 320×320 export | 5.0 ms | 6.3 ms |
| YOLOv5 640×640 export | 7.4 ms | 8.2 ms |
| End-to-end 640×640 export | 16.6 ms | 20.6 ms |

The CLI's `aim2 models bench` currently uses random 1280×720 frames, so numbers differ slightly because of preprocessing/scaling; the table above is for judging "does switching models make it significantly slower", not closed-loop latency.

How to read them: the CLI has no `metrics` subcommand, and `describe()` does not include the full metrics. Use a script when needed:

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python - <<'PY'
import json, time
from dma.core.config import load
from dma.core.metrics import Metrics
from dma.engine.runtime import Engine
from dma.capture.sources import SyntheticSource
from dma.transport.link import MockLink

m = Metrics()
s = load()
e = Engine(s, metrics=m, source=SyntheticSource(s.capture.width, s.capture.height, 60),
           link=MockLink(s.output, m))
e.start()
try:
    for _ in range(300):
        e.step(); time.sleep(1/240)
finally:
    e.stop()
print(json.dumps(m.snapshot(), ensure_ascii=False, indent=2))
PY
```

`Metrics.to_prometheus()` can export a minimal text exposition; there is no HTTP endpoint at the moment, you have to wire it yourself.

---

## 6. Common troubleshooting

### 6.1 Startup/environment

| Symptom | Possible cause | Check | Handling |
|---|---|---|---|
| `ModuleNotFoundError: dma` | `PYTHONPATH=src` not set | `echo $PYTHONPATH` | set it per §1.1 |
| `aim2: command not found` | console script not installed | `which aim2` | use `python -m dma.cli` or the alias |
| `doctor` reports onnxruntime missing | venv lacks dependencies/wrong Python | `which python`, `pip list` | reinstall with `.venv/bin/python` |
| `doctor` reports the model does not exist | `vision.model` path wrong relative to cwd | `aim2 config get vision.model` | run from the repository root; or use an absolute path |
| `doctor` reports "no shootable classes" | model metadata has no `names`, or `target_classes` does not match | `aim2 models inspect`, `config show` | fix `vision.target_classes`; switch model |
| `doctor` reports "teammate/bot classes listed as shootable" | `teammate/ai` wrongly added to `target_classes` | `config show` | remove them from target; keep them in ignore |
| `doctor` reports yolov8 using stretch | `vision.letterbox=off/stretch` | `config get vision.letterbox` | change back to `auto`/`on` |
| Configuration invalid | out of range/enum/near>=far/deadzone_exit<deadzone/odd width or height | the error text of `aim2 config validate` | fix the field; do not hand-edit JSON to bypass validation |
| Worried `config migrate` overwrites | it writes `configs/settings.json` and does not touch `config.json` | `ls -l configs/` | `cp` first if you need the old file kept |

### 6.2 Capture/screen

| Symptom | Possible cause | Check | Handling |
|---|---|---|---|
| No picture / `capture stream interrupted` | camera permission, no HDMI signal, held by OBS, USB power | `capture.read_errors`, `grabber.describe()` | grant permission in System Settings; change cable/port; close OBS; check the capture card LED |
| `waiting for the first inference` forever after the first frame | inference thread crashed/model incompatible | log `inference failed`, `perception.errors` | verify with `aim2 models inspect`; read the exception text |
| `inference result stale` | inference p95 above 120 ms | `perception.infer_ms` | switch to a 320 model/lower ROI/close other CPU-heavy programs |
| Capture only 30 fps | the capture card only supports 30 at 720p/negotiation failed | `fps/width/height` of `grabber.describe()` | lower the resolution/change fourcc; see docs/MEASUREMENTS.md |
| 120 on the desktop, 60 in game | exclusive fullscreen locks HDMI to 60 Hz | `capture.fps` | use borderless fullscreen/uncap the frame rate/disable vsync (docs/MEASUREMENTS.md) |
| Capture resolution does not match the game | the card is scaling, adding 1–2 frames | `frame.shape` vs `capture.width/height` | align with the game's native resolution (docs/MEASUREMENTS.md) |
| `capture.reconnects` keeps growing | momentary signal loss/unstable USB | logs, `read_errors` | change cable/port/add power; check the HDMI cable |

### 6.3 Aiming/control

| Symptom | Possible cause | Check | Handling |
|---|---|---|---|
| Aiming is on but nothing moves | `health != OK`; no target; `aim_enabled` not on | `Snapshot.top_line()`, `fire.reason`, `health` | fix health/targets first; toggle with space |
| Completely reversed/erratically drifting | wrong world-frame scale/sign; wrong gain calibration | §3 gain calibration; `output.gain` | recalibrate; do not flip signs in the controller |
| Back-and-forth overshoot near the target | `kp_far` high + latency estimate too small; `max_step` large | `loop_ms`, `confidence`, `aim.saturated` | lower `kp_far`/`max_step`; wait for latency to converge |
| Flick reaches the target then rebounds | integral windup/feedforward too large | `lead_max`, `integral_limit`, `aim.saturated` | lower `lead_max`/`ki`; confirm anti-windup is active |
| Tracking lag | insufficient feedforward/large latency | `lead_gain`, `loop_ms` | raise `lead_gain` (≤1.0); fix latency first |
| Jitter at rest | dead band too small/output unsmoothed/detection noise | `aim.err_px` p50, `output_ema`, `deadzone` | raise the dead band/lower `output_ema`/raise `kd` |
| Switching back and forth between targets | `switch_margin` too small; track identity unstable | `policy.switches` (no metric, needs logs/UI), `tracker.count` | raise `switch_margin`; raise `tracker.iou_thr` consistency; do not tune `fov` at the same time |
| Pulls toward teammates | model has no teammate class/class configuration wrong | targets from `models inspect` | switch to the sjz model; `ignore_classes` contains teammate; turn aiming off in team modes first |
| Aiming above/below the head | `head_ratio` not calibrated/model changed | §2 calibration | recalibrate; do not use head_ratio when there is a head class |
| Feel changes abruptly after hot reload | `Engine.apply` rebuilds the controller because of the `_kind` check, clearing integral/EMA | log `config hot reload` | restart after changing parameters; or accept one state reset |
| Small displacement remains after disabling aiming | the flush of `set_aim_enabled(False)` submits the residue again (see ARCHITECTURE §8) | observe `planner.pending` | `config set output.dry_run true` before disabling; or fix the flush in code |

### 6.4 Fire

The `reason` of `FireController.update` is the first-hand information for diagnosing firing problems:

| reason | Meaning | Handling |
|---|---|---|
| `off` | `fire.mode="off"` | change to `single`/`auto` |
| `no-target` | no usable target in the current frame | check detection/target selection/FOV/masks |
| `coasting` | target lost, extrapolating | normal protection; if frequent check tracking/occlusion |
| `uncertain` | `track.sigma > fire.max_sigma` | target just appeared/switched; wait for it to settle; or raise `max_sigma` (carefully) |
| `off-target` | impact point not inside the inset box and residual > `fire.radius` | aim point/latency/fire radius problem; look at `aim.err_px` first |
| `settling` | on the target but dwell < `dwell_s` | normal; happens with target jitter/just after switching |
| `cooldown` | less than `cooldown_s` since the last shot | normal; repeat cadence |
| `already-fired` | this lock already fired in `single` mode | normal; switch target or use `auto` |
| `fire` | firing | — |

| Symptom | Check | Handling |
|---|---|---|
| Not a single shot, reason=`off` | `Snapshot.fire_enabled`, `fire.mode` | CLI `--fire` or configure `fire.mode` |
| `off-target` all the time | impact point/box/latency | turn fire off first, look at the aim error; recalibrate head point/latency |
| `uncertain` all the time | `tracker.count`, whether the target just appeared | wait 2–3 frames; check `tracker.accel_noise/meas_noise` |
| `fire` but the game does not react | serial/firmware/key mask | `transport.frames`, `device.rx_frames`, key frames |
| Repeat too fast/too slow | `cooldown_s`/`dwell_s`/`tap_s` | tune per [TUNING](./TUNING.md); one at a time |

### 6.5 Link/serial

| Symptom | Possible cause | Check | Handling |
|---|---|---|---|
| `no serial device found` | not plugged in/driver/taken | `ls /dev/cu.usbmodem*`, `aim2 doctor` | change cable/install driver/close the program holding it |
| Multiple candidate ports | several USB serial devices | the detail of `doctor` | set `output.port` explicitly |
| Serial opens but the game does not move | protocol mismatch/firmware not running HID | `output.protocol`, `device.rx_frames` | align the protocol; reflash the firmware |
| `transport.write_errors` growing | cable/baud rate/power | `link.status()["detail"]` | change cable/lower baud rate/add power |
| `transport.reconnects` growing | the port keeps dropping | logs | change USB port; avoid hubs |
| `device.crc_errors` growing | cable quality/baud rate deviation | STATUS | use a shorter cable; confirm 921600 |
| `device.queue_drops` growing | report rate too high/firmware busy | `output.report_hz` | lower to 250/500; check the firmware queue |
| Mouse jitter/stepping | low report rate/per-report clipping | clipped/residual of `planner.report()` | raise `report_hz`; confirm `max_per_report=127` |

### 6.6 Latency/simulation/replay

| Symptom | Possible cause | Check | Handling |
|---|---|---|---|
| `confidence=0` | target/crosshair not moving; no detections on synthetic frames | `samples`, `frames` | use a real moving target; see [CALIBRATION](./CALIBRATION.md#4-latency-calibration) |
| `frames` jumping | discontinuous detections/dropped frames | `capture.interval_ms`, `tracker.count` | fix capture/tracking first |
| `loop_ms` does not match `frames/fps` | unstable frame rate/`frame_dt` median distorted | same as above | estimate again once the frame rate is stable |
| `aim2 bench` fails | the predictive law is worse than v1 in some scenarios | the printed `failures` | see [TUNING §5](./TUNING.md#5-simulation-regression-verification); do not ignore the gate |
| `aim2 replay` results look wrong | replay hardcodes the class name `body` and a fixed 1280×720 centre | session resolution | only for control-law comparison; do not use it to verify head locking |
| `autotune` pushes parameters to the boundary | search-space upper bound/objective function | history/rows of `--json` | re-check with `bench`/`replay`; do not ship directly |

---

### 6.7 Desktop app

| Symptom | Possible cause | Check | Handling |
|---|---|---|---|
| `ModuleNotFoundError: PySide6` | venv lacks the desktop dependency | `python -c "import PySide6"` | `pip install PySide6` (`requirements.txt` does not include it) |
| Window opens but no picture | capture card/model not ready; `snap.frame` empty | the HUD's `cap`, `inf`; `health` | verify the UI with `--sim` first; on live hardware check camera permission |
| Changing parameters reports "configuration rejected" | `set_fields` validation failed (out of range/enum/near>=far) | the log panel text | fix the value per the hint; do not bypass the `FIELDS` range |
| Feel changes abruptly after a parameter change | any diff triggers a controller rebuild (the `_kind` check in `Engine.apply`) | whether a "restart needed" hint appeared | restart after big changes; accept one integral reset for small ones |
| Model switch has no effect | the model dropdown only affects the next start | top-bar selection | save the configuration and restart `aim2`/the desktop app |
| GUI does not start | no display session | terminal output | use `python -m dma.cli gui` directly, or open a terminal inside the desktop |
| The desktop app cannot stop the engine | `closeEvent` waits 1 second | whether the process exists | `pkill -f dma.desktop.app`; then `pkill -f "dma.cli run"` |

## 7. Rolling back to v1

### 7.1 First decide the rollback level

| Level | How | Impact |
|---|---|---|
| Control law only | `aim.controller=legacy` | v2 Engine/world frame/protocol stay; uses `dma.aim.AimController` (v1 PD+I) for A/B; not a full v1 |
| Host program | run `src/dma/main.py` / `run_mac.sh gui` | back to v1 single-threaded capture+inference+serial; `config.json` takes effect |
| Firmware | flash the v1 firmware back | the v1 host sends `0xAA`; v2 firmware only understands `0xA5`, they must be switched as a pair |
| Code | `git checkout <v1 commit or tag>`, or restore single files from your own backup | never overwrite the current `src/` wholesale |
| Desktop app | close `desktop/app.py` and go back to v1 `qt_gui.py` | the two UIs have different parameter models: v2 edits `configs/settings.json`, v1 edits `config.json` |

### 7.2 Preserve the scene

```bash
cd ~/dma
BK="out/rollback_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BK"
cp configs/settings.json "$BK/" 2>/dev/null || true
cp config.json "$BK/" 2>/dev/null || true
cp -r out/logs out/telemetry "$BK/" 2>/dev/null || true
echo "scene saved to $BK"
```

v2's configuration is in `configs/settings.json`; v1's is in `config.json`. They are independent, and v2's `save` does not modify `config.json`, so **do not delete `config.json`**.

### 7.3 Rollback steps (host)

1. Stop v2: Ctrl-C (`aim2 run` goes through `Engine.stop()`), or `pkill -f "dma.cli run"`; for the desktop app also `pkill -f dma.desktop.app`.
2. Confirm `config.json` still holds v1 values: `python -c "import json;print(json.load(open('config.json'))['model_path'])"`.
3. Run v1:

```bash
cd ~/dma
./run_mac.sh check          # needs the serial port, otherwise exit 2
./run_mac.sh gui            # needs sudo/launchctl; or:
PYTHONPATH=src .venv/bin/python src/dma/main.py
```

4. v1 windowless mode:

```bash
PYTHONPATH=src .venv/bin/python src/dma/main_cli.py
```

5. Verify: the GUI has a preview, the HUD shows "measured latency", the space/F hotkeys work, and fire reasons are correct.
6. If v2 ever upgraded the firmware to the v2 protocol: the v1 host sends `0xAA`, which v2 firmware does not understand; either flash the v1 firmware back, or do not use the v1 host for now.

### 7.4 About legacy code

The public repository ships no v1 backup archive. To compare against the old
implementation:

```bash
git log --oneline --all | tail                          # find the v1-era commit
git show <commit>:src/dma/aim.py > /tmp/v1_aim.py      # restore a single file
```

`control/legacy_pid.py` keeps the v1 control law itself for reference.

### 7.5 Cleanup after rollback

- Do not let v2 and v1 open the capture card/serial port at the same time: one device can only belong to one process.
- Keep v2's `out/telemetry` and `out/logs`, do not overwrite them.
- If you later go back to v2: run `aim2 config validate`; `configs/settings.json` may still hold v2 tuning values that are out of sync with `config.json`, so run `aim2 config diff --other config.json` first (note the latter is v1 format and `load` migrates it).

---

## 8. Operations checklist

### 8.1 Before power-on

- [ ] `aim2 doctor --dry-run` passes; use a non-dry-run for the serial check.
- [ ] targets/heads from `aim2 models inspect` are correct; no teammate/ai.
- [ ] when using the desktop app, `python -c "import PySide6"` passes.
- [ ] `python -m pytest -q` fully passes (currently 79 cases).
- [ ] the game is borderless fullscreen, uncapped, vsync off; capture resolution = game native.
- [ ] in-game sensitivity/DPI unchanged (if changed, recalibrate gain/latency).
- [ ] disk space available (logs/recordings; `out/` already holds large files).
- [ ] only this process uses the capture card/serial port.
- [ ] the firmware was flashed with the USB settings from `firmware/esp32_hid/README.md` §2; `output.protocol` matches.

### 8.2 While running

- [ ] `health=OK`; all three `stages` ok.
- [ ] `capture.fps` as expected (60 in game).
- [ ] `perception.infer_ms` p95 in the same order as the baseline.
- [ ] `latency.confidence ≥ 0.35`, `frames` stable.
- [ ] `aim.saturated` does not keep growing fast.
- [ ] `transport.write_errors=0`, `device.crc_errors=0`, `queue_drops=0`.
- [ ] `fire.reason` distribution reasonable (mostly `fire/settling`).

### 8.3 After finishing

- [ ] keep `out/logs/aim.jsonl` (if enabled explicitly per §4.2) and the sessions (if `--record` was on).
- [ ] note the latency/link/shots from `engine.describe()`.
- [ ] if parameters changed: archive `aim2 config diff --other backup.json`.
- [ ] do not commit `.venv/`, `out/` or large model files to version control (the repository currently has no git, still keep the directory tidy).

---

This document corresponds to schema_version 2 and the src/dma layout; please update it when the code changes.
