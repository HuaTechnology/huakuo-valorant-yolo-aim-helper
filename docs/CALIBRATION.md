<div align="right">

[简体中文](zh-CN/CALIBRATION.md) | **English**

</div>

# Calibration Handbook (CALIBRATION)

> ⚠️ **For learning, research and engineering study only.**
> Do not use this project in any competitive or ranked setting. Automated aiming
> violates the terms of service of most online games and may lead to permanent bans.
> You are solely responsible for how you use it.

> Code baseline: `src/dma`, schema 2. Defaults come from `src/dma/core/config.py`; measured conclusions are in `docs/MEASUREMENTS.md`.
> The calibration order is fixed: **model/classes → head aim point → output sensitivity/gain → closed-loop latency → verification checklist**. Do not start the next item until the previous one passes.

---

## 0. Prerequisites

| Condition | Requirement | Check |
|---|---|---|
| Game display | Borderless fullscreen, uncapped frame rate, vertical sync off | docs/MEASUREMENTS.md: exclusive fullscreen locks HDMI at 60 Hz |
| Capture resolution | Same as the game's native resolution (default 1280×720) | `CvSource.describe()`, `capture.width/height`; docs/MEASUREMENTS.md: misalignment adds 1–2 frames of scaling latency |
| In-game sensitivity | Fixed during calibration; mouse acceleration / smoothing beyond raw input off | must not be changed after calibration |
| Model | `aim2 models inspect` passes | `vision.model`, `vision.target_classes/ignore_classes` |
| Output link | Serial port opens, firmware protocol matches | `aim2 doctor`, `output.protocol` |
| Safe environment | Practice range / offline room, no teammates around the crosshair | `vision.ignore_classes` already contains `teammate/ai/crosshair`, but the model can still misdetect |

---

## 1. Model and class calibration

### 1.1 Inspection commands

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python -m dma.cli models list
PYTHONPATH=src .venv/bin/python -m dma.cli models inspect YOLOv8 320×320 export --json
PYTHONPATH=src .venv/bin/python -m dma.cli doctor --dry-run
```

`tuning/calibration.py::model_report` reports:

| Field | Meaning | Pass criterion |
|---|---|---|
| `format` | `e2e` / `yolov5` / `yolov8` | matches the model's actual output; `yolov8` must use `letterbox` |
| `targets` | class names hit by `vision.target_classes` | at least one; must not contain `teammate/ai` |
| `heads` | classes recognised as head | with `head` you can lock the head directly; without it you must calibrate `head_ratio` |
| `errors` | blocking items | must be empty |
| `warnings` | non-blocking hints | confirm them one by one (commonly "the model has no head class") |

Measured metadata of the three models in this repository:

| Model | Input | Output | Decoding | Classes | head |
|---|---|---|---|---|---|
| YOLOv8 320×320 export (`vision.model` default) | 320×320 | `[1,9,2100]` | `yolov8` | `body, head, teammate, ai, crosshair` | yes |
| End-to-end 320×320 export (example) | 320×320 | `[1,300,6]` | `e2e` | `st, qiu, dj, fyb, sg1..sg4` (class table of the example model) | no |
| YOLOv5 640×640 export | 640×640 | `[1,25200,7]` | `yolov5` | `head, body` | yes |

### 1.2 Class whitelist/blacklist

- `vision.target_classes` defaults to `("body","head","st","person","enemy","target")`.
- `vision.ignore_classes` defaults to `("teammate","ai","crosshair")`.
- `ClassMap.is_targetable` requires "in target and not in ignore"; `OnnxDetector.detect` filters before NMS, so **ignored classes never reach the tracker**.
- When the model only has a body class (such as `st`), the body is shootable; without a `head` class, head locking can only rely on `head_ratio`.
- When the model has `teammate`, `ignore_classes` still catches it even if `target_classes` is wrong; but `model_report` still reports "a teammate is listed as target" as an error, asking you to fix the configuration instead of relying on the fallback.

### 1.3 Live confirmation

1. `aim2 run --dry-run --duration 30 --no-hotkeys` (no aiming, look at detection first).
2. In the UI/log confirm: boxes hug the body/head, class names are correct, distant small targets are also detected.
3. Adjust `vision.conf` (default 0.35) and `vision.iou` (default 0.45): too many misses → lower conf; duplicated boxes → lower iou; change only one at a time.
4. Use `vision.roi="center"/"target"` only when the target is always near the screen centre; ROI changes `Transform`, but the inverse transform shares the same object, so no extra calibration is needed.

---

## 2. Head aim point

### 2.1 With a head class

`TargetPolicy._fuse` and `_aim_point` (`control/policy.py`):

- each head box tries to pair with a body box; conditions:
  - `head.area < body.area * 1.4` (a head cannot be larger than a body);
  - `body_box.contains(head.cx, head.cy, margin=-0.02)` (head centre inside the body box, 2% outward margin allowed).
- Pairing succeeds: the target takes the body's identity (tid/velocity from body), the aim point uses the **head box centre**, `is_head=True`.
- Pairing fails: the head becomes an independent target, the aim point is still the head centre.
- Target scoring: `score = dist - head_bonus` (default 28 px); with `prefer_head=True`, head targets are preferred.

So with a head class you do **not** need `policy.head_ratio`, and you should not use it for fine-tuning; changing `head_bonus` only affects "who is selected", not "where to aim".

### 2.2 Body class only

`_aim_point` returns for non-head targets:

```python
(box.cx, box.y1 + box.h * policy.head_ratio)
```

- default `policy.head_ratio = 0.11`.
- the current migrated `config.json` value is `0.1386` (v1 `HEAD_RATIO`, docs/MEASUREMENTS.md §4 records ~0.139).
- For body-only models head locking depends on this ratio, and it **must be calibrated on site with the red cross**; typical range 0.08–0.15 (see `docs/MEASUREMENTS.md`).

Why a fixed value does not work:

- different models define the "body box" differently (whether it includes the head/hair/shoulders);
- camera height, FOV and target distance change where the top of the head sits inside the box;
- a resolution change alters the box height in pixels, but the ratio should not change in theory; changing model/game requires recalibration.

### 2.3 Automatic estimation (recording with head + body)

```bash
# 1) record a session where head/body appear together (must use a model with a head class)
PYTHONPATH=src .venv/bin/python -m dma.cli record --duration 60 --dry-run --no-hotkeys

# 2) estimate the median ratio from head/body pairs in the recording
PYTHONPATH=src .venv/bin/python -m dma.cli calibrate head --path out/telemetry/session_YYYYmmdd_HHMMSS.jsonl.gz
```

`tuning/calibration.py::head_ratio_from_session`:

- only counts pairs where the head centre falls inside the body box;
- needs ≥20 samples, otherwise returns `None`;
- outputs the `head_ratio` suggestion and the median sample count;
- the command prints a hint for `aim2 config set policy.head_ratio X`.

Note: `SessionRecorder.frame` records detections **after class filtering**; if `head` is not in `vision.target_classes`, the recording has no head and cannot be estimated. For this calibration put `head` into target first, then adjust as needed afterwards.

### 2.4 Manual calibration (model without a head class)

1. Open the v1 PySide6 GUI (`qt_gui.py`, `./run_mac.sh gui` or `aim`); it provides preview plus red-cross overlay (docs/MEASUREMENTS.md §6).
2. Stand in the practice range and let an enemy (or dummy) appear at medium distance, then keep still.
3. Drag `HEAD_RATIO` until the red cross lands on the head centre; record the value.
4. Verify once at close range and once at long range; if the same ratio is off at both, the model's box definition changes with distance — take the middle value and accept the error.
5. Write it into v2:

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli config set policy.head_ratio 0.12
```

6. Re-check with `aim2 config get policy.head_ratio`; restart `aim2 run` (`policy` supports hot reload, but a restart is cleanest).

### 2.5 Head calibration acceptance

| Check | Pass criterion |
|---|---|
| Stationary target | the crosshair settles on the head centre and does not drift up and down |
| Left-right strafe | the aim point stays on the head, not on the shoulder/chest |
| Long distance | still on the head when the box shrinks; the error shrinks linearly with distance |
| Fire | `fire.reason` reaches `fire`, not a permanent `off-target` (the inset box `inset` defaults to 0.06, so aiming at the neck may also be accepted, see §5) |
| Multiple targets | two tids do not fight for the lock because of duplicated head/body boxes |

---

## 3. Sensitivity and gain calibration

### 3.1 Why it is mandatory

The world frame assumes:

```
1 px of control output  ==  1 px of screen translation
```

`MotionPlanner.add` (`transport/planner.py`) multiplies the controller output `dx` by `output.gain` and then sends HID counts. The ratio at which the game converts counts into screen pixels is determined by "in-game sensitivity × DPI". If that ratio is not 1:

- the scale of the world frame `world = source + cam` is wrong;
- the target velocity seen by the tracker is amplified/reduced;
- the feedforward `v * apply_s` and `lead` are both wrong;
- the symptom is "oscillation at high gain" or "it never keeps up no matter how you tune", but the root cause is not the control law.

### 3.2 Definition

Let:

- `N`: the HID counts we send;
- `P`: the number of pixels a stationary feature moves on screen (source pixels);
- `k = N / P`: counts needed per pixel (counts-per-pixel).

`MotionPlanner.add` sends `gain * dx` counts. For that to equal `k * dx`, we need:

```
output.gain = k = N / P
```

The default is `output.gain=1.0` (dataclass); the current `config.json`'s `SINK_GAIN=3.0` migrates to 3.0. The v1 experience is ">1.5 oscillates easily" (docs/MEASUREMENTS.md §6), but that was v1 without a world frame/latency compensation; v2 still recommends starting from the calibrated value and not cranking it up by feel.

### 3.3 Measurement steps

1. Enter the practice range, face a wall with clear texture/right angles, and put the crosshair on a fixed feature.
2. Disable aiming (so the controller does not insert displacement) and keep in-game sensitivity unchanged.
3. Send a test displacement with a known number of counts. The one-off script below (not committed to the repository) works:

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python - <<'PY'
import time, serial
from dma.transport import protocol as P

PORT = "/dev/cu.usbmodemREPLACE"   # port shown by aim2 doctor
N = 200                             # counts to send
with serial.Serial(PORT, 921600, timeout=0.01, write_timeout=0.05) as s:
    s.write(P.encode_move(N, 0, 1)); time.sleep(0.2)
    s.write(P.encode_move(-N, 0, 2))  # back to the origin, so you can repeat
    time.sleep(0.2)
print("sent", N)
PY
```

> If the firmware still speaks the v1 protocol, replace `P.encode_move` with `P.legacy_move`; `output.protocol` only affects what the Engine sends, not this script.

4. From the captured frames (or the `aim2 run --dry-run` preview), measure the feature's displacement in x, `P` pixels. Repeat 3 times and take the average.
5. Compute `k = N / P` and write it:

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli config set output.gain 1.00   # example: k=1.0
```

6. Re-measure: send `N=200` again, the displacement should be about 200 px (±5%). If it differs a lot, check:
   - whether in-game mouse acceleration/smoothing is off;
   - whether DPI was changed by driver software;
   - whether the capture resolution matches the game's native one (if not, `P` is not in game pixels);
   - whether you are in a game menu/map screen (sensitivity may differ).

### 3.4 Acceptance and common mistakes

| Symptom | Meaning | Action |
|---|---|---|
| 200 counts move the screen only 100 px | `k=2`, the current gain is too small | `output.gain=2.0` |
| 200 counts move the screen 400 px | `k=0.5`, the current gain is too large | `output.gain=0.5` |
| The crosshair moves but the screen does not | serial/firmware/HID is not taking effect | `aim2 doctor`, `device.rx_frames` |
| Movement goes the opposite direction | coordinate sign/game settings | check the protocol dx sign and the game Y axis; do not flip signs in the controller |
| Still oscillating after calibration | latency estimation/tracking problem | do §4 first, then lower `kp_far` |

After calibration, **do not change in-game sensitivity/DPI again**; if you do, you must recalibrate `output.gain` and rerun latency calibration (the world-frame scale changed).

---

## 4. Latency calibration

### 4.1 Goal

Get `LatencyEstimator` to converge to the real `loop_s` of the current link, and confirm `confidence` is high enough, before touching gain.

### 4.2 Procedure (live hardware, recommended)

1. Confirm `aim2 doctor --dry-run` passes and the model can detect targets.
2. Enter the practice range and make the target (dummy/enemy) strafe continuously; keep aiming on too, so the controller produces displacement.
3. Record for 60–90 seconds:

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli run --record --duration 90 --aim --no-hotkeys --quiet
```

4. When it ends the CLI prints `describe()`, which includes:

```json
"latency": {"loop_ms": ..., "frames": ..., "confidence": ..., "measure_ms": ..., "apply_ms": ..., "lead_ms": ...}
```

5. Judgement (`tuning/calibration.py::latency_from_estimator`):

| Condition | Verdict |
|---|---|
| `confidence >= 0.35` and `samples >= 30` | trustworthy |
| `0.2 <= confidence < 0.35` | marginal, keep collecting; do not raise gain based on it |
| `confidence < 0.2` | untrustworthy; `loop_s` is still the prior/last value |

6. Cross-check: `frames × 1000 / capture.fps ≈ loop_ms`. At 60 fps, `frames=1` → about 16.7 ms; `frames=2` → about 33.3 ms.
7. Optional: write the trustworthy value into the prior to shorten the next convergence:

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli config set aim.latency_prior_s 0.017
```

But do **not** disable the estimator (there is no switch; raising `latency_min_conf` only makes it update less often). The prior is just a starting point; it keeps updating at runtime.

### 4.3 Procedure (video/offline)

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python - <<'PY'
from dma.core.config import load
from dma.capture.sources import VideoSource
from dma.engine.runtime import Engine
from dma.transport.link import MockLink
from dma.tuning.calibration import wait_for_latency

s = load()
eng = Engine(s, source=VideoSource("videos/REPLACE.mp4"), link=MockLink(s.output))
eng.start()
try:
    print(wait_for_latency(eng, seconds=30))
finally:
    eng.stop()
PY
```

The video must contain targets the model can detect, and the target must be moving. A static image cannot estimate latency (see §4.5).

### 4.4 About `aim2 calibrate latency`

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli calibrate latency --seconds 8
```

Internally it uses `SyntheticSource` (`cli.py::cmd_calibrate`); synthetic squares are usually outside the model's training distribution, so in practice you get `samples=0 / confidence=0 / loop_ms=33.0`. Treat it as a smoke test that "Engine + estimator runs", not as a calibration result.

### 4.5 Typical reasons for low confidence

| Cause | Symptom | Action |
|---|---|---|
| Target and crosshair both still | error is flat, the cost of all lags is close | make the target strafe, and track it yourself too |
| Target frequently lost | `on_detection` samples are discontinuous, `frames` jumps | fix tracking/target selection first (`policy`/`tracker`), then do latency |
| Large detection jitter | residual dominated by noise | raise `tracker.meas_noise` or lower `vision.conf` false detections; confirm the model is stable |
| Unstable capture frame rate | `frame_dt` median untrustworthy | check `capture.interval_ms` p95, game frame rate, capture card |
| Wrong world-frame scale | your own displacement is wrongly amplified/reduced | do the §3 gain calibration first |
| Latency beyond `max_lag` | the estimator only goes up to 10 frames | at 60 fps 10 frames = 167 ms; a normal link should not exceed it; check capture card/display |

---

## 5. Fire gate and its relation to head calibration

`FireController.update` (`control/fire.py`) currently decides:

```python
inner = preview.box.shrink(min(0.45, max(0.0, cfg.inset)))
inside = inner.contains(*preview.impact_xy)
near   = preview.residual <= cfg.radius
if not (inside or near):  # either one condition is enough
    ...
```

- `fire.inset` defaults to 0.06: the box shrinks by 6%, more conservative the closer to the edge.
- `fire.radius` defaults to 12 px: even if the impact point is not inside the inset box, as long as the residual is ≤12 px fire is allowed.
- These two are an **or**, not an **and**. If you aim at the neck (`head_ratio` too large), the impact point may be outside the upper edge of the inset box but inside `radius`, and fire still happens; if neither holds, `fire.reason` is `off-target`.
- `fire.dwell_s=0.04`, `cooldown_s=0.12`, `tap_s=0.03` decide "how long it must stay stable / how fast it repeats / how long the key is held"; keep the defaults while calibrating the head, and do not use the fire gate to compensate for aim-point error.
- `fire.max_sigma=80 px`: no fire once the Kalman position uncertainty exceeds it; `uncertain` right after a target appears/switches is normal.

---

## 6. Verification checklist

Run in order; keep a record for every item (command output/screenshot/session summary).

| # | Check | Command/observation | Pass criterion |
|---|---|---|---|
| 1 | Config valid | `aim2 config validate` | prints `config valid`; no unhandled warnings |
| 2 | Model loadable | `aim2 doctor --dry-run` | `model ok=true`, `classes` has a target |
| 3 | Classes correct | `aim2 models inspect ... --json` | `targets` contains no teammate/ai; `heads` as expected |
| 4 | Serial port opens | `aim2 doctor` (non dry-run) | `serial ok=true`, correct port |
| 5 | Capture has frames | `aim2 run --duration 10 --dry-run` | `health=ok`, `capture` 30–60+ fps (60 in game) |
| 6 | Inference time | same as above / `perception.infer_ms` | p95 in the same order as docs/MEASUREMENTS.md 4–6 ms (320 model) |
| 7 | Gain calibrated | §3 script | send 200 counts, the screen moves 200±10 px |
| 8 | World-frame tracking | `aim2 run --dry-run` + UI | while you strafe the target box does not drift in the world frame; `tracker.count` stable |
| 9 | Head point | §2 | on the head at rest/strafe/near and far |
| 10 | Latency trustworthy | latency from `aim2 run --record` | `confidence>=0.35`, `samples>=30`, `frames` stable |
| 11 | No oscillation | 30 seconds of live tracking | no periodic back-and-forth overshoot; `aim.saturated` does not keep growing |
| 12 | Fire reasonable | `fire.reason` distribution | mostly `fire`/`settling`; `uncertain` does not dominate for long |
| 13 | Link healthy | `link.status()` / `device.*` | `errors=0`, `reconnects=0`, `crc_errors=0`, `queue_drops=0` |
| 14 | Logs on disk | `out/logs/aim.jsonl` | startup/health/parameter events present; no repeated errors |
| 15 | Replay reproducible | `aim2 replay SESSION --compare` | two controllers on the same session produce comparable output lines |

---

## 7. When recalibration is mandatory

| Change | Head point | Gain | Latency |
|---|---|---|---|
| Change model | yes (head class/box definition changed) | no | maybe (inference time changes) |
| Change game resolution/capture card | maybe (box scale) | yes (pixel scale) | yes (capture card latency) |
| Change in-game sensitivity/DPI | no | **yes** | yes (world-frame scale changes) |
| Change firmware/HID report rate | no | maybe | yes (apply latency) |
| Change display mode (60↔120 Hz) | no | no | yes (frame age/input tick) |
| Change machine/USB port | no | no | yes (serial/driver) |

Back up the current configuration before every recalibration:

```bash
cp configs/settings.json configs/settings.$(date +%Y%m%d_%H%M%S).json 2>/dev/null || true
PYTHONPATH=src .venv/bin/python -m dma.cli config show > out/config_before_$(date +%Y%m%d_%H%M%S).txt
```

(`configs/settings.json` not existing means you are still on the migrated v1 `config.json`; run `aim2 config migrate` to generate a baseline first, then change.)

---

This document corresponds to schema_version 2 and the src/dma layout; please update it when the code changes.
