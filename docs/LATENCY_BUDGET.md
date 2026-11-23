<div align="right">

[简体中文](zh-CN/LATENCY_BUDGET.md) | **English**

</div>

# Latency budget (LATENCY_BUDGET)

> ⚠️ **For learning, research and engineering study only.**
> Do not use this project in any competitive or ranked setting. Automated aiming
> violates the terms of service of most online games and may lead to permanent bans.
> You are solely responsible for how you use it.

> Code baseline: `src/dma`, schema 2. Defaults come from `src/dma/core/config.py`.
> "Measured" means the number comes from `docs/MEASUREMENTS.md` or a runtime field; "inferred" means it is derived from the frame rate, the config or a code path but was not measured separately on the bench; "default" means a configuration default, not a measurement.

---

## 1. Loop definition

The plant (`control/controller.py` module docstring):

```
e(t) = p(t - L_measure) - c(t - L_apply)
```

- `p`: target position (screen pixels, crosshair at the origin).
- `c`: camera position (the displacement we have issued so far, in screen pixels).
- `L_measure`: latency from "the frame being really produced" to "the controller holding the detection result".
- `L_apply`: latency from "the controller issuing the command" to "the picture/crosshair actually moving".
- `L = L_measure + L_apply`: total closed-loop latency, the quantity identified online.

`AimCfg.latency_split` (default `0.5`) splits the total latency in half:

| Field | Meaning | Default | Use in code |
|---|---|---|---|
| `latency_split` | measurement-side share | 0.5 | `measure_offset = split * loop_s`; `age_s += split*loop_s`; `apply_s = max(dt, (1-split)*loop_s)` |
| `latency_prior_s` | prior used before convergence | 0.033 s | `LatencyEstimator(prior_s=...)` |
| `latency_window` | identification window | 45 detection samples | `deque(maxlen=window)` |
| `latency_min_conf` | update threshold | 0.5 | `loop_s` is smoothed only when `conf >= min_conf` |

The relation between the three time quantities in `Engine.step`:

```
loop_s      = latency.loop_s
det_age     = now - det.capture_ts
age_s       = det_age + split*loop_s          # extrapolate the target to "now"
apply_s     = max(dt, (1-split)*loop_s)       # command-application window
lead_s      = loop_s + det_age                # total look-ahead used by the fire preview
```

How the three relate:

```
age_s + apply_s = det_age + loop_s = lead_s        (when apply_s does not hit the max(dt, ...) floor)
```

At very low control rates (`dt > (1-split)*loop_s`) `apply_s` falls back to `dt` and the right-hand side is slightly larger; this never happens at a normal 240 Hz.

---

## 2. End-to-end breakdown

### 2.1 Frame → detection

| # | Stage | Code location | Typical magnitude | Basis | How it is observed |
|---|---|---|---|---|---|
| 1 | game rendering + display scan-out | game/HDMI, outside this repo | 0–1 display frames; 60 Hz in game → 0–16.7 ms | **inferred**: docs/MEASUREMENTS.md (measured) "the game locks HDMI to 60 Hz"; the frame interval is observed via `capture.fps` | `FrameGrabber.fps`, `capture.interval_ms` |
| 2 | HDMI → capture-card signal + in-card scaling/encoding | capture-card firmware | 1–5 frames (16.7–83 ms at 60 Hz) | **measured/empirical**: docs/MEASUREMENTS.md §7 "the capture card has 1~5 frames of inherent signal latency; aligning the resolution saves 1~2 frames; MJPG adds encode/decode latency over YUYV" | not readable from the host alone; A/B by swapping card or resolution |
| 3 | driver buffering + `cv2.read` | `capture/sources.py::CvSource.read`, `capture/grabber.py::_run` | 0–1 frame with `buffer_size=1` | **default + inferred**: `capture.buffer_size=1`; the capture thread keeps only the newest frame | `grabber.stale_s()`, `capture.read_errors` |
| 4 | frame age (detection frame to control tick) | `det_age` in `Engine.step` | 0–1 capture intervals; ~8.3 ms average at 60 Hz, ~16.7 ms worst case | **measured field**: `snapshot.latency["frame_age_ms"]` | HUD/`latency.frame_age_ms` |
| 5 | preprocessing (crop/letterbox/normalisation) | `perception/detector.py::_preprocess` | no separate baseline given | **measured field**: `perception.pre_ms`; no bench nominal value | metric p50/p95 |
| 6 | ONNX inference | `OnnxDetector.detect` → `sess.run` | 320 models 4.1–5.0 ms p50; 640 V5 7.4 ms; 640 e2e 16.6 ms | **measured**: CoreML on this machine, 2560×1600 input frames, `aim2 models bench` (`docs/MEASUREMENTS.md`) | `perception.infer_ms`, `aim2 models bench` |
| 7 | decoding + NMS + class filtering | `decoders.*`, `detector.detect` | no separate baseline given | **measured field**: `perception.post_ms`; pure numpy, grows with the number of detections | metric p50/p95 |
| 8 | tracking + target selection + control law | `perception/tracker.py`, `control/policy.py`, `control/controller.py` | no more than one control period (4.17 ms by default) | **inferred**: the control thread runs at a fixed 240 Hz and executes synchronously inside `Engine.step`; nothing is timed separately | `control.hz`, `Snapshot.top_line` |

### 2.2 Output → frame

| # | Stage | Code location | Typical magnitude | Basis | How it is observed |
|---|---|---|---|---|---|
| 9 | delta accumulation + sub-frame splitting | `transport/planner.py::add/take` | `report_hz=500` → 2 ms per report | **default**: `output.report_hz=500` | reports/clipped/residual from `planner.report()` |
| 10 | serial write | `transport/serial_link.py::_write` | 9-byte frame (4 header + 4 payload + CRC) @921600 bps; ~0.098 ms/frame in theory; 9400 frames/s measured link capacity | **measured**: docs/MEASUREMENTS.md "9400 frames/s measured over serial"; v2 frames are shorter | `transport.write_ms` p95, `transport.write_errors` |
| 11 | ESP32 parsing + HID report queueing | firmware | not measured | **inferred**: depends on the firmware queue; drops are observable via `STATUS.queue_drops` | `device.queue_drops`, `device.report_hz` |
| 12 | USB HID polling + game input tick | Windows/game | 0–1 game frames; 60 fps → 0–16.7 ms | **inferred**: the game applies HID on its own input tick; the 60 Hz frame lock in docs/MEASUREMENTS.md has the same origin | cannot be read directly; only estimated in the closed loop |
| 13 | total closed-loop latency `L` | `LatencyEstimator` | measured D=1 frame (v1 HUD, 60 Hz ≈ 16.7 ms); v2 prior 33 ms | **measured**: docs/MEASUREMENTS.md "measured control latency D=1 frame"; 33 ms is the v2 prior default | `snapshot.latency["loop_ms"/"frames"/"confidence"]` |

### 2.3 Why the numbers in the table cannot simply be added

- Capture, inference and control form a **pipeline**: a frame's 4–8 ms of inference (320/V5) overlaps with the next frame's capture; `frame_age_ms` already includes "this frame waiting for the control thread".
- The "capture card 1–5 frames", "game input 0–1 frame" and "display 0–1 frame" entries above are **ranges**, and the worst cases do not happen at the same time.
- Engineering therefore trusts only two numbers:
  1. **single-stage observables** (`infer_ms`, `write_ms`, `frame_age_ms`, `capture.fps`);
  2. **the closed-loop estimate** `loop_ms` (`LatencyEstimator`), which estimates display, capture card, HID and game input — the stages the host cannot see — together.
- Do not use "5 ms inference + 16.7 ms capture + 0.2 ms serial = 22 ms" to override `loop_ms`; that sum only describes the magnitude of each stage, it is not the closed-loop latency.

---

## 3. Measured vs inferred checklist

| Conclusion | Type | Source | Note |
|---|---|---|---|
| 60 fps capture in game (120 on the desktop) | measured | docs/MEASUREMENTS.md | the cause is the game locking HDMI output to 60 Hz (mostly exclusive fullscreen); 120 is only possible with borderless fullscreen + uncapped frame rate + vsync off |
| Control latency D=1 frame | measured | docs/MEASUREMENTS.md | the "measured latency D" on the v1 GUI HUD; about 16.7 ms at 60 Hz |
| Inference 4.1–5.0 ms p50 (320 models) | measured | `aim2 models bench` (`docs/MEASUREMENTS.md`) | v2 cross-checks with the p50/p95 of `perception.infer_ms` |
| 9400 frames/s over serial | measured | docs/MEASUREMENTS.md | v1 used 10-byte frames; v2 uses 9-byte frames, same order of magnitude |
| Capture-card signal latency 1–5 frames | measured/empirical | docs/MEASUREMENTS.md §7 | a resolution mismatch adds 1–2 frames; MJPG adds encode/decode latency over YUYV |
| Flick limit at 60 fps is about 10 frames (167 ms) | measured/empirical | docs/MEASUREMENTS.md | at 120 fps the same frame count is ~83 ms; raising the frame rate is the only step change |
| Display/game input 0–1 frame each | inferred | 60 Hz frame interval | not an independent measurement; covered by `loop_ms` |
| Preprocessing/NMS/tracking/target-selection cost | inferred/field | code path | metrics fields exist but there is no bench nominal value; do not invent numbers |
| HID report 2 ms | default | `output.report_hz=500` | the actual report rate comes from `device.report_hz` |
| ESP32 queue/firmware latency | not measured | firmware | `device.queue_drops` is the only clue |

Inference stage, measured per model (CoreML on this machine, 2560×1600 input frames, `aim2 models bench`; see `docs/MEASUREMENTS.md`):

| Model | Format | p50 | p95 |
|---|---|---|---|
| YOLOv8 320×320 export | YOLOv8 (head/teammate/body) | 4.1 ms | 7.9 ms |
| End-to-end 320×320 export | end-to-end | 5.0 ms | 6.3 ms |
| YOLOv5 640×640 export | YOLOv5 | 7.4 ms | 8.2 ms |
| End-to-end 640×640 export | end-to-end | 16.6 ms | 20.6 ms |

How to read this: switching `sjz` does not get slower just because the input is 320; the 640 V5 is still in the 8 ms range, while the 640 end-to-end lands at 16–20 ms. Once inference p95 goes above 20 ms, the margin in `frame_age_ms` and `max_infer_age_s=250 ms` is visibly eaten away (the 640 e2e p95 already reaches 20.6 ms), so switch to a 320 model or shrink the ROI first.

> The reported conditions are 2560×1600 input frames; the `aim2 models bench` CLI currently uses random 1280×720 frames, so preprocessing/scaling differ and the numbers vary slightly. Only before/after comparisons with the same command and the same input size are meaningful.

FPS to frame-time conversion (used only to turn "frames" into milliseconds):

| Capture/display rate | 1 frame |
|---|---|
| 60 Hz | 16.7 ms |
| 120 Hz | 8.3 ms |
| 240 Hz | 4.17 ms |

---

## 4. How to read latency

### 4.1 `aim2 doctor`

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python -m dma.cli doctor --dry-run
PYTHONPATH=src .venv/bin/python -m dma.cli doctor --dry-run --probe --json
```

- `doctor` checks python/onnxruntime/opencv/pyserial/model/classes/serial port (`cli.py::cmd_doctor`).
- `--probe` runs `OnnxDetector.bench` on one synthetic image and prints `p50_ms/p95_ms/fps`; this is an **inference-stage** benchmark, not closed-loop latency.
- A model without a head class, a suspicious `head_ratio`, `report_hz<125`, `control_hz<120` and `latency_prior_s<=0` show up under "suggestions" (`tuning/calibration.py::suggest`).
- **doctor does not measure closed-loop latency**; closed-loop latency can only be estimated in a live loop that has a target.

### 4.2 The "measured latency" in the HUD / Snapshot

The `Snapshot.latency` field produced by `Engine._publish` (`LatencyEstimator.report`):

| Field | Meaning | How to read |
|---|---|---|
| `loop_ms` | closed-loop total latency estimate, in milliseconds | once stable, cross-check against `frames × 1000/capture fps` |
| `frames` | latency expressed in how many **detection frames** | at 60 fps `frames=1` ≈ 16.7 ms; **not a count of control ticks** |
| `confidence` | identification confidence, 0–1 | below `latency_min_conf` (default 0.5) means no update, prior only; `>=0.35` together with `samples>=30` counts as trustworthy (`calibration.latency_from_estimator`) |
| `measure_ms` | `loop_ms * split` | the picture-staleness part |
| `apply_ms` | `loop_ms * (1-split)` | the command-application part |
| `lead_ms` | equal to `loop_ms` | used to display the "total look-ahead" |
| `stage_infer_ms` | inference time of the current detection frame | cross-check with `perception.infer_ms` |
| `frame_age_ms` | age of the detection frame at the control tick | should be within 0–17 ms at 60 fps; occasional larger values mean the scheduler needs checking |

The v1 GUI's "measured latency D" is an early version of the same estimate (shown in frames on the HUD); v2's `frames` has the same semantics but adds confidence and a millisecond value. **Do not copy v1's D straight into `aim.latency_prior_s`**: v1's D is a frame count, v2's prior is in seconds, and v2's default of 33 ms is deliberately conservative (see §5.1).

The desktop HUD (`desktop/app.py::MainWindow._tick`) squeezes the same set of fields into one line:

```
cap 60.0Hz  ctl 240.0Hz  inf  4.7ms   err  12.3px  mode TRACK  lat  33.0ms(conf 0.00)   det 1  trk 1  fire settling
```

- `lat` is `snap.latency["loop_ms"]` and the value in parentheses is `confidence`; `conf 0.00` means the estimator has not learned anything yet (no moving target / aiming not enabled), not "latency 0".
- `inf` is the `infer_ms` of the current detection frame, not `loop_ms`.
- `err` is the control error, not latency; do not use `err` to judge latency.
- v1's "measured latency D" only exists in the old HUD of `qt_gui.py`; the v2 desktop/CLI use `loop_ms + confidence`.

### 4.3 What the calibration command actually does

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli calibrate latency --seconds 8
```

`cmd_calibrate("latency")` runs the Engine on a `SyntheticSource` and then calls `wait_for_latency`. The rectangles in the synthetic picture are **not** targets from the model's training distribution, so in practice you often get `samples=0`, `confidence=0` and `loop_ms` stuck at the 33 ms prior. Therefore:

- treat it as a **smoke test** (the Engine starts, the latency estimator does not crash);
- real calibration needs a video with targets or the live device: see [CALIBRATION](./CALIBRATION.md#4-latency-calibration).

Offline usage with targets (a video file):

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

Live usage: `aim2 run --record --duration 90`; when it finishes, the `describe()` the CLI prints contains `latency`; the `tick` records in the recording `out/telemetry/session_*.jsonl.gz` also carry `lat.loop_ms/confidence/frames`.

### 4.4 How confidence is computed

`control/latency.py::_estimate`:

1. Take the last `latency_window` (default 45) detection events, each with a screen error `(ex,ey)` and the displacement `(ux,uy)` issued during that interval.
2. For each candidate lag `m ∈ [0, max_lag]` (`max_lag` defaults to 10 and never exceeds `n-8`), compute the median residual:
   `cost(m) = median(|Δex_k + ux_{k-m}| + |Δey_k + uy_{k-m}|)`.
3. Take the `m` with the smallest cost; `confidence = 1 - best_cost / median(all_costs)`, clamped to 0–1.
4. When `conf >= latency_min_conf`, `loop_s` is smoothed towards `m * frame_dt` with `smoothing=0.25`; otherwise the confidence is multiplied by 0.5 again (meaning "nothing was learned this time").
5. `frame_dt` is the median detection interval; `frames` is therefore a number of detection frames, and `loop_ms = frames * frame_dt`.

How to read it:

- Target static and ourselves static too → the error curve is flat, all lags have similar cost, `confidence` is low and `loop_ms` sits at the last value/prior. **Both the target and the crosshair have to be moving**.
- Frequent target loss/switching → the `on_detection` samples are discontinuous and `frames` drifts; fix tracking/target selection first, then look at latency.
- To cross-check: `frames × 1000 / capture.fps` should be roughly `loop_ms` (both derive from the detection interval, so in theory they agree).

---

## 5. Stability boundaries of latency and gain

### 5.1 Plant and the "dead-beat" gain

Treat the screen error as the integral of our own output, with latency `L`:

```
e_{k+1} = e_k - u_{k-m} + Δp_k        (m = L / sample period)
```

For a pure delay plus integrator the ideal control is **dead-beat**: remove the error in one step, i.e. `u = e` (gain 1). Gain 1 is also the stability boundary: any equivalent gain above 1 overshoots under delay and forms a limit cycle; below 1 it is stable but converges slowly.

The design gains of `PredictiveAim`:

| Parameter | Default | Active range |
|---|---|---|
| `kp_near` | 0.50 | error < `near_px=26` |
| `kp_far` | 0.85 | error > `far_px=260` |
| transition | `smoothstep(26, 260, dist)` | smooth between 26–260 px |

This disagrees with the "k in [0.42, 0.75]" written in the `control/controller.py` docstring; **the dataclass defaults win** (0.50 / 0.85). The defaults are already close to but below 1, leaving roughly 15% of margin for phase/latency error.

### 5.2 Rate normalisation

The first lines of `PredictiveAim.step`:

```python
scale = clamp(dt / max(1e-3, apply_s), 0.02, 1.0)
u = scale * (kp*e + ki*I + ff) + kd*(e - e_prev)
max_u = max_step * scale
```

Why `scale` is mandatory:

- The semantics of the control law are "how many pixels should be travelled within the command-application window `apply_s`", not "how many pixels per tick".
- v1's gains were hand-tuned inside a 60 Hz loop; moving the same `kp/max_step` to 240 Hz **makes the displacement per second 4× larger**, and it necessarily oscillates under delay. This is exactly the "high gain + delay → overshoot limit cycle" mechanism in docs/MEASUREMENTS.md.
- After multiplying by `scale`, the physical meaning of `kp` is decoupled from the loop rate: the maximum displacement per second is the same at 60/120/240 Hz.

Numbers with the default parameters (`loop_s=33 ms, split=0.5, apply_s=16.5 ms, max_step=140`):

| Control rate | `dt` | `scale = dt/apply_s` | Max displacement per tick `140*scale` | Max displacement per second |
|---|---|---|---|---|
| 60 Hz | 16.7 ms | 1.0 (clamped) | 140 px | ≈ 8.4 k px/s |
| 120 Hz | 8.33 ms | 0.505 | 70.7 px | ≈ 8.5 k px/s |
| 240 Hz | 4.17 ms | 0.253 | 35.4 px | ≈ 8.5 k px/s |
| 500 Hz | 2.0 ms | 0.121 | 17.0 px | ≈ 8.5 k px/s |

(Per-second values = per-tick × rate; the rounding differences come from the `scale` clamp.)

### 5.3 How to set the gain ceiling

Engineering rules (not measurements, but derived from the model above):

1. **The equivalent loop gain is kp** (after `scale` normalisation). It must satisfy `kp < 1` to be stable; the default `kp_far=0.85` is already close to the ceiling.
2. **Latency estimation error amplifies the equivalent gain**: if the real `L` is `ε` larger than `loop_s`, feed-forward/extrapolation falls short and the error shows up as extra phase lag, making the limit cycle easier to excite. Treat `confidence < 0.35` as "latency unknown" and do not push `kp_far` any higher.
3. **`max_step` is the second limiter**: it caps the per-tick displacement and does not change the loop gain; too small and even large errors cannot move (flicks get slow), too large and feed-forward plus the integral surge together while saturated.
4. **`lead_gain` defaults to 0.9**: feed-forward compensates only 90% of the target velocity, keeping 10% of margin for velocity estimation error; `lead_max=70 px` stops one bad velocity estimate from becoming a huge feed-forward term.
5. **`latency_split` changes the allocation, not the total**: a larger `split` → more latency is treated as "picture staleness" and more extrapolation is applied; smaller → more is treated as "command application", so `apply_s` is larger and `scale` smaller. The total `loop_s` is unchanged but the residual self-motion differs; 0.5 is the starting point.
6. **`deadzone=1.5` / `deadzone_exit=5.0`**: the dead band has hysteresis so the output does not chatter in and out around 1.5 px; the group delay of `output_ema=0.9` is about `(1-α)/α * dt ≈ 0.46 ms @240 Hz` (code comment), negligible, but it does not replace `kd`.
7. **`max_accel=0` is off by default**: enabling it adds a slope limit in px/tick that is also multiplied by `scale`; it slows large flicks and cannot be used to cure oscillation (cure oscillation by lowering `kp`/`max_step` first).

### 5.4 What to change when latency grows

| Symptom | Check first | Adjustment |
|---|---|---|
| Overshoot back and forth near the target (limit cycle) | whether `loop_ms` is larger than last time; whether `confidence` is low | lower `kp_far` (0.85→0.7→0.6) and `max_step`; do not add `kd` first |
| Rebound after a flick lands | `aim.saturated` counter, `fire.reason` | lower `max_step` or `kp_far`; check whether `lead_max` is too large |
| Persistent lag while tracking | `loop_ms`, `lead_gain` | raise `lead_gain` (≤1.0) or `ki` (inside `integral_gate`); confirm the latency estimate is trustworthy first |
| Jitter while static | `output_ema`, `deadzone`, tracker noise | raise `deadzone`/`deadzone_exit`, lower `output_ema` (more smoothing), or raise `kd` |
| Capture fps drops from 120 to 60 | `capture.fps`, game display mode | first switch the game to borderless fullscreen/uncapped/vsync off; align the capture card to the native resolution |
| `frame_age_ms` p95 jumps | whether the control thread is blocked by hot reload/UI | check `control.hz`; hot reload rebuilds the controller (see [OPERATIONS](./OPERATIONS.md#6-common-troubleshooting)) |

**Do not** compensate "large latency" by raising `max_step`; raising the step size when latency is large only amplifies the limit cycle. The conclusion in docs/MEASUREMENTS.md is "the larger D is, the smaller the `max_step/kp` that remains stable".

---

## 6. Recommended budget accounting practice

1. After a cold start, look at the inference p95 from `aim2 doctor --probe` to confirm the compute stage has not regressed.
2. Run 60–90 seconds of live device or video with a moving target, and wait until `confidence ≥ 0.35` and `frames` is stable.
3. Record `loop_ms`, `frames`, `frame_age_ms` p95, `perception.infer_ms` p95 and `transport.write_ms` p95.
4. Cross-check `loop_ms` with `frames × 1000/capture.fps`; a large deviation means the detection interval is unstable (dropped frames / target-selection jumps).
5. Only touch `kp_far/max_step/lead_gain` once `loop_ms` is stable; change one at a time and A/B with `aim2 replay` or on the live device.
6. Write the results into `docs/` or the session summary under `out/telemetry`; do not leave them in your head.

---

This document corresponds to schema_version 2 and the src/dma layout; if you change the code, please update it here as well.
