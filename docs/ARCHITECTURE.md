<div align="right">

[简体中文](zh-CN/ARCHITECTURE.md) | **English**

</div>

# Architecture (v2)

> ⚠️ **For learning, research and engineering study only.**
> Do not use this project in any competitive or ranked setting. Automated aiming
> violates the terms of service of most online games and may lead to permanent bans.
> You are solely responsible for how you use it.

> Code baseline: `src/dma`, config `schema_version = 2`, CLI version `aim2 2.0.0` (`src/dma/core/__init__.py`).
> Every default in this document comes from the dataclass defaults in `src/dma/core/config.py`; numbers marked "measured" come from runtime observable fields or `docs/MEASUREMENTS.md`. Inferred values are marked as such.
> Related docs: [latency budget](./LATENCY_BUDGET.md), [calibration](./CALIBRATION.md), [operations](./OPERATIONS.md), [tuning](./TUNING.md), [ADR-0001](./adr/0001-predictive-control.md), [ADR-0002](./adr/0002-world-frame-tracking.md).

---

## 1. System boundary

Physical link (unchanged from v1; v2 only rewrote the Mac-side software):

```
Windows (game) ──HDMI──▶ USB capture card ──▶ Mac (capture + ONNX inference + tracking + control)
      ▲                                                     │
      └────────── ESP32 USB HID mouse ◀── serial ◀──────────┘
```

Division of responsibility in v2:

| Side | Owns | Does not own |
|---|---|---|
| Mac / `src/dma` | frame capture, inference, tracking, target selection, control, fire decisions, protocol packing, metrics and logging | in-game input injection, HID enumeration |
| ESP32 firmware | parsing serial frames, injecting mouse deltas/buttons at the HID report rate, reporting STATUS back | vision, control law, parameter tuning |
| Game | applying HID input on the next input tick, outputting the picture over HDMI | any form of cooperation |

---

## 2. Layers

Dependencies may only point downwards; `control` does not import `cv2`/`onnxruntime`, `transport` does not import `control`, and `core` does not import any layer above it.

```
┌──────────────────────────────────────────────────────────────────────────────┐
│ Interface   cli.py (aim2)   desktop/app.py (PySide6)   qt_gui.py (v1 legacy) │
│             sim/  tuning/  telemetry/  (offline tools, off the loop)         │
├──────────────────────────────────────────────────────────────────────────────┤
│ Engine      engine/runtime.py   Engine: wiring, control tick, health,        │
│             snapshot                                                         │
├──────────────────────────────────────────────────────────────────────────────┤
│ Control     control/latency.py   control/policy.py                           │
│             control/controller.py   control/fire.py                          │
├──────────────────────────────────────────────────────────────────────────────┤
│ Perception  perception/geometry.py   perception/decoders.py                  │
│             perception/detector.py   perception/tracker.py                   │
├──────────────────────────────────────────────────────────────────────────────┤
│ Capture     capture/sources.py   capture/grabber.py                          │
│ Transport   transport/protocol.py   transport/planner.py                     │
│             transport/link.py   transport/serial_link.py                     │
├──────────────────────────────────────────────────────────────────────────────┤
│ Base        core/config.py   core/types.py   core/metrics.py                 │
│             core/logging.py   core/clock.py                                  │
└──────────────────────────────────────────────────────────────────────────────┘
```

How the layering rules show up in practice:

- `perception/detector.py` depends only on `core`, `geometry` and `decoders`, returns a `DetectionFrame`, and does not know the controller exists.
- `control/controller.py` only receives `ControlInput` (pure numbers); it does not read frames and does not write the serial port, which is why `sim/harness.py` can drive it with fake time.
- `transport/planner.py` only quantises "pixels → HID counts" and splits that over the report rate; it does not decide where to aim.
- `engine/runtime.py` is the only composition root: model, capture, link, tracker, policy, controller, fire gate and latency estimator are all wired in `Engine.__init__`.

---

## 3. Thread model

### 3.1 Thread inventory

| Thread | Entry point | Cadence | Reads | Writes | Blocking point |
|---|---|---|---|---|---|
| main/control thread | `Engine.run` → `FixedRate(runtime.control_hz)` → `Engine.step` | 240 Hz (default) | `_det`, `cam_line`, `link` state | `_snap`, `cam`, `cam_line`, `link` accumulators, fire state | no blocking I/O; `step` does only numeric work and enqueueing |
| capture thread | `FrameGrabber._run` (`capture/grabber.py`) | actual capture-card frame rate (60 Hz measured in game) | `FrameSource.read` | `_packet` (replaced under lock) | `cv2.VideoCapture.read` blocks |
| inference thread | `Engine._infer_loop` (`engine/runtime.py`) | runs whenever there is a new `seq` | `grabber.latest()` | `_det` (replaced under lock) | `onnxruntime` inference |
| link thread | `SerialLink._run` (`transport/serial_link.py`) | `output.report_hz` (default 500 Hz) plus heartbeat | `planner` accumulators, button state, serial port | serial writes, `planner.stats`, device telemetry | serial write (bounded by `write_timeout_s`) |
| UI thread | `desktop/app.py::MainWindow._tick` (QTimer) polls `Engine.snapshot()` | 30 Hz (33 ms) | `_snap` | none | read-only; `EngineThread` runs `Engine.run()` separately, so the UI never blocks control |
| hotkey thread | `pynput.keyboard.Listener` (only enabled by `aim2 run`) | event driven | none | toggles via `Engine.set_*_enabled` | none |

In `aim2 run` the main thread *is* the control thread; `Engine.start()` is what creates the capture/inference threads, and `Engine.start()` calls `SerialLink.start()`. `Engine.stop()` stops inference, capture and serial in that order and sets health to `STOPPED`.

### 3.2 Threads and shared state

```
 HDMI / capture card
      │  cv2.read()
      ▼
┌──────────────┐  FramePacket(ts, seq, frame) ┌──────────────┐
│ dma-grab     │ ───── _packet (Lock) ──────▶ │ dma-infer    │
│ FrameGrabber │                              │ detector     │
└──────────────┘                              └──────┬───────┘
                                                       │ DetectionFrame
                                                       ▼
                                             ┌────────────────────┐
                                             │ _det (Lock)        │
                                             └────────┬───────────┘
                                                      │ consumed only when frame_id changes
                                                      ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│ main/control thread 240 Hz: tracker → policy → controller → fire → publish   │
│   cam accumulator / DelayLine are written by this thread only                │
└───────────────┬──────────────────────────────────────────────────────────────┘
                │ submit_motion(dx,dy) / submit_button(pressed)
                ▼
┌────────────────────────────────┐     planner.take(now)     ┌────────────────┐
│ dma-link 500 Hz                │ ◀──────────────────────── │ MotionPlanner  │
│ SerialLink._run                │ ─── serial v2 frames ───▶ │ ESP32 → HID    │
│ STATUS / heartbeat / reconnect │ ◀── STATUS / telemetry ── │                │
└────────────────────────────────┘                           └────────────────┘
                │
                ▼
        Engine.snapshot() ──▶ UI / CLI print / recording
```

Shared state and how it is synchronised:

| State | Producer | Consumer | Synchronisation |
|---|---|---|---|
| `FrameGrabber._packet` | capture thread | inference thread, control thread (takes the `frame` reference) | `threading.Lock`, whole packet replaced; stale frames are dropped rather than queued |
| `Engine._det` | inference thread | control thread | `_det_lock`; the control thread de-duplicates by `frame_id` |
| `Engine._snap` | control thread | UI/CLI | `_lock`, whole snapshot replaced |
| `Engine.cam`, `cam_line` | control thread | control thread | no lock, single writer |
| `planner` accumulators | control thread `submit_motion` | link thread `take` | single producer / single consumer; `MotionPlanner` holds an internal `threading.Lock` covering `add/take/reset/report` |
| `SerialLink` device telemetry | link thread | `status()` / metrics | plain attribute assignment (atomic under the GIL) |
| `fire`/`policy`/`tracker` state | control thread | control thread | no lock, single writer |

Design trade-offs:

- **No frame queue.** The capture thread keeps only the newest frame; when inference is slow, old frames are dropped instead of piling up. A backlog *is* input latency.
- **No output queue.** `LinkBase.submit_motion` only adds the delta into the `MotionPlanner` accumulator, and the link thread drains it at the report rate; a stalled serial port merges deltas instead of letting latency grow with queue length.
- **The control thread does no I/O.** Serial writes, capture reads and inference all live in other threads; the control thread only does numpy/scalar maths and `submit_*`.

### 3.3 Timing of one tick

The exact order inside `Engine.step()` (`engine/runtime.py`):

```
t = monotonic()
dt = clamp(t - last_tick, 1e-4, 0.2)

1. det = _det (reference taken under lock)
2. health, detail = _evaluate(t, det)
     - grabber.stale_s()*1000 > runtime.watchdog_ms    → DEGRADED "capture stream interrupted" # capture stream stalled
     - det is None                                     → DEGRADED "waiting for the first inference" # waiting for the first inference result
     - t - det.capture_ts > runtime.max_infer_age_s    → DEGRADED "inference result stale" # inference result stale
     - not link.ok()                                   → DEGRADED "output link down" # output link down
3. if det.frame_id != _last_det_used (new detection frame):
     a. update center, policy.frame_w/h from the latest frame size; on size change
        clear cam_line/latency
     b. t_sample = det.capture_ts - latency_split * latency.loop_s
        cam_meas = cam_line.sample(t_sample)
     c. world_dets = det.detections translated by (+cam_meas)
     d. tracks = tracker.update(world_dets, det.capture_ts)
     e. target = policy.select(tracks, cam_meas, center, now)
     f. if target is not None and not coasting:
            latency.on_detection(det.capture_ts, target.screen - center)
     g. if target.tid changed: controller.reset()   # integral/EMA belong to the old target
4. can_aim = aim_enabled and health == OK
   if can_aim and target:
     age    = now - det.capture_ts
     apply  = max(dt, loop_s * (1 - latency_split))
     input  = ControlInput(world_xy, vel, cam_now=cam, cam_meas,
                           age_s=age + loop_s*latency_split, apply_s=apply,
                           sigma=track.sigma, coasting=target.coasting)
     out    = controller.step(input)
     link.submit_motion(out.dx, out.dy)
     latency.add_motion(out.dx, out.dy)
     cam   += (out.dx, out.dy)
     cam_line.push(now, cam)
     if source has set_camera: source.set_camera(*cam)   # closes the loop for SyntheticSource
   if target is None: controller.reset()
5. fire:
     lead = loop_s + (now - det.capture_ts)
     target_pt = aim_world + vel*lead - cam
     box_impact = screen_box translated by (vel*lead + cam_meas - cam)
     decision = fire.update(target, preview, track.sigma, now)
     link.submit_button(fire.pressed(now) and fire_enabled and can_aim)
6. _publish(now, det) → assemble Snapshot (health, latency, cached metrics, stage state,
   reference to the latest frame)
```

Timing notes:

- **Tracking only updates on detection frames** (when `det.frame_id` changes), while the controller runs on every control tick. The tracker is timestamped with `det.capture_ts`, not `now`, so the Kalman `dt` equals the capture interval.
- **World coordinates are established before the tracker**: `world_dets = source box + cam_meas`. `cam_meas` is "the displacement we had already issued when this frame was produced", sampled by the delay-sampling line at `capture_ts - split*loop_s`.
- **The latency estimator only consumes measured positions**: coasted targets are extrapolations, and feeding them to `LatencyEstimator` teaches it a latency that does not exist; `Engine.step` calls `on_detection` only when `not target.coasting`.
- **Switching targets resets the controller**: `controller.reset()` on a `target.tid` change keeps the integral from dragging the crosshair back towards the old target; this is also the structured replacement for v1's `SWITCH_JUMP` patch in v2.
- **The controller input carries both measurement age and application delay**: `age_s` makes the controller extrapolate the target to "now", `apply_s` normalises the gains to the command-application window. Their sum is roughly `loop_s + frame_age`.
- **The fire preview uses the whole loop latency**: `lead_s = loop_s + det_age`, which is `(1-split)*loop_s` more than the controller's `age_s`, because the command still has to go through the application delay before the bullet lands.
- **When health is not OK, only aiming/firing stop; capture, inference and the link keep running**: this keeps recovery and observation possible. When `can_aim` is false, `target` is still updated and the HUD still shows detection/tracking results.

Start and stop:

```
Engine.start:
  grabber.ensure_open() → grabber.start() → link.start() (if any)
  _running.set() → start infer thread → health=OK
Engine.run(duration):
  loop step() at FixedRate(control_hz); exit on timeout or Ctrl-C
Engine.stop:
  _running.clear() → join infer → grabber.stop → source.close → link.close → health=STOPPED
```

The first few hundred milliseconds after start are normally `DEGRADED / capture stream interrupted` or `DEGRADED / waiting for the first inference`, which is normal warm-up; see [OPERATIONS](./OPERATIONS.md#3-health-states).

---

## 4. Coordinate frames

The module docstring of `core/types.py` defines four coordinate frames; this is the engineering view of them.

| Frame | Definition | Origin/unit | Producer | Consumer | Transform |
|---|---|---|---|---|---|
| `source` | pixel coordinates of the captured frame | top-left, px | `Transform.to_source_boxes`, `Detection` | before tracker input, recording, UI drawing | inverse of the model transform |
| `model` | network input tensor coordinates (N×N) | top-left, px | `Transform.to_model_xy`, `detector._preprocess` | ONNX output decoding | `x_m = (x_s - crop_x0)*sx + pad_x` |
| `world` | source pixel coordinates with our own camera motion removed | same scale as source; the origin is "the picture at the moment the aiming session started" | `world_dets` in `Engine.step`, `Track` | tracker, policy, controller, recording/replay | `world = source + cam_meas` |
| `screen` | coordinates of the picture the player actually sees | top-left, px; the crosshair sits at `center=(w/2,h/2)` | `TargetPolicy.select`, controller error | controller, fire preview, UI | `screen = world - cam_now` |

### 4.1 source ↔ model

`Transform` in `perception/geometry.py` is the same affine transform in both directions:

- `make_transform(src_w, src_h, in_w, in_h, crop, mode)`:
  - `mode="letterbox"`: preserve aspect ratio, `sx=sy=min(in_w/cw, in_h/ch)`, grey padding 114;
  - `mode="stretch"`: stretch directly, `sx=in_w/cw, sy=in_h/ch`, no padding;
  - `crop=(x0,y0,x1,y1)` supports ROI.
- `OnnxDetector._preprocess` uses the same `transform.crop`, `sx/sy` and `pad_x/pad_y` to crop/scale/pad; `detect` finally uses `transform.to_source_boxes` to map boxes back to source. **Preprocessing and the inverse transform share one object**, which fixes the root cause behind v1's two hand-written mappings and the boxes that were off by a few percent.
- `letterbox="auto"` (`OnnxDetector._resolve_letterbox`): `e2e` models use `stretch`, `yolov5/yolov8` use `letterbox`. `model_report` (`tuning/calibration.py`) errors out directly with "a yolov8 model preprocessed with stretch will produce shifted boxes".

### 4.2 source ↔ world

In `Engine.step`:

```python
cam_meas = self.cam_line.sample(det.capture_ts - measure_offset)
world_dets = [det.box.translate(cam_meas[0], cam_meas[1]) ...]
```

- `cam` is "the displacement we have issued since aiming was enabled", in screen pixels; `cam_line` is the `(ts, cam_x, cam_y)` history and `DelayLine.sample` interpolates linearly.
- `cam_meas` is the camera position **at the moment the frame was produced**, not the current one; `measure_offset = latency_split * loop_s` (default 0.5 × 33 ms = 16.5 ms).
- `world = source + cam_meas` means: add back the picture shift caused by camera movement to recover the target's position in the map/world.
- This is an **incremental world frame, not an absolute map**: the origin is where the aiming session started; the player's own hand movement on the mouse, recoil and in-game camera shake never enter `cam`, so they are not compensated. `set_aim_enabled(False)` and frame-size changes both clear `cam`/`cam_line`. See [ADR-0002](./adr/0002-world-frame-tracking.md).
- A synthetic source must feed `cam` back into the scene to form a closed loop: `Engine.step` calls `source.set_camera` when the source has it (`SyntheticSource.set_camera`). Otherwise the picture never moves, targets "accelerate away forever", and the tracking results from `aim2 run --sim` are meaningless.

Precondition (must be calibrated):

- 1 px of output must correspond to 1 px of picture shift. `MotionPlanner.add` multiplies `dx` by `output.gain` and converts it to HID counts; if the "counts → screen pixels" ratio is not 1, the whole world frame is scaled, and both tracking speed and feed-forward go wrong. Calibration procedure in [CALIBRATION](./CALIBRATION.md#3-sensitivity-and-gain-calibration).

### 4.3 world ↔ screen

```python
# policy.select
sx, sy = wx - cam_x, wy - cam_y
# controller.step
ex = (world_x + vx*age) - cam_now_x - center_x
```

- `screen` is the picture the player sees; the crosshair is fixed at `center`.
- The policy uses `screen` for FOV and top/bottom mask filtering; the controller computes its output from the `screen` error.
- The fire preview translates the target box from `screen_box` by `vel*lead + cam_meas - cam` to get where the target will be "once every command has taken effect"; see the `ImpactPreview` construction in `Engine.step`.

### 4.4 Coordinate invariants

1. A `Box` inside `Detection`/`DetectionFrame` is **always** in `source` coordinates; `Track.box` is **always** in `world` coordinates (`core/types.py::Track` docstring).
2. `Transform`'s `to_source_*` and `to_model_*` must be inverses; any new preprocessing must go through `make_transform`, and no separate scaling may be written inside a detector.
3. `world = source + cam_meas` and `screen = world - cam_now` are inverses of each other; never mix `cam` and `cam_meas` within the policy/controller.
4. The crosshair centre is always `(w/2, h/2)` in the `screen` frame; `policy.frame_w/h` is updated when the frame size changes, and the delay-sampling line and latency estimator are cleared at the same time (in `Engine.step`).

---

## 5. Module responsibilities and key invariants

### 5.1 core

| Module | Key functions/classes | Responsibility | Invariants / failure modes |
|---|---|---|---|
| `core/config.py` | `Settings`, `load`, `save`, `from_dict`, `migrate_v1`, `validate`, `set_fields`, `diff`, `FIELDS` | versioned config, migration, validation, atomic saves, field-level diff | every config section is frozen; unknown fields go into `warnings` instead of being silently dropped; `save` uses temp file + fsync + rename; `validate` checks ranges/enums/`near_px<far_px`/`deadzone_exit>=deadzone`/even width and height |
| `core/types.py` | `Box`, `Detection`, `DetectionFrame`, `Track`, `ControlOutput`, `FireDecision`, `Health`, `Snapshot` | data contract between stages | all frozen/slots; `Box` is source, `Track` is world; `Health` has five states |
| `core/metrics.py` | `RollingStats`, `Metrics`, `Stopwatch` | fixed-window percentiles, counters, gauges | `RollingStats.add` is O(1) and only `summary()` sorts; the control thread only adds samples and never reads |
| `core/logging.py` | `setup`, `JsonFormatter`, `ConsoleFormatter`, `StageLogger` | JSONL + console logging, rate limiting | files rotate at 8 MiB × 3; `StageLogger.throttled` rate-limits by key |
| `core/clock.py` | `FixedRate`, `DelayLine`, `RateMeter`, `smoothstep`, `clamp` | monotonic clock, drift-free pacing, delay-sampling line | every `ts` is `time.monotonic()`; after an overload `FixedRate` resynchronises instead of accumulating catch-up; `DelayLine.sample` clamps the endpoints and interpolates linearly |

### 5.2 capture

| Module | Key functions/classes | Responsibility | Invariants |
|---|---|---|---|
| `capture/sources.py` | `CvSource`, `VideoSource`, `SyntheticSource`, `open_source` | device/video/synthetic frame sources | sources are "dumb": they only `open/read/close`; reconnects, timestamps and frame-drop policy all live in the grabber |
| `capture/grabber.py` | `FrameGrabber._run`, `latest`, `stale_s` | dedicated capture thread, newest-frame slot, FPS, reconnect on stream loss | keeps only the newest frame; closes and reopens only after 8 consecutive read failures (`_fails >= 8`); every successful read stamps `capture_ts` |

`CvSource`'s open parameters come from `capture.*`: `fourcc` defaults to `MJPG`, `width/height` to 1280×720, `fps` to 120, `buffer_size` to 1. Note that `capture.fps` is a **request**: the real frame rate is decided by the capture card / the game's HDMI output; in game it was measured to be locked at 60 Hz (docs/MEASUREMENTS.md).

### 5.3 perception

| Module | Key functions/classes | Responsibility | Invariants |
|---|---|---|---|
| `perception/geometry.py` | `Transform`, `make_transform`, `center_crop` | source↔model affine transform, ROI | preprocessing and inverse transform share one `Transform`; letterbox padding is 114 grey |
| `perception/decoders.py` | `infer_format`, `decode_e2e`, `decode_yolov5`, `decode_yolov8`, `nms`, `per_class_nms` | decoding for three detection heads + NMS | pure numpy, no onnxruntime dependency; e2e `(N,6)`; v5 `obj*cls`; v8 has no objectness and is transposed automatically |
| `perception/detector.py` | `OnnxDetector.__init__`, `_load`, `_roi`, `_preprocess`, `detect`, `bench` | session management, provider chain, metadata classes, ROI, decoding, class filtering | `resolve_providers` always appends the CPU fallback; the class whitelist is applied before NMS; `teammate/ai/crosshair` are in the default `ignore_classes` and can never reach the tracker |
| `perception/tracker.py` | `KalmanBox`, `Tracker.update`, `predicted_box` | 6-state Kalman + greedy IoU association | state `[cx,cy,vx,vy,w,h]`; `Q` uses a white-noise-acceleration model; `update` uses the Joseph form to keep P positive semi-definite; association is per class (`det.cls_id == trk.cls_id`); a track is dropped only when `misses > max_age` |

Model family (from `aim2 models inspect` / metadata on this machine):

| Model | Input | Output | Decoding | Classes | Measured p50/p95 (CoreML) |
|---|---|---|---|---|---|
| YOLOv8 320×320 export | 320×320 | `[1,9,2100]` | `yolov8` | `body, head, teammate, ai, crosshair` | 4.1 / 7.9 ms |
| End-to-end 320×320 export (example) | 320×320 | `[1,300,6]` | `e2e` | `st, qiu, dj, fyb, sg1..sg4` (class table of the example model) | 5.0 / 6.3 ms |
| YOLOv5 640×640 export | 640×640 | `[1,25200,7]` | `yolov5` | `head, body` | 7.4 / 8.2 ms |
| End-to-end 640×640 export | 640×640 | `[1,300,6]` | `e2e` | same end-to-end export family | 16.6 / 20.6 ms |

> Measurement conditions: CoreML on this machine, 2560×1600 input frames, `aim2 models bench` (`docs/MEASUREMENTS.md`). This is not closed-loop latency, only the inference stage.

`ClassMap.is_targetable` requires both "in `vision.target_classes`" and "not in `vision.ignore_classes`"; the default targets are `body/head/st/person/enemy/target` and the default ignores are `teammate/ai/crosshair`. This is a code-level guarantee that "a teammate can never become the target", rather than something the selection logic hopes for.

### 5.4 control

| Module | Key functions/classes | Responsibility | Invariants |
|---|---|---|---|
| `control/latency.py` | `LatencyEstimator.add_motion`, `on_detection`, `_estimate`, `report` | closed-loop latency identification | estimates only on **detection events**; candidate lags `0..max_lag`; the smallest median of `\|Δe + u_{k-m}\|` wins; `confidence = 1 - best/median(all)`; `loop_s` is smoothed only when `conf >= min_conf` |
| `control/policy.py` | `TargetPolicy.select`, `_fuse`, `_aim_point` | head/body fusion, target selection, hysteresis, coast | with a head class, aim at the centre of the head box; without one, `y1 + h*head_ratio`; the head box must lie inside the body box and have area < 1.4×body; `switch_margin` penalises non-current targets; coast only when there are no candidates |
| `control/controller.py` | `PredictiveAim.step`, `LegacyAim.step`, `build_controller` | predictive control law | gains scheduled by error; feed-forward sits outside kp; integral only inside a small-error gate; saturation projects the allowed integral by headroom (anti-windup); output EMA plus `max_step` limiting; `max_accel` is optional |
| `control/fire.py` | `FireController.update`, `pressed` | fire gate state machine | a hit needs "inset box hit **or** residual ≤ `fire.radius`", combined with dwell, sigma, not-coasting and cooldown/single-shot; every rejection reason is counted |

Key controller formulas (`PredictiveAim.step`):

```
scale = clamp(dt / apply_s, 0.02, 1.0)
kp    = kp_near + (kp_far-kp_near) * smoothstep(near_px, far_px, dist)
ff    = clamp(v * apply_s * lead_gain, ±lead_max)   # ×0.4 while coasting
u     = scale * (kp*e + ki*I + ff) + kd*(e - e_prev)
u     = slew_limit(u); u = clamp_magnitude(u, max_step*scale)
I    += e*dt (only while dist < integral_gate), clamped to integral_limit
allowed integral ceiling while saturated = (max_u - |PD+FF|)/ki
```

See [LATENCY_BUDGET](./LATENCY_BUDGET.md#5-stability-boundaries-of-latency-and-gain) and [ADR-0001](./adr/0001-predictive-control.md) for details.

### 5.5 transport

| Module | Key functions/classes | Responsibility | Invariants |
|---|---|---|---|
| `transport/protocol.py` | `crc8`, `encode`, `encode_move`, `decode_status`, `StreamParser.feed`, `legacy_*` | v2 frames, CRC8/ATM, seq, device telemetry, v1 compatibility | v2 magic `0xA5`, version/type byte `0x21`, payload ≤255, CRC covers header+payload; on a bad frame `StreamParser` drops 1 byte and resynchronises |
| `transport/planner.py` | `MotionPlanner.add`, `take`, `flush` | sub-frame planner: accumulate deltas, split them over the report rate, carry the remainder, clamp to ±127 | `add` multiplies by `output.gain`; the accumulator is capped at `4*max_per_report`; `take` emits at most 8 reports per call; the remainder is carried over instead of being dropped |
| `transport/link.py` | `LinkBase`, `MockLink`, `build_link` | link abstraction, dry-run | no queues; `MockLink` only records and never touches the port |
| `transport/serial_link.py` | `SerialLink._run`, `_open`, `_emit_motion`, `_read_available`, `status` | serial thread, reconnect, heartbeat, telemetry | buttons are sent only on change; motion is sent every report period; 5 consecutive write errors close and reopen the port; `ok()` requires the port to be open and errors < 5 |

Protocol frame (v2):

```
0      magic 0xA5
1      version (high 4 bits) = 2 | type (low 4 bits)
2      seq
3      payload length
4..    payload
last   CRC-8/ATM (poly 0x07) over bytes[0 : 4+len]
```

- `T_MOVE=1`: payload `<hh>` dx,dy (int16; the planner guarantees ±127 per report).
- `T_BUTTON=2`: 1-byte mask.
- `T_PING=3`: 4-byte counter.
- `T_STATUS=4` (device → host): `STATUS_LEN=13`, `struct "<IHHHHB"` → uptime_ms(u32), rx_frames(u16), crc_errors(u16), queue_drops(u16), report_hz(u16), flags(u8). Firmware `firmware/esp32_hid/README.md` §5 matches the host byte for byte.
- v1 compatibility: with `output.protocol="v1"` the host sends the old 10-byte `0xAA` frames, and the firmware has to match.

### 5.6 engine

| Function | Responsibility | Invariants |
|---|---|---|
| `Engine.__init__` | wire detector/source/grabber/link/tracker/policy/controller/fire/latency/cam_line | every dependency is injectable (replaced in tests/simulation) |
| `Engine.start/stop/run` | lifecycle and fixed pacing | start order: capture → link → inference; stop order is the reverse |
| `Engine.step` | the only place that couples the stages together | see §3.3; `tracker` updates only when `frame_id` changes; `cam` accumulates only when `can_aim and target` |
| `Engine._evaluate` | health decision | four DEGRADED reasons, see §5.7 |
| `Engine._publish` | assemble `Snapshot` | the UI is read-only; cached metrics refresh every `metrics_interval_s` |
| `Engine.apply` | hot-reload the sections that do not need reopening hardware | changes to `vision/capture/output.port/baud` only produce a restart hint; any diff rebuilds the controller (see §7 known issues) |
| `Engine.describe` | operations summary | health/detail/model/capture/link/latency/shots/frame_size |

### 5.7 Health states

| State | Produced in | Meaning | Aim/fire |
|---|---|---|---|
| `BOOT` | `Engine.__init__` | not started | no |
| `OK` | `Engine._evaluate`, all checks pass | capture/inference/link all healthy | allowed |
| `DEGRADED` | four branches of `_evaluate` | one stage is unhealthy; details in `health_detail` | suppressed (`can_aim=False`) |
| `FAILED` | **no assignment path in the current code** | reserved for unrecoverable errors | no |
| `STOPPED` | `Engine.stop` | stopped | no |

The four `DEGRADED` detail strings are the operational criteria:

| detail | Trigger | Threshold | Action |
|---|---|---|---|
| `capture stream interrupted` (capture stream stalled) | `grabber.stale_s()*1000 > runtime.watchdog_ms` | 400 ms | check HDMI/capture card/camera permission/whether OBS holds the device |
| `waiting for the first inference` (waiting for first inference) | `det is None` | — | wait for warm-up; if it persists, check the model/inference thread for exceptions |
| `inference result stale` (inference result stale) | `now - det.capture_ts > runtime.max_infer_age_s` | 250 ms | check inference p95, whether capture dropped to 0, whether the thread is blocked |
| `output link down` (output link down) | `not link.ok()` | port not open, or 5 consecutive write errors | check serial/firmware/cable; `status()["detail"]` holds the exception type |

### 5.8 sim / tuning / telemetry / cli

| Module | Key functions | Responsibility | Invariants |
|---|---|---|---|
| `sim/world.py` | `Scenario.pose/velocity`, `build`, `SCENARIOS` | deterministic target motion | pure functions of time, no randomness (the rng from query seed is not used for pose); exactly 6 scenarios |
| `sim/fake_detector.py` | `SyntheticDetector.detect/describe/bench` | threshold detector (HSV colour segmentation), no onnxruntime | emits the same `DetectionFrame` as the ONNX path; only recognises the blue/green rectangles of the synthetic picture, with `cls_name="body"`; used by `aim2 run --sim` and the desktop `--sim` |
| `sim/harness.py` | `simulate`, `compare`, `_recovery` | closed-loop simulation, bench metrics, v1/v2 comparison | capture/detection rate, half-loop latency, quantisation remainder and dropouts are all configurable; the `legacy` control group is forced to `control_hz=detect_hz` with no tracker |
| `tuning/autotune.py` | `evaluate`, `autotune`, `SearchSpace` | offline coordinate descent | objective `track_rms + 0.35*jitter + 2*recovery + penalty`; accepts only strictly better candidates; `passes` defaults to 2 |
| `tuning/calibration.py` | `model_report`, `head_ratio_from_session`, `latency_from_estimator`, `wait_for_latency` | calibration helpers | a model without a head class only warns; teammates/bots listed as targets is an error; head_ratio needs ≥20 samples |
| `telemetry/recorder.py` | `SessionRecorder.frame/tick/fire`, `load_session`, `summarize` | session recording | gzip JSONL with four record kinds: header/frame/tick/fire; frame stores source boxes + cam samples; tick stores delta/target/latency |
| `telemetry/replay.py` | `replay`, `replay_compare` | replay real recordings through different controllers | the world trajectory is rebuilt from "source boxes + cam samples" and does not depend on the controller that was active at the time |
| `cli.py` | `cmd_doctor`, `cmd_models`, `cmd_sim`, `cmd_bench`, `cmd_autotune`, `cmd_replay`, `cmd_run`, `cmd_record`, `cmd_config`, `cmd_calibrate` | unified entry point | every subcommand has `--json`; the no-hardware path (`--sim/--dry-run`) and the hardware path share the same Engine |
| `desktop/app.py` | `MainWindow`, `EngineThread`, `ParamRow`, `PreviewWidget` | PySide6 console | parameter widgets are generated from `core.config.FIELDS`; edits go through `set_fields` + `Engine.apply` hot reload; polls `Snapshot` at 30 Hz; the preview overlays detection/tracking/target boxes |

---

## 6. Why a world frame

v1 locked onto a point in the screen frame (`lock=(aim_x,aim_y)` in `main_cli.py`), which brings four structural problems:

1. **Own motion contaminates target velocity.** The difference of the screen error `e = p_world - c` contains both the enemy's velocity and ours; the PID's D term and any velocity feed-forward treat our own output as target motion, creating positive feedback. v1 could only suppress this with low gains and heavy damping, at the cost of tracking lag.
2. **Identity is unstable across multiple targets.** In the screen frame, whenever the target moves the lock point moves; v1 cleared the integral with `SWITCH_JUMP` and kept tracks alive with `COAST_FRAMES`, both patches. v2 keeps identity in `Track.tid`, and velocity/covariance follow the identity.
3. **Latency compensation has no reference frame.** Predicting "where the enemy will be when the command takes effect" requires knowing "how the enemy moves relative to the world"; in the screen frame that quantity changes with every step we take, so it cannot be extrapolated.
4. **Recordings cannot be reproduced.** If only screen boxes are stored, replaying with a different controller yields different self-motion, and the apparent target trajectory changes too. v2 stores `cam` samples, so the world trajectory is independent of the controller and replay A/B is possible.

Costs of the world frame:

- output sensitivity must be calibrated (1 px ↔ 1 px);
- the delay-sampling line is required (otherwise self-motion still leaks into the tracker);
- the origin/reset of an incremental world frame has to be handled (aim toggle, resolution change);
- the player's own mouse movement, recoil and camera shake are outside the compensation scope and remain a known residual.

---

## 7. Extension points

### 7.1 Swapping the model

1. Drop the `.onnx` into `models/`.
2. `aim2 models inspect end-to-end 320×320 export --json` to see `input/format/targets/heads/letterbox`.
3. `aim2 config set vision.model end-to-end 320×320 export`; adjust `vision.target_classes` / `ignore_classes` / `conf` / `iou` / `roi` as needed.
4. Restart the process (`Engine.apply` only hints at a restart for `vision.*`; it does not rebuild the detector).
5. Verify: `aim2 doctor --probe`, `aim2 models inspect`, and the detection boxes and classes in the on-device HUD.

Hard constraints on the model side:

- Class names come from the ONNX metadata `names`; without metadata, `_parse_names` falls back to parsing `"0: body, 1: head"` style strings, and if that also fails there are only numeric class names, so `ClassMap` cannot map `body/head` to ids, `targets` is empty and `model_report` reports an error.
- The output head must be one of the three; a new shape needs a predicate in `decoders.infer_format`, a `decode_*` in `decoders.py`, and a dispatch branch in `OnnxDetector._decode`.
- `yolov8` must go through letterbox (`vision.letterbox=auto` already guarantees this; setting `stretch` by hand is reported as an error by `model_report`).

### 7.2 Swapping output/actuator

- A new link implements `submit_motion` / `submit_button` / `ok` / `status` / `close` from `transport/link.py::LinkBase` and is dispatched in `build_link`.
- A new protocol adds `T_*` and `encode_*` / `decode_*` in `transport/protocol.py`; `StreamParser` only knows `MAGIC` and CRC, so new types need no parser change.
- The firmware must match `output.protocol`: v2 is `0xA5` + CRC8; v1 is `0xAA` + XOR.

### 7.3 Swapping the game

| Change | Where | Notes |
|---|---|---|
| resolution/capture card | `capture.width/height/fps/fourcc` | must match the game's native resolution (docs/MEASUREMENTS.md: a mismatch adds 1–2 frames of scaling latency); width and height must be even |
| crosshair/FOV | `policy.fov_radius`, `top_mask`, `bottom_mask` | HUD positions differ per game; masks are proportional to the picture |
| head | `vision.target_classes` containing `head` makes `head_ratio` unnecessary; otherwise `policy.head_ratio` | without a head class it must be calibrated on site |
| sensitivity | `output.gain` + in-game sensitivity | a 1 px command must produce 1 px of picture shift, otherwise the world frame is distorted |
| fire decision | `fire.*` | hitboxes/latency differ per game; validate aiming first with `fire.mode=off` |
| input tick | `output.report_hz`, `runtime.control_hz` | when the game is locked at 60 Hz, raising the control rate does not raise the visible update rate, but it does shrink the quantisation step |

### 7.4 Swapping the controller / adding UI

- Controller: implement `step(ControlInput) -> ControlOutput`, add a `kind` attribute, dispatch in `control/controller.py::build_controller`, and add an `aim.controller` enum value. `Engine.apply` compares `kind`, so the controller is rebuilt only when the control law actually changes.
- UI: read only the `Snapshot` returned by `Engine.snapshot()`; add fields in `_publish` rather than reading tracker/controller internals from the UI thread. The current implementation is in `desktop/app.py`: `MainWindow._tick` takes a snapshot every 33 ms, `EngineThread` runs `Engine.run()` on its own thread, parameter widgets are generated from `FIELDS`, and edits take effect through `set_fields` + `Engine.apply`.
- Desktop entry point: `PYTHONPATH=src .venv/bin/python -m dma.desktop.app` (`--sim` for no hardware, `--dry-run` to avoid writing the serial port).
- Tests: decoders/geometry are pure numpy and suited to unit tests; tracking/control use `sim/harness.py`; end-to-end uses `aim2 run --sim --dry-run`.

---

## 8. Current implementation boundaries (where code and description disagree)

These are facts confirmed in the code tree while writing this document; operations/tuning must know them:

| Item | Current state | Impact |
|---|---|---|
| `desktop/app.py` | implemented: `FIELDS`-generated parameters, 30 Hz `Snapshot`, `EngineThread` running the engine, `set_fields` + `Engine.apply` hot reload, preview overlay | desktop entry point is `python -m dma.desktop.app [--sim]`; `qt_gui.py` is still the v1 flat-parameter GUI (imports `dma.config/vision/aim`) and does not drive the v2 Engine |
| `tests/` | 8 test files implemented; `pytest -q` passes fully on this machine (currently 79 cases) | covers config/geometry/three decoders/NMS/tracking/latency/rate normalisation/target selection/fire/protocol resync/planner/engine/simulation/record-replay |
| `firmware/esp32_hid/` | implemented: `esp32_hid.ino`, `platformio.ini`, `README.md`; STATUS is 13 bytes and matches the host protocol byte for byte | before flashing, set `USB-OTG (TinyUSB)` + `USB CDC On Boot: Enabled` per README §2; HID at 1000 Hz |
| `Engine.apply` controller check | fixed to `getattr(self.controller, "kind", None)` | the controller is rebuilt only when the control law changes; ordinary parameter edits no longer clear integral/EMA state |
| `fire.radius` | already fixed to "inset box hit **and** residual ≤ radius"; defaults `radius=9`, `inset=0.06` | larger is looser (fires more easily), smaller is stricter; together with `inset` it defines the firing window |
| `policy.lock_radius` | extrapolation drift cap in `TargetPolicy.select`: `\|v\|·Δt` beyond it abandons the coast | prevents chasing a ghost from before the teleport; too small degrades tracking through occlusion |
| `aim.settle_px` | used only in `sim/harness.py` for the first-capture metric | not used by real-time control |
| `capture.warmup_s` | not used by `FrameGrabber`/`CvSource` | start-up warm-up is determined by the capture card itself and by waiting for the first frame |
| `runtime.record`/`record_dir` | not read by `Engine`; `cli._run_engine` reads them and creates a `SessionRecorder` | using `Engine` directly does not record |
| `runtime.log_level`/`log_dir` | not read by `Engine`; `cli.main` hardcodes `setup_logging(level="INFO", log_dir=None)`; the desktop app uses `console=False` | the CLI only logs to the console and never to disk; the desktop app does not log to the console; for JSONL call `setup(..., "out/logs")` explicitly |
| `FAILED` | `Health.FAILED` has no assignment path | do not rely on that state for alerting |
| `run_mac.sh` | seven entry points: `run / run-dry / gui / sim / doctor / selftest / bench` | all go through `python -m dma.cli`, with paths relative to the script itself |
| `aim2 bench` | the `legacy` control group is forced to 60 Hz with no tracker in `sim/harness.py::compare` | that is the intentional "v1-equivalent" baseline; the gate passes with the current defaults |
| `telemetry/replay.py` | replay sets every detection's `cls_name` to `"body"` | replay cannot reproduce head-class fusion/head preference; `head_ratio` still works |
| world-frame reset | `set_aim_enabled(False)` resets `cam`/`cam_line`, a frame-size change clears `cam_line`/`latency`, but neither clears `tracker` | old world tracks disagree with the new origin, so the aim may jump once after re-enabling or changing resolution; see [ADR-0002](./adr/0002-world-frame-tracking.md#53-known-gaps-in-the-current-code) |
| `Engine.set_aim_enabled(False)` | now calls `planner.reset()` and drops the leftover delta | no tail motion after aiming is disabled |

These are not "documentation disclaimers" but the real boundaries of the current code; changing any of them should update this document and the corresponding docs in the same commit.

---

This document corresponds to schema_version 2 and the src/dma layout; if you change the code, please update it here as well.
