<div align="right">

[简体中文](../zh-CN/adr/0002-world-frame-tracking.md) | **English**

</div>

# ADR-0002: Track targets in the world frame, not the screen frame

> ⚠️ **For learning, research and engineering study only.**
> Do not use this project in any competitive or ranked setting. Automated aiming
> violates the terms of service of most online games and may lead to permanent bans.
> You are solely responsible for how you use it.

- Status: Accepted
- Date: 2026-11-21
- Decision scope: `src/dma/core/clock.py` (DelayLine), `src/dma/engine/runtime.py`, `src/dma/perception/tracker.py`, `src/dma/control/policy.py`
- Related: ADR-0001 (predictive control), [ARCHITECTURE §4](../ARCHITECTURE.md#4-coordinate-frames), [CALIBRATION §3](../CALIBRATION.md#3-sensitivity-and-gain-calibration)

---

## 1. Background

v1 maintained the lock point in screen coordinates:

```python
# main_cli.py (v1)
lock = (aim_x, aim_y)          # screen pixels
...
dx, dy = ctrl.update(aim_x, aim_y, cx, cy, dt)
```

The difference of the screen error `e = p_screen - center` contains two parts at once:

```
Δe = Δp_enemy - Δc_self
```

- `Δp_enemy`: the enemy's own motion;
- `Δc_self`: the screen translation caused by the mouse displacement we send.

Consequences:

1. **The velocity estimate is polluted**. Any velocity/feedforward/D term based on `Δe` treats our own output as target motion. At high gain this forms positive feedback ("the more it chases, the more it jitters"), at low gain it lags.
2. **Latency compensation has no reference**. To predict "where the enemy will be when the command applies" you must know how the enemy moves relative to the world; in the screen frame that quantity changes with our every step.
3. **Identity is unstable**. When the target box jumps on screen the lock point jumps; v1's `SWITCH_JUMP` to clear the integral and `COAST_FRAMES` to keep it alive are both patches.
4. **Replay is not reproducible**. Only screen boxes are stored, so a different controller produces different self-motion, and the target's apparent trajectory changes too.

v1's "D=1 frame", "high-gain spring" and "tracking lag" can all be attributed to the same structural problem: **the controller does not distinguish "the target is moving" from "we are moving"**.

---

## 2. Decision

### 2.1 Definition

```
world  = source + cam_meas        (camera position at the instant the frame was produced)
screen = world  - cam_now         (the frame we see now)
```

- `source`: pixel coordinates in the captured frame (`Detection.box`).
- `cam`: the displacement we have cumulatively sent since aiming was enabled (screen pixels, `Engine.cam`).
- `cam_meas`: the value of `cam` sampled **at the instant the frame was produced**, taken from `DelayLine`.
- `world`: target position in the world (incremental world frame, origin = start of the aiming session).
- `screen`: the frame the player sees; the crosshair is always at `center=(w/2,h/2)`.

`Track.box`, `Track.vx/vy` are all in **world** coordinates (`core/types.py::Track` docstring); `TargetPolicy.select` selects targets in world and then converts to screen for FOV/masks/error.

### 2.2 How the delay line is sampled

`Engine.step` does this for every newly processed detection frame:

```python
measure_offset = aim.latency_split * latency.loop_s
cam_meas = cam_line.sample(det.capture_ts - measure_offset)
world_dets = [d.box.translate(*cam_meas) for d in det.detections]
tracks = tracker.update(world_dets, det.capture_ts)
```

`DelayLine` (`core/clock.py`):

- `push(now, cam_x, cam_y)`: the control thread calls it after every actual displacement; it pushes only when `can_aim and target`.
- `sample(ts)`: binary search + linear interpolation in the history buffer; `ts` outside either end is clamped to the endpoint; the buffer is `maxlen=512`, about 2.1 seconds of history at 240 Hz.
- `span_s`: the current history span, useful for diagnostics.

**Why sample at `capture_ts - measure_offset` instead of `capture_ts`**:

- `capture_ts` is when the `grabber` read this frame, not when this frame's content was actually produced; the capture card/driver/display add more latency.
- The camera position corresponding to the frame content is earlier than "the frame was read"; `measure_offset` is the estimate of the "measurement side" share of the total latency (default `0.5 × 33 ms = 16.5 ms`).
- If we sampled directly at `capture_ts`, we would add back our own motion from after the frame was produced, and part of the self-motion would still remain in the world frame; the symptom is target velocity underestimated/overestimated and feedforward in the wrong direction.

`latency_split` allocates, it does not set the total: the default 0.5 is a starting point; if the capture card latency is large and inference is fast, raise `split` (more counted as stale frame); if command application is slow (HID/game input tick), lower it. The total `loop_s` stays the same.

### 2.3 Lifecycle of the world frame

| Event | `cam` | `cam_line` | `tracker` | Note |
|---|---|---|---|---|
| Startup | (0,0) | empty | empty | origin |
| Aiming enabled | kept | kept | kept | keeps accumulating |
| Aiming disabled | reset (0,0) | `reset()` | **not reset** | controller reset; world origin changes |
| Frame size change | kept | `reset()` | **not reset** | also `latency.reset()`; see the risks in §4 |
| Target lost | stops accumulating | stops pushing | coasts to `max_age` | the world frame keeps its last state |
| Process restart | (0,0) | empty | empty | not persisted |

**Important**: this is an **incremental world frame**, not an absolute map. The origin is at the most recent "aiming enabled/disabled" or process start; the player's own manual mouse movement, recoil and camera shake are not in `cam`, so they are not compensated. It is not "map coordinates", only the reference frame that "subtracts our commands from the observation".

---

## 3. Why not other options

| Option | Problem |
|---|---|
| **Screen-frame tracking + subtract command velocity** | the command velocity itself has latency/quantisation; `Δe - u` still has a phase error, and you need to know when `u` took effect. It is the world frame done after the fact, but more prone to instability |
| **Image self-motion estimation (phase correlation/optical flow)** | when the game camera pans, the background moves as a whole and it is in principle estimable; but low-texture walls/sky fail, the computation is not worth it within the 4 ms control budget, and latency alignment is still needed. Possible future redundancy, not the main path |
| **IMU / game API / driver hook** | unavailable or unreliable on the target machine; high cross-platform cost |
| **Full homography** | for an FPS camera with fixed FOV and no roll, yaw/pitch are approximately translation; a homography adds more parameters and calibration for insufficient benefit |
| **Background feature tracking** | parallax, occlusion, dynamic objects interfere; neither real-time behaviour nor robustness is as direct as "known command displacement" |

---

## 4. Consequences of failed self-motion compensation

This is the most important part of this ADR: the world frame is not free — break any of the following and the tracker treats "our movement" as "the enemy's movement".

| Failure mode | Mechanism | Symptom | Detection/mitigation |
|---|---|---|---|
| **Output gain not calibrated** | `MotionPlanner.add` sends `gain*dx` counts; the game converts counts to pixels at a ratio other than 1 → `cam` and real screen displacement have different scales | world-frame velocity amplified/reduced; feedforward overshoots or under-compensates; oscillation at high gain, trailing at low gain | calibrate per [CALIBRATION §3](../CALIBRATION.md#3-sensitivity-and-gain-calibration); watch whether the target box drifts while you strafe in `aim2 run --dry-run` |
| **Wrong `latency_split`** | `cam_meas` sampled at the wrong instant → `Δc_self` remains in the world frame | target velocity shows a fake "it speeds up whenever we flick"; latency identification `confidence` low/`frames` drifting | start with the default 0.5; compare `aim.err_px`/`confidence` at different `split`; do not treat it as a gain |
| **Wrong `loop_s` estimate** | `measure_offset` does not match reality; `apply_s` is wrong too | extrapolation/feedforward direction off; limit cycle | look at `confidence`; fall back to the prior below 0.2; fix capture/tracking before estimating |
| **Delay line reset** | `cam_line` is empty, `sample` returns (0,0), but the world tracks in `tracker` still use the old origin | world coordinates jump on the next frame; the policy may switch targets/coast; false fire decisions | allow 2–3 frames of settling after a resolution change/aim toggle; restart the process if needed |
| **World drift while aiming is off** | `cam` is reset to 0 but the player moved the mouse manually; after re-enabling, the origin no longer matches the old tracks | the target jumps at the instant of re-enabling | also clear the tracker when disabling aiming (the current code does not, see §5) or show a hint in the UI |
| **Unmodelled self-motion** | manual mouse movement, recoil, camera shake, game camera smoothing/acceleration | the enemy shows extra motion in the world frame; drift accumulates over long tracking | this is residual, not a bug; limit the impact with `tracker.accel_noise`/`meas_noise` and a short coast |
| **Sampling outside the history** | `sample` clamps to the endpoint; the control thread has not pushed for a long time (no target/link stuck) | the world frame stops updating, the target position freezes | look at `cam_line.span_s`; do not coast too long without a target |
| **Frame size change** | `cam_line.reset()`/`latency.reset()`, but `tracker` keeps the old world tracks | one jump, possibly a wrong lock | restart after changing resolution; at least wait for the policy to re-select |

### 4.1 Why these failures "look like control-parameter problems"

- wrong world-frame scale → wrong velocity estimate → wrong feedforward → looks exactly like "`kp_far` too high";
- wrong sampling instant → self-motion leaks into the tracker → looks exactly like "large detection jitter";
- delay line reset → target jumps → looks exactly like "not enough switch hysteresis".

So the troubleshooting order must be: **confirm the world frame/calibration first, then tune control parameters**. This is also why [CALIBRATION](../CALIBRATION.md) puts gain calibration before latency calibration.

---

## 5. Consequences and open issues

### 5.1 Benefits

- the tracker's velocity/covariance reflect only the target's own motion, which is what makes feedforward and fire preview meaningful.
- `Track.tid` identity is stable; the policy's `switch_margin` is genuine "switch hysteresis", not a "screen jump patch".
- the recording stores `cam` samples (`SessionRecorder.frame`), so a replay can rebuild world trajectories, and changing the controller does not change the target trajectory (`telemetry/replay.py`).
- the simulation harness can inject "self motion" and "target motion" separately, which is what makes regression meaningful.

### 5.2 Costs

- `output.gain` must be calibrated, otherwise the whole world frame is scaled.
- `loop_s`/`split` must be estimated online, otherwise the sampling instant is wrong.
- the world frame is incremental/local and cannot be reused across aiming sessions; a process restart/resolution change loses the origin.
- the recording file stores one extra `cam` field compared to the screen frame; replay has to rebuild in the world frame (the current `replay.py` still hardcodes the class name).

### 5.3 Known gaps in the current code

| Gap | Impact | Location |
|---|---|---|
| `set_aim_enabled(False)` resets `cam`/`cam_line` but does not clear `tracker` | when re-enabled the old world tracks do not match the new origin, may jump once | `Engine.set_aim_enabled` |
| A frame size change only clears `cam_line`/`latency`, not `tracker` | same as above | the size-change branch of `Engine.step` |
| `SyntheticSource` needs `set_camera` to close the loop | without feeding the camera position back in synthetic frames the target "accelerates away forever"; `Engine.step` already calls it via `hasattr(source, "set_camera")` | `capture/sources.py::SyntheticSource.set_camera` |
| Replay hardcodes the class name to `body` | head fusion cannot be reproduced; the world trajectory itself can still be rebuilt | `telemetry/replay.py` |
| Manual player mouse/recoil not modelled | world-frame residual; long tracking drifts | a design boundary, not a short-term bug |

---

## 6. Code locations and verification

| Content | Location |
|---|---|
| World-frame definition | `core/types.py` module docstring, `Track` docstring |
| Delay line | `core/clock.py::DelayLine.push/sample/span_s` |
| Sampling and translation | `engine/runtime.py::Engine.step` (`measure_offset`, `world_dets`) |
| Screen-frame target selection | `control/policy.py::TargetPolicy.select` (`sx = wx - cam_x`) |
| Controller error | `control/controller.py::PredictiveAim.step` (`ex = tx - cam_now - center`) |
| Fire preview | `engine/runtime.py::Engine.step` (`ImpactPreview`) |
| Recording | `telemetry/recorder.py::SessionRecorder.frame` (stores `cam` samples) |
| Replay | `telemetry/replay.py::replay` |
| Synthetic self-motion | `capture/sources.py::SyntheticSource.set_camera` |

Verification method:

1. Live `aim2 run --dry-run`, strafe left and right yourself and watch whether the target box is stable in the world frame (the track dashed boxes in the UI should not drift as a whole with your movement).
2. Record a strafe + tracking session, then `aim2 replay --compare`: the world trajectory should be independent of your mouse motion at the time; both controllers should run on the same target trajectory.
3. Change `output.gain` by ±10% and see whether `aim.err_px`/`confidence` degrade significantly — if they do, the world-frame scale is sensitive to gain and must be recalibrated.
4. Change `latency_split` to 0.3/0.5/0.7 and look at `confidence` and `aim.err_px`; pick the value with high confidence and low error, not by feel.

---

## 7. When to re-evaluate

- If the exact "input apply instant" or camera pose becomes available from a driver/game API in the future, `cam` can be upgraded from "accumulated commands" to "measured camera displacement", and the world frame will be more accurate.
- If the player's manual mouse movement takes a large share (for example manual correction when tapping with semi-automatic weapons), physical mouse input must be included in `cam`, otherwise the world-frame residual grows.
- If the game camera has non-linear smoothing/acceleration the pure translation model fails, and a camera model with velocity/acceleration states is needed.
- If the capture resolution/sensitivity changes during a session, the world frame and tracker must be reset; currently only the delay line is reset.

---

This document corresponds to schema_version 2 and the src/dma layout; please update it when the code changes.
