<div align="right">

[简体中文](zh-CN/TUNING.md) | **English**

</div>

# Tuning Handbook (TUNING)

> ⚠️ **For learning, research and engineering study only.**
> Do not use this project in any competitive or ranked setting. Automated aiming
> violates the terms of service of most online games and may lead to permanent bans.
> You are solely responsible for how you use it.

> Code baseline: `src/dma`, schema 2. Defaults come from `src/dma/core/config.py`; the simulation comparison numbers are quoted from `docs/MEASUREMENTS.md` §4 (closed-loop simulation, 33 ms link latency, 2.5 px detection noise, 60 fps capture, 240 Hz control), not bench measurements.
> Tuning order: **calibrate first ([CALIBRATION](./CALIBRATION.md)) → then simulate → then replay → live hardware last**. Without calibration, tuning only compensates for a wrong world frame/latency.

---

## 1. Three hard rules

1. **Gain is normalised per "command apply window"**, not per tick. `u = scale*(kp*e + ki*I + ff) + kd*Δe`, `scale = dt/apply_s`. So the same `kp` behaves identically at 60/120/240 Hz; do not retune `kp` just because you changed the control rate.
2. **The physical upper bound of `kp` is 1** (dead beat). The defaults `kp_near=0.50` and `kp_far=0.85` are already close; do not push `kp_far` any higher unless the latency estimate has `confidence ≥ 0.35` and both simulation and replay pass.
3. **Change one parameter at a time**, and leave a `config diff` plus `bench`/`replay` results afterwards. v1's 33 sliders relied on feel; v2 uses numbers.

---

## 2. Parameter semantics

### 2.1 `aim` (control law)

| Parameter | Default | Range | Meaning | Increase | Decrease |
|---|---|---|---|---|---|
| `controller` | `predictive` | `predictive｜legacy` | control law selection | `legacy` = v1 PD+I adapter (A/B/rollback only) | — |
| `kp_near` | 0.50 | 0–1.5 | near-distance proportional gain (error < `near_px`) | faster at point blank, too large jitters/overshoots near the target | steadier, but slow to converge up close |
| `kp_far` | 0.85 | 0–1.5 | far-distance proportional gain (error > `far_px`) | faster flicks, >1 easily limits-cycles | slower flicks, more latency tolerance |
| `near_px` | 26 | 2–120 px | start of the near bandwidth | enters low gain earlier, stable but slower overall | enters low gain later, more aggressive at point blank |
| `far_px` | 260 | 40–600 px | end of the far bandwidth | high gain applies only to larger errors, stable | high gain covers more errors, fast but jittery |
| `ki` | 0.05 | 0–0.8 | integral coefficient (removes steady-state error) | less tracking lag, too large windup/rebound | more conservative, steady-state error left to kp |
| `kd` | 0.10 | 0–1.0 | derivative damping (per-tick difference) | suppresses overshoot/jitter, too large amplifies detection noise | more responsive, but tends to overshoot |
| `integral_limit` | 60 | 0–300 px | integral term ceiling | allows larger steady-state compensation, more risk | more conservative |
| `integral_gate` | 70 | 0–200 px | integrate only when the error is below this | integrates over a wider range, may windup on large errors | integrates only very close, prevents flick rebound |
| `lead_gain` | 0.9 | 0–1.5 × | velocity feedforward strength; 1.0 = fully compensate target motion within `apply_s` | tighter tracking, too large jitters on velocity-estimation noise | more conservative, more lag |
| `lead_max` | 70 | 0–250 px | single-shot feedforward clamp | allows more prediction, a bad velocity estimate is amplified | steadier, fast targets under-compensated |
| `latency_prior_s` | 0.033 | 0–0.20 s | latency prior before online identification converges | more conservative before convergence (more look-ahead) | more aggressive before convergence; 0 means no prediction |
| `latency_split` | 0.5 | 0.2–0.8 | share of the total latency that is "stale frame" | more extrapolation, may overshoot | more counted as apply latency, `apply_s` grows, `scale` shrinks |
| `latency_window` | 45 | 10–200 tick | latency identification window | smoother, slower | faster, jittery |
| `latency_min_conf` | 0.2 | 0–1 | below this confidence `loop_s` is not updated | more conservative, slow to converge | updates more easily, may be biased by noise |
| `max_step` | 140 | 10–400 px | maximum displacement per tick (then multiplied by `scale`) | faster flicks, fewer saturations; too large overshoots | steadier, large errors do not move |
| `max_accel` | 0 (off) | 0–200 px/tick | output change limit per tick (multiplied by `scale`) | smoother, slower flicks | 0 = off |
| `deadzone` | 1.5 | 0–20 px | inner radius of the dead band | steadier at rest, duller fine adjustment | more responsive, may jitter |
| `deadzone_exit` | 5.0 | 0–40 px | radius to leave the dead band (hysteresis) | harder to be pulled out by small errors, stable | easier to leave, may jitter in and out |
| `output_ema` | 0.9 | 0–1 | output smoothing coefficient; 1 = no smoothing | more responsive, more noise | smoother, adds group delay (`(1-α)/α·dt`) |
| `settle_px` | 7.0 | 1–40 px | **simulation only**, first-capture criterion | no live effect | no live effect |

### 2.2 `policy` (target selection)

| Parameter | Default | Range | Meaning | Increase | Decrease |
|---|---|---|---|---|---|
| `fov_radius` | 550 | 80–1200 px | maximum on-screen distance of a candidate target | can lock farther targets, more wrong locks/drifting | locks only nearby, stable but narrow |
| `lock_radius` | 160 | 20–500 px | drift limit of extrapolation after loss (coast is abandoned once `\|v\|·Δt` exceeds it) | tolerates occlusion/fast movement better, but may chase a phantom | gives up earlier, stable but easily lost under occlusion |
| `switch_margin` | 55 | 0–300 px | distance penalty for non-current targets (switch hysteresis) | sticks to the current target, slow to switch | switches more easily, wavers between targets |
| `coast_s` | 0.18 | 0–0.6 s | extrapolation duration after the target is lost | more continuous through brief occlusion, over-extrapolates on long disappearances | gives up faster, may reset frequently |
| `head_bonus` | 28 | 0–120 px | distance bonus for head targets | prefers the head more, may pick a far target over a near one | favours distance more, head priority drops |
| `top_mask` | 0.08 | 0–0.40 | fraction of the top of the screen ignored (HUD) | filters more HUD, may miss high targets | filters less |
| `bottom_mask` | 0.12 | 0–0.45 | fraction of the bottom of the screen ignored (hands/weapon) | filters more bottom clutter | filters less |
| `head_ratio` | 0.11 | 0–0.40 | for models without a head class, the head aim point ratio inside the box | aim point moves down (neck/chest), too high hits the body | aim point moves up (top of head/air) |
| `prefer_head` | true | bool | whether head targets get the bonus | when off, only distance matters | — |
| `min_hits` | 1 | 1–6 | minimum hits required to select a target | more conservative, new targets appear slowly | locks faster, misdetection risk |

### 2.3 `fire` (firing)

| Parameter | Default | Range | Meaning | Increase | Decrease |
|---|---|---|---|---|---|
| `mode` | `single` | `off｜single｜auto` | firing mode | `auto` repeats; `single` fires one shot per lock | `off` never fires |
| `radius` | 12 | 0–60 px | fallback radius from the impact point to the aim point (an **or** with the inset box) | fires more easily (may fire early) | stricter |
| `inset` | 0.06 | 0–0.40 | inset ratio of the target box | more conservative (must be closer to the centre) | looser (edge hits count too) |
| `dwell_s` | 0.04 | 0–0.30 s | how long the conditions must hold before firing | steadier, slower reaction | faster, may fire halfway |
| `cooldown_s` | 0.12 | 0.02–0.60 s | minimum interval between two shots | slower repeat | faster repeat |
| `tap_s` | 0.03 | 0.005–0.15 s | how long the key is held down | longer press (the weapon may repeat) | shorter tap |
| `max_sigma` | 80 | 1–300 px | upper bound of Kalman position uncertainty | more willing to fire while uncertain | more conservative, no fire when a new target appears |

For troubleshooting `fire.reason` see [OPERATIONS §6.4](./OPERATIONS.md#64-fire).

### 2.4 `tracker` (Kalman tracking)

| Parameter | Default | Range | Meaning | Increase | Decrease |
|---|---|---|---|---|---|
| `enabled` | true | bool | whether tracking is enabled; when off every frame's detection becomes its own track | when off there is no velocity/covariance, the controller degrades | — |
| `iou_thr` | 0.25 | 0.05–0.80 | IoU required for association | stricter association, identity breaks easily | looser association, may merge two people into one |
| `max_age` | 12 | 1–40 frames | how many missed frames before a track is deleted | tolerates occlusion longer, may keep ghosts | cleans up faster, slower re-acquisition |
| `accel_noise` | 3000 | 100–12000 px/s² | manoeuvre capability σa (process noise) | follows faster and estimates velocity more responsively, also jitters more | smoother, lags on fast direction changes |
| `meas_noise` | 3.5 | 0.5–20 px | detection position noise σ | trusts the model more, smoother | trusts detections more, more responsive but jittery |
| `size_noise` | 6.0 | 0.5–30 px | box size noise σ | smoother size changes | size follows more closely |

### 2.5 `output` / `runtime` / `capture` / `vision` (operations-related)

| Parameter | Default | Meaning | Note |
|---|---|---|---|
| `output.protocol` | `v2` | `v1｜v2` | must match the firmware; `v1` has no telemetry |
| `output.port` | `auto` | serial port path | specify explicitly with multiple devices; changing it needs a restart |
| `output.baud` | 921600 | baud rate | changing it needs a restart |
| `output.report_hz` | 500 | HID report rate | sub-frame smoothness; the real firmware/device value is in `device.report_hz` |
| `output.gain` | 1.0 | HID counts multiplier per pixel of command | **world-frame scale**; must be calibrated per [CALIBRATION §3](./CALIBRATION.md#3-sensitivity-and-gain-calibration) |
| `output.max_per_report` | 127 | int8 limit per report | limited by HID; the planner splits reports |
| `output.heartbeat_ms` | 100 | heartbeat period | too small increases serial load |
| `output.write_timeout_s` | 0.02 | write timeout | too small causes false failures, too large stalls the link |
| `output.dry_run` | false | do not write to serial | for integration testing |
| `runtime.control_hz` | 240 | control thread frequency | changing it needs a restart of the `run` loop (`FixedRate` is created at startup) |
| `runtime.watchdog_ms` | 400 | capture staleness threshold | too large discovers faults slowly, too small false-alarms |
| `runtime.max_infer_age_s` | 0.12 s | maximum staleness of inference results | match it to inference p95 and capture fps |
| `runtime.metrics_interval_s` | 1.0 | metric cache refresh | only affects `Snapshot.metrics` refresh |
| `runtime.log_level` / `log_dir` | INFO / `out/logs` | logs | the CLI currently uses `log_dir=None`, see [OPERATIONS §4](./OPERATIONS.md#4-logs) |
| `runtime.record` / `record_dir` | false / `out/telemetry` | session recording | only read by the CLI `run --record`/`record` |
| `capture.width/height` | 1280×720 | capture resolution | must align with the game's native one; even width and height; changing it needs a restart |
| `capture.fps` | 120 | requested frame rate | 60 in practice when the game is locked to 60; trust `capture.fps` |
| `capture.fourcc` | MJPG | capture format | MJPG adds encode/decode latency over YUYV (docs/MEASUREMENTS.md) |
| `capture.buffer_size` | 1 | driver buffer | >1 adds latency |
| `capture.warmup_s` | 2.0 | **currently unused by the grabber** | startup warm-up is determined by waiting for the first frame |
| `vision.model` | `models/model.onnx` | model path | changing it needs a restart; the output head (e2e/v5/v8) is auto-detected |
| `vision.conf` | 0.35 | confidence threshold | changing it needs a restart; lower it for misses, raise it for false detections |
| `vision.iou` | 0.45 | NMS IoU | changing it needs a restart; lower it for duplicated boxes |
| `vision.max_det` | 32 | maximum number of detections | for multi-target scenes |
| `vision.letterbox` | auto | preprocessing | `yolov8` must use letterbox |
| `vision.roi` / `roi_size` | off / 512 | ROI cropping | use it when the target is always centred; ROI changes the effective resolution |
| `vision.target_classes` / `ignore_classes` | body/head/st/... / teammate/ai/crosshair | class whitelist/blacklist | changing it needs a restart; never put `teammate` into target |

---

## 3. Automatic tuning

### 3.1 Objective function

`tuning/autotune.py::evaluate`:

```
cost_scenario = tracking_rms_px
              + 0.35 * jitter_px
              + 2.0  * min(recovery_s, 0.6)
              + penalty
cost = mean(cost_scenario over scenarios)
```

Penalties (`_penalty`):

| Condition | Penalty |
|---|---|
| `tracking_rms` not finite | 1e6 (eliminated immediately) |
| `tracking_rms > 120 px` | +500 |
| `jitter_px > 6.0` | +(jitter-6)*25 |
| `on_target < 0.5` | +(0.5-on_target)*200 |

A non-finite `recovery_s` (no large deviation event) counts as 0; `min(...,0.6)` caps it.

### 3.2 Search space

`SearchSpace.values` (default):

| Field | Candidate values |
|---|---|
| `kp_near` | 0.28, 0.36, 0.42, 0.50, 0.58 |
| `kp_far` | 0.50, 0.62, 0.75, 0.88, 1.00 |
| `ki` | 0.0, 0.05, 0.10, 0.18, 0.30 |
| `kd` | 0.0, 0.03, 0.06, 0.10, 0.16 |
| `lead_gain` | 0.70, 0.85, 1.00, 1.15 |
| `deadzone` | 0.0, 1.0, 2.0, 3.0 |

Coordinate descent, `passes=2`, a candidate is accepted only when `cost < best_cost - 1e-6`. By default 6 scenarios × about 30–50 evaluations, seconds on this machine.

### 3.3 Usage

```bash
cd ~/dma
export PYTHONPATH=src

# look at the result without writing to disk
.venv/bin/python -m dma.cli autotune --passes 2 --json

# write back into the aim section (to --settings or the default configs/settings.json)
.venv/bin/python -m dma.cli autotune --passes 2 --write
```

Output:

| Field | Meaning |
|---|---|
| `cost` / `baseline_cost` / `improvement` | after tuning / before tuning / relative improvement |
| `aim` | the complete `AimCfg` (not just the changed fields) |
| `history` | `(field,value,cost)` of every candidate |
| `rows` | `track_rms/jitter/on_target/recovery_s` per scenario |

Notes:

- The search-space upper bound `kp_far=1.0` is exactly the stability boundary. Autotune **may** push it to 1.0; before accepting you must run `bench` + `replay` and re-check on live hardware with `confidence≥0.35`.
- The objective includes `jitter` and `recovery`, so it will not turn the output neurotic for the sake of RMS; but the simulation has no real detection jitter spectrum, so live hardware may still be jitterier.
- `autotune` only optimises the `aim` section; `policy/fire/tracker` are not in the search space, changing those is manual + simulation.
- `--write` saves the complete configuration (including the migrated `output.gain` etc.), not just the 6 fields; back up before writing.
- `autotune`'s simulation uses `SimConfig()` defaults: 240 Hz control, 60 Hz detection, 33 ms latency, 2.5 px noise, 4% dropout, seed=11. To change them, pass `sim=` in Python.

### 3.4 Combinations not to change together

| Combination | Risk |
|---|---|
| `kp_far` ↑ + `max_step` ↑ | fewer saturations but larger overshoot/limit cycles |
| `ki` ↑ + `integral_gate` ↑ | integrates even on large errors, rebound after a flick |
| `kd` ↑ + `meas_noise` ↓ | derivative amplifies detection noise, jitters at rest |
| `lead_gain` ↑ + `tracker.accel_noise` ↑ | velocity estimate more responsive, feedforward overshoots more easily |
| `deadzone` ↑ + `output_ema` ↓ | double smoothing, dull fine adjustment and tracking lag |
| `switch_margin` ↑ + `fov_radius` ↑ | locks onto one person, cannot hit the next one |
| `fire.inset` ↓ + `fire.radius` ↑ | fire window too wide, fires halfway |

---

## 4. Fields you should not edit by hand

### 4.1 Rules

- Use `aim2 config set <path> <value>` instead of editing JSON: `set_fields` does type conversion, range/enum validation and atomic saving; hand edits bypass validation and only fail at the next `load`.
- After changing, keep a diff with `aim2 config diff --other backup.json`; `Engine.apply` writes the old/new of every field into the log.
- After changing `vision.*`/`capture.*`/`output.port|baud`/`runtime.control_hz` you must **restart**; hot reload only covers the runtime fields of tracker/policy/aim/fire/output.

### 4.2 Fields that are currently inert or simulation-only

These fields exist but have no consumer in the current code; do not use them to tune feel:

| Field | Status |
|---|---|
| `aim.settle_px` | only used by the first-capture metric in `sim/harness.py` |
| `capture.warmup_s` | not used by `FrameGrabber`/`CvSource` |

### 4.3 Fields that need calibration/restart to change

| Field | Reason |
|---|---|
| `output.gain` | world-frame scale; if changed you must recalibrate sensitivity, otherwise tracker/feedforward are all wrong |
| `policy.head_ratio` | depends on the model's box definition/viewpoint; changing model/resolution needs recalibration |
| `vision.model` / `vision.conf` / `vision.iou` / `vision.target_classes` | the detector is loaded in `Engine.__init__`; `apply` does not rebuild it |
| `capture.*` | the capture card is already open with the old parameters; `apply` only reports "restart needed" |
| `output.port` / `output.baud` | the serial port is already open; `apply` only reports "restart needed" |
| `runtime.control_hz` | `FixedRate` is created when `Engine.run` starts; hot reload does not change the cadence |
| `runtime.log_level` / `log_dir` | `setup_logging` is called only once at process start |
| `runtime.record` / `record_dir` | only decided at CLI startup whether to create a `SessionRecorder` |

### 4.4 Side effects of hot reload

`Engine.apply` uses `getattr(self.controller, "_kind", None)` to decide whether the controller changed, while the controller actually exposes `kind`. As a result **any** config diff is judged a "controller change" and rebuilds the controller:

- `PredictiveAim.reset()` clears the integral, EMA and dead-band state;
- changing an unrelated parameter while tracking briefly zeroes/rebuilds the output;
- when tuning, change parameters while the target is gone/fire is off, or restart after changing.

---

## 5. Simulation regression verification

### 5.1 Scenarios

`sim/world.py::SCENARIOS` (deterministic, pure functions of time):

| Scenario | Duration | Motion | Focus |
|---|---|---|---|
| `strafe_mid` | 6 s | 0.8 Hz sine, ±120 px | steady-state mid-range tracking |
| `strafe_far` | 7 s | 0.55 Hz sine, ±190 px, offset 360 px | far range/prediction error |
| `jitter` | 5 s | multi-frequency small jitter | noise suppression |
| `dash` | 5 s | 3 teleports of 60 ms/about 200–500 px | overshoot/recovery |
| `peek` | 6 s | 1.5 s period, 0.85 s visible | coast/re-acquisition |
| `micro_drift` | 6 s | 6 px/s + 3 px/s slow drift | steady-state error/integral |

### 5.2 Metrics

`sim/harness.py::SimMetrics`:

| Metric | Definition |
|---|---|
| `rms_px` | error RMS over all visible ticks |
| `p95_px` | 95th percentile of the error |
| `track_rms` | RMS after dropping the first 1/3 (steady state) |
| `first_acq_s` | time until the error first ≤ `aim.settle_px` |
| `on_target` | fraction of visible ticks with the crosshair inside the true box |
| `recovery_s` | average time from a >100 px peak back to <10 px (`_recovery`) |
| `jitter_px` | standard deviation of the output magnitude in the second half |
| `hit_rate` | simulated fire hits / fire count |

### 5.3 Regression gate

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli bench
```

`cli.py::cmd_bench`:

- runs `legacy` (v1 equivalent: 60 Hz, no tracker) and `predictive` on each of the 6 scenarios;
- the gate is end-to-end quality, not steady-state dither:
  - `new.rms_px > old.rms_px * 1.02`, or
  - `new.on_target < old.on_target - 0.01`, or
  - `new.jitter_px > old.jitter_px + 1.0`;
- any failure → prints `failures` and returns 2; all pass → 0.

That is also why `peek`/`jitter` can degrade in `track_rms` and still pass the gate: the gate looks at RMS including capture transients, the hit window and jitter, not only steady-state dither. `track_rms` is still used for diagnostics in `sim`/`autotune`.

Note that when `cmd_bench` calls `compare()` it **neither reads `--settings` nor passes `aim_cfg`**, so it uses the `AimCfg()` defaults. It verifies "the default control-law behaviour", not your profile; to verify a profile use `aim2 sim` (which reads `settings.aim`), or call `simulate(..., aim_cfg=load().aim)` from Python.

The reported values in `docs/MEASUREMENTS.md` §4 (same simulation; current `bench` output may differ slightly due to config/code tweaks, trust the command):

| Scenario | Controller | rms_px | track_rms | on_target | recovery_s | jitter_px |
|---|---|---|---|---|---|---|
| `strafe_mid` | legacy | 22.48 | 21.58 | 0.53 | nan | 3.75 |
| `strafe_mid` | predictive | 10.64 | 9.60 | 0.99 | nan | 1.88 |
| `strafe_far` | legacy | 60.31 | 23.55 | 0.34 | 0.27 | 3.93 |
| `strafe_far` | predictive | 45.14 | 8.70 | 0.96 | 0.12 | 1.96 |
| `jitter` | legacy | 17.19 | 3.07 | 0.97 | 0.15 | 1.44 |
| `jitter` | predictive | 14.14 | 4.68 | 0.98 | 0.06 | 0.82 |
| `dash` | legacy | 56.04 | 64.22 | 0.90 | 0.16 | 10.51 |
| `dash` | predictive | 55.23 | 62.67 | 0.93 | 0.09 | 3.12 |
| `peek` | legacy | 45.87 | 2.05 | 0.93 | 0.25 | 0.99 |
| `peek` | predictive | 38.04 | 8.50 | 0.94 | 0.13 | 1.56 |
| `micro_drift` | legacy | 10.29 | 2.48 | 0.99 | nan | 1.22 |
| `micro_drift` | predictive | 8.72 | 3.59 | 0.99 | nan | 0.74 |

How to read it:

- the `on_target` and `recovery` of mid range/long range/flicks improve markedly;
- `peek`'s `track_rms` actually grows (2.05 → 8.50), coming from `policy.coast_s=0.18 s` extrapolation: useful for real occlusion, over-extrapolates on long disappearances. To be more conservative lower `coast_s` to 0 or 0.08 and check `first_acq_s`.
- `jitter`'s `track_rms` is slightly larger (3.07 → 4.68) but `jitter_px` is smaller; if live hardware jitters at rest, tune `deadzone`/`output_ema` first and do not look at this one metric alone.

### 5.4 Replay

```bash
# record (live hardware/video)
PYTHONPATH=src .venv/bin/python -m dma.cli record --duration 60 --dry-run --no-hotkeys

# summary
PYTHONPATH=src .venv/bin/python -m dma.cli replay out/telemetry/session_x.jsonl.gz --summary

# compare two controllers
PYTHONPATH=src .venv/bin/python -m dma.cli replay out/telemetry/session_x.jsonl.gz --compare
```

Limitations (must know):

- when `cmd_replay` calls `replay(session, kind)` it **does not pass `aim_cfg`**, so it uses the `AimCfg()` defaults, not the current `configs/settings.json`. To use the current configuration, call `telemetry.replay.replay(..., aim_cfg=load().aim)` from Python.
- `telemetry/replay.py` hardcodes the `cls_name` of every detection to `"body"`, so head-class fusion cannot be reproduced; it is good for comparing control laws, not for verifying head locking.
- the replay centre is fixed at `(640,360)`, which only fits 1280×720 sessions; other resolutions will be off.

### 5.5 Recommended verification flow

1. `aim2 config migrate` to generate a baseline, `cp configs/settings.json` to back up.
2. `aim2 bench` to confirm the baseline passes.
3. Change one parameter (`config set`).
4. `aim2 bench`: must not be worse than the baseline; roll back on failure.
5. `aim2 sim --compare` to look at single-scenario details.
6. `aim2 replay SESSION --compare` to re-check against real trajectories (mind the default AimCfg limitation in §5.4).
7. `aim2 run --dry-run` for 20–30 seconds on live hardware, watch `aim.err_px`, `aim.saturated`, `fire.reason`.
8. Turn fire off first before firing on live hardware, confirm the aim is stable; then enable `fire.mode=single`.
9. Note down the `config diff` and the results in the change log.

---

## 6. Symptom → parameter quick reference

| Symptom | First choice | Second choice | Avoid |
|---|---|---|---|
| Flicks too slow | `max_step` ↑, `kp_far` ↑ | `far_px` ↑ | adding `ki` first |
| Jitter near the target | `kd` ↑, `deadzone` ↑ | `output_ema` ↓, `kp_near` ↓ | raising `lead_gain` |
| Tracking lag | `lead_gain` ↑, `ki` ↑ | `integral_gate` ↑ | blindly raising `kp_far` |
| Flick rebound | `max_step` ↓, `kp_far` ↓ | `lead_max` ↓, `integral_gate` ↓ | raising `kd` to mask it |
| Wavering between targets | `switch_margin` ↑, `fov_radius` ↓ | `coast_s` ↓, `tracker.iou_thr` ↑ | lowering `min_hits` at the same time |
| Pulls toward teammates | switch to the sjz model, verify `ignore_classes` | `policy.min_hits` ↑ | fixing it with `head_ratio` |
| Cannot hit the head | with a head class check fusion/`head_bonus`; without one recalibrate `head_ratio` | `fire.inset` ↓ | enlarging `fire.radius` to mask it |
| No fire | check `fire.reason` | `fire.radius`/`inset`/`dwell` | going straight to `mode=auto` |
| Jitters at rest | `deadzone`/`deadzone_exit` ↑, `output_ema` ↓ | `tracker.meas_noise` ↑ | lowering `max_step` too much |
| Cannot follow fast direction changes | `tracker.accel_noise` ↑ | `kp_far` ↑ (carefully) | only raising `lead_gain` |
| Over-extrapolates after losing the target | `policy.coast_s` ↓ | `aim.lead_gain` ↓ (already ×0.4 while coasting) | turning the tracker off |

---

## 7. Change log template

Leave one entry per tuning session, in `out/` or in the PR description:

```markdown
### YYYY-MM-DD change title
- Config: configs/settings.json (profile=...)
- diff:
  - aim.kp_far: 0.85 -> 0.75
  - policy.coast_s: 0.18 -> 0.10
- Reason/symptom:
- Verification:
  - `aim2 bench`: pass (failures=[])
  - `aim2 replay SESSION --compare`: predictive track_rms ... (note the default AimCfg limitation)
  - 60s on live hardware: latency loop_ms=..., confidence=..., aim.err_px p50/p95=..., fire.reason distribution=...
- Rollback: cp configs/settings.bak.json configs/settings.json
```

---

This document corresponds to schema_version 2 and the src/dma layout; please update it when the code changes.
