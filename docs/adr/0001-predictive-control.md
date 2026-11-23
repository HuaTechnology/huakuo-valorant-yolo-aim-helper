<div align="right">

[简体中文](../zh-CN/adr/0001-predictive-control.md) | **English**

</div>

# ADR-0001: Replacing pure PID with predictive control

> ⚠️ **For learning, research and engineering study only.**
> Do not use this project in any competitive or ranked setting. Automated aiming
> violates the terms of service of most online games and may lead to permanent bans.
> You are solely responsible for how you use it.

- Status: Accepted
- Date: 2026-11-21
- Decision scope: `src/dma/control/controller.py`, `src/dma/control/latency.py`, `src/dma/engine/runtime.py`
- Related: ADR-0002 (world-frame tracking), [LATENCY_BUDGET](../LATENCY_BUDGET.md), [TUNING](../TUNING.md)

---

## 1. Background

The v1 control law was "PD + anti-windup integral + distance-adaptive EMA + piecewise kp":

- `aim.py::AimController.update` runs PID on **stale screen error**;
- gains were hand-tuned in the 60 Hz main loop, there was no notion of "control frequency";
- latency relied on the heuristics in `predict.py`, and was often disabled by default;
- the serial port sends in blocks of 127 and **drops the remainder**, so a 4 px dead band had to be kept to avoid jitter.

Phenomena recorded in docs/MEASUREMENTS.md:

| Symptom | v1 explanation |
|---|---|
| High gain → "spring" oscillation | latency + high gain → overshoot limit cycle (docs/MEASUREMENTS.md) |
| Low gain → tracking lag | proportional control on stale error, steady-state lag ∝ velocity/gain |
| Retune after changing capture/control frame rate | gain had a "per frame" dimension, not "per second/per apply window" |
| Rebound after a flick | integral windup + no feedforward/latency compensation |
| Measured D=1 frame, but unusable | latency estimation untrustworthy, prediction off by default (`PREDICT_FRAMES=0` in `config.py`) |

The v1 code comments also contain two simulation conclusions (`aim.py` docstring):

```
D=2: initial overshoot ≈13px, tracking lag ≈8.7px
D=3: initial overshoot ≈44px, tracking lag ≈8.4px
```

This shows: **with the same gain, going from 2 to 3 frames of latency changes the overshoot from 13 px to 44 px**. Pure PID has no stable operating point when latency changes, so it can only be hand-tuned over and over.

---

## 2. Decision

Model the plant explicitly as a **delayed integrator** and design the control law around that model, instead of patching PID further.

### 2.1 Plant

```
e(t) = p(t - L_measure) - c(t - L_apply)
```

- `p`: target position (screen frame).
- `c`: the displacement we have cumulatively sent (screen frame).
- `L_measure`: frame produced → detection available.
- `L_apply`: command issued → visible on screen/crosshair.
- `L = L_measure + L_apply`: total closed-loop latency, identified online by `LatencyEstimator`.

`Engine.step` splits `L` into:

```python
age_s   = det_age + split * loop_s          # extrapolate the target to "now"
apply_s = max(dt, (1 - split) * loop_s)     # command apply window
lead_s  = loop_s + det_age                  # fire preview
```

### 2.2 Control law

`PredictiveAim.step`:

```
scale = clamp(dt / apply_s, 0.02, 1.0)
kp    = kp_near + (kp_far - kp_near) * smoothstep(near_px, far_px, dist)
ff    = clamp(v * apply_s * lead_gain, ±lead_max)     # ×0.4 while coasting
u     = scale * (kp*e + ki*I + ff) + kd*(e - e_prev)
u     = slew_limit(u)
u     = clamp_magnitude(u, max_step * scale)
I    += e*dt (only when dist < integral_gate), clamp to integral_limit
when saturated, I ceiling = (max_u - |PD+FF|) / ki
```

Five design points:

1. **Velocity feedforward sits outside kp**. Feedforward compensates "how far the target moves on its own within `apply_s`", and should not be scaled by the proportional gain; otherwise a purely proportional loop has a steady-state lag of `v*apply/kp` on a constant-velocity target, which is exactly the source of v1's "tracking lag".
2. **Feedforward clamping** `lead_max=70 px`. The velocity estimate comes from the Kalman filter, and a bad frame/instantaneous misdetection may produce a huge `v`; clamping guarantees one misjudgement does not become a 600 px overshoot. While coasting the feedforward is multiplied by another 0.4, because extrapolated velocity is less trustworthy than measured velocity.
3. **Rate normalisation** `scale = dt/apply_s`. The control law's semantics are "how many pixels to move within the command apply window", not "per tick". Without `scale`, a 240 Hz loop moves 4× as far per unit time as a 60 Hz loop and must oscillate under latency — this is exactly the root cause of v1 gains "only working at 60 fps".
4. **Gain scheduling**: far `kp_far` (default 0.85) is close to dead beat, near `kp_near` (default 0.50) is more conservative, because near errors come more from detection noise/latency estimation error than from real target motion. The transition uses `smoothstep(26,260)` to avoid the hard switching of v1's piecewise kp.
5. **Anti-windup by headroom projection**, not a simple clamp: when saturated the integrator may only keep "output budget minus what P/D/FF already consumed". v1's `integral_limit` was a fixed 100 px, so the integral kept accumulating during a flick and rebounded afterwards.

### 2.3 Latency identification

`LatencyEstimator` performs one-dimensional system identification **on detection events**:

```
cost(m) = median_k |Δe_k + u_{k-m}|      (L1 sum over the two dimensions)
m*      = argmin cost(m)
conf    = 1 - cost(m*) / median(cost)
```

- candidates `m ∈ [0,10]` frames, window of 45 detection samples.
- only when `conf >= latency_min_conf` (default 0.2) is `loop_s` updated with smoothing (`smoothing=0.25`).
- when the target is stationary/we are not moving, the error curve is flat, `conf` is low, and the prior is kept — this is intentional, no "identification without excitation".

Why not a per-stage timer: `perception.infer_ms`/`transport.write_ms` only see the 5–15 ms stages; the truly dominant capture card/display/game input tick is invisible from the host and can only be estimated in the closed loop.

### 2.4 Keeping legacy

`aim.controller="legacy"` can still switch to `LegacyAim` (wrapping the v1 `AimController`):

- used for A/B, replay comparison, and degraded operation as "v2 engine + v1 control law";
- it is not a full v1 rollback (world frame/threads/protocol are still v2), see [OPERATIONS §7](../OPERATIONS.md#7-rolling-back-to-v1).

---

## 3. Rejected options

| Option | Why rejected |
|---|---|
| **Pure PID with higher kp to force it** | no stable operating point when latency changes; v1's D=2→3 overshoot going 13→44 px is the evidence. High kp limit-cycles under latency, low kp lags in steady state, you cannot have both |
| **Full Smith predictor** | needs an exact model/queue state; feeding "sent but not yet applied displacement" back into the prediction amplifies detection jitter. The `predict.py` docstring records the v1 measurement: the full Smith predictor jittered a lot, clamped feedforward cut the lag from 9 px to 3–4 px while jitter stayed at baseline. v2 further removed the output queue (`MotionPlanner` only keeps an accumulator), so Smith's "pending queue" premise no longer holds |
| **Position dead-band compensation** | using a dead band to hide quantisation error costs a permanent 4 px dead band up close (v1's `deadzone=4.0`). v2 has `MotionPlanner` carry the remainder, so sub-pixel error accumulates up to 1 count, and the 1.5 px dead band is used for "hold", not for "hide" |
| **Fixed-frame prediction** (v1 `PREDICT_FRAMES`) | frames are not a physical quantity; after 60→120/240 Hz the same "2 frames" means something completely different. v2 uses seconds + online identification with `lead_max` clamping |
| **MPC / LQR** | the plant is a one-dimensional delayed integrator, and model error comes mainly from latency estimation and detection noise, not from high-order dynamics; solving a QP within the 4.17 ms control budget brings no benefit and costs a lot to validate. Coordinate descent over 6 parameters + closed-loop simulation already covers it |
| **Learning-based control (RL/imitation)** | unexplainable, non-replayable, safety boundaries hard to guarantee; not put into the real-time loop before there is enough real data |

---

## 4. Consequences

### 4.1 Benefits

- when latency changes you only need the `loop_s` estimate to be trustworthy, gains do not need retuning with the frame rate.
- simulation metrics for mid range/long range/flicks improve markedly (`docs/MEASUREMENTS.md` §4): `strafe_mid` hit window 0.53→0.99, `strafe_far` 0.34→0.96, `dash` recovery 0.16→0.09 s.
- `peek`/`jitter` degrade in steady-state `track_rms` (2.05→8.50, 3.07→4.68), caused by `coast_s` extrapolation and weaker output smoothing; this is a known cost, see [TUNING §5.3](../TUNING.md#53-regression-gate) for tuning.

### 4.2 Costs/risks

| Risk | Mitigation |
|---|---|
| Wrong latency estimate → feedforward/extrapolation in the wrong direction | `confidence` threshold + prior; `lead_max` clamping; do not raise `kp_far` at low confidence |
| Wrong world-frame scale → wrong velocity estimate | mandatory `output.gain` calibration ([CALIBRATION §3](../CALIBRATION.md#3-sensitivity-and-gain-calibration)) |
| Near-range noise → jitter | dead-band hysteresis + `output_ema` + low `kp_near`; do not force it down with higher `kd` |
| Integral windup | conditional integration + headroom projection + `controller.reset()` on target switch |
| Residual state after a target switch | `Engine.step` resets the controller when `target.tid` changes (new in v2) |
| Parameter semantics changed | `legacy` kept; `aim.*` fields have `FIELDS` help text; `config diff` auditing |

### 4.3 Impact on testing/operations

- `LatencyEstimator` + world frame + calibration flow are required; it is not "just change one kp".
- `aim2 bench` turns "the predictive law must not be worse than v1" into a gate; `aim2 sim/replay` provide reproducible comparisons.
- during live tuning any `Engine.apply` rebuilds the controller (the `_kind` check problem) and clears integral/EMA; see [OPERATIONS §6.3](../OPERATIONS.md#63-aimingcontrol).

---

## 5. Evidence and code locations

| Conclusion | Location |
|---|---|
| Delayed-integrator model | `control/controller.py` module docstring |
| Rate normalisation | `scale = clamp(dt/apply_s, ...)` in `PredictiveAim.step` |
| Feedforward outside kp + clamping | `ff_x/ff_y` in `PredictiveAim.step` |
| Conditional integration + headroom projection | `integral_gate` / `headroom` in `PredictiveAim.step` |
| Gain scheduling | `smoothstep(near_px, far_px, dist)` in `PredictiveAim.step` |
| Latency identification | `LatencyEstimator._estimate` |
| Reset on target switch | the `_ctrl_tid` branch in `Engine.step` |
| v1 simulation conclusions | `src/dma/aim.py` docstring (overshoot/lag at D=2/3) |
| v1 Smith feedforward conclusions | `src/dma/predict.py` docstring |
| Closed-loop comparison | `docs/MEASUREMENTS.md` §4, `sim/harness.py` |

---

## 6. When to re-evaluate

- Capture goes from 60 fps to 120/240 fps: rerun `aim2 bench` and confirm the `kp` schedule is still appropriate (rate normalisation keeps the dimension unchanged, but the detection noise spectrum changes).
- Variable latency/jitter appears (WiFi serial, USB hub, unstable game input tick): consider adaptive gain based on `confidence`, or add a variance penalty to `loop_s`.
- After introducing models with head/teammate, target selection/fusion error becomes the dominant error source: the control law is not the bottleneck, do not add gain.
- If the exact game input tick timing becomes available in the future (driver/API), `L_apply` can change from an estimate to a measurement, and `latency_split` can be fixed at the physical ratio.

---

This document corresponds to schema_version 2 and the src/dma layout; please update it when the code changes.
