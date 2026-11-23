<div align="right">

[简体中文](zh-CN/MEASUREMENTS.md) | **English**

</div>

# Measured data and hard constraints

> ⚠️ **For learning, research and engineering study only.**
> Do not use this project in any competitive or ranked setting. Automated aiming
> violates the terms of service of most online games and may lead to permanent bans.
> You are solely responsible for how you use it.

> This file records only "things that were measured": bench measurements, closed-loop simulation, reproducible experiments.
> Every entry states how it was measured, so a reader can reproduce or refute it with the same commands.
> Unit conventions: px = captured-frame pixels, counts = mouse counts (HID report values).

---

## 1. Hard link constraints (design drivers, not something tuning can change)

| # | Conclusion | How it was obtained | Impact on the design |
|---|---|---|---|
| 1 | **A game in exclusive fullscreen locks HDMI output to 60 Hz** | 120 Hz on the desktop, but the capture card only outputs 60 frames once the game is running | sampling rate is capped at 60 fps, so the control strategy must be designed for "low sampling + high latency" |
| 2 | The capture card has 1–5 frames of inherent signal latency, and **a resolution mismatch adds another 1~2 frames** | switch the capture resolution and compare the response time of the same scene | `capture.width/height` must match the game's native resolution |
| 3 | Inference is **not** the bottleneck | 320×320 export p50 4–5 ms (see §3) | put the optimisation effort into latency compensation rather than model acceleration |
| 4 | Serial **is not** the bottleneck | a 9-byte frame at 921600 bps ≈ 9400 frames/s, no backlog measured | the report rate can go up to 1000 Hz |
| 5 | End-to-end latency is on the order of 30 ms (≈2 frames) | closed-loop online identification: minimising the `Δe_k + u_{k-m}` residual, m≈8 ticks @240 Hz | latency compensation is mandatory; the control gain ceiling is pinned by latency |
| 6 | At 60 fps the physical limit of "flick + settle" is about 10 frames (167 ms) | simulation bench and live measurement agree | raising the frame rate is the only step change; everything else only approaches that limit |

## 2. Why not a "high-gain PID"

On an integrating plant with D frames of delay, the stability condition for pure proportional control is roughly `kp·D < 2`;
at 60 fps with D≈2, the visual feel demands `kp > 1`, which necessarily leads into a limit cycle: the crosshair sweeps back and forth across the target.

Reproduction experiment (`tests/test_control.py::test_sim_survives_a_wrong_apply_window`):

| Command-application window assumed by the controller | Tracking error RMS with a real latency of 33 ms | Hit-window share |
|---|---|---|
| 2.5 ms | **304 px** | **1 %** |
| 10 ms | 10.7 px | 99 % |
| 16.5 ms (correct value) | 11.7 px | 99 % |

Conclusion: **for every factor of two the latency estimate is wrong, the effective loop gain is wrong by the same factor.** That is why
`control/latency.py` has a physical floor (≥1.2 frames) and three rejection conditions, instead of "use whatever was estimated".

## 3. Inference latency (CoreML on this machine, 2560×1600 input frames, `aim2 models bench`)

| Model output format | Input | p50 | p95 | 60 fps budget headroom |
|---|---|---|---|---|
| YOLOv8 `[1,9,2100]` | 320×320 | 4.1 ms | 7.9 ms | +8.8 ms |
| End-to-end `[1,300,6]` | 320×320 | 5.0 ms | 6.3 ms | +10.4 ms |
| YOLOv5 `[1,25200,7]` | 640×640 | 7.4 ms | 8.2 ms | +8.5 ms |
| End-to-end `[1,300,6]` | 640×640 | 16.6 ms | 20.6 ms | **−3.9 ms (dropped frames)** |
| End-to-end `[1,300,6]` | 416×416 | 13.9 ms | 14.3 ms | +2.4 ms (exceeded as soon as it heats up) |

The 60 fps budget is 16.7 ms per frame. **Only the 320-level exports keep headroom**; 640 starts dropping frames.

## 4. Control law comparison (closed-loop simulation: 33 ms latency, 2.5 px detection noise, 60 fps capture)

The `legacy` control group = the classic "PID on the delayed error", forced to 60 Hz control with no tracking (`dma/control/legacy_pid.py`).

| Scenario | Metric | legacy | this implementation |
|---|---|---|---|
| Mid-range sweep | RMS / hit window | 22.5 px / 53 % | **10.6 px / 99 %** |
| Long-range sweep | RMS / hit window | 60.3 px / 34 % | **45.1 px / 96 %** |
| Long-range sweep | recovery time | 0.27 s | **0.12 s** |
| Flick (500 px teleport) | recovery / jitter | 0.16 s / 10.5 px | **0.09 s / 3.1 px** |
| Peek (repeatedly disappearing) | first capture | 0.27 s | **0.14 s** |
| Near-static jitter | output jitter | 3.8 px | **1.9 px** |

Reproduce: `aim2 sim --compare` / regression gate: `aim2 bench` (returns non-zero as soon as any metric regresses).

## 5. Resources and thermals (same fanless laptop, 320×320 end-to-end model)

| Configuration | Per-frame latency | Per-frame CPU time | Sustained CPU usage |
|---|---|---|---|
| CoreML, `intra_threads=2` | 6.32 ms | 13.47 ms | **213 %** |
| CoreML, `intra_threads=1` | 6.40 ms | 7.17 ms | **112 %** |
| CPU, 1 thread | 14.57 ms | 16.93 ms | 116 % |

Conclusion: **more threads do not shorten latency, they just burn one more core.** On a fanless machine that is the direct cause of throttling.
The UI preview is the same story: after replacing 30 fps full-resolution smooth scaling with "downscale first, then paste + configurable frame rate",
UI + engine together went from about 3/4 of a core to about 1/2.

## 6. Head point for body-only models

Models with only a body class (`st` / `body`) have no `head`, so head locking can only use
`aim_point_y = box.y1 + head_ratio · box.h`:

- it must be **calibrated on site against the red crosshair in the live picture**; different resolutions and field-of-view ratios differ;
- the common range is 0.08–0.15; too large hits the neck, too small misses;
- this is a limitation of the model's capability, not something control parameters can compensate.

## 7. Acceptance criteria

At the end of a match with enemies, the session check-up on exit should satisfy:

| Metric | Pass line | Note |
|---|---|---|
| `error p50` | < 8 px | everyday distance between the crosshair and the aim point |
| `error p95` | < 15 px | convergence quality after flicks/target switches |
| `latency` (latency) | 25–60 ms with conf > 0.3 | below 20 ms is physically impossible |
| fire reason | `off-target` should not account for more than 90 % | otherwise the lock is not holding or the radius is too tight |
