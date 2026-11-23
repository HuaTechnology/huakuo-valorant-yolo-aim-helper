<div align="right">

[简体中文](CHANGELOG.zh-CN.md) | **English**

</div>

# Changelog

All notable changes are documented here. Format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versioning follows
[SemVer](https://semver.org/).

## [2.0.0] - 2026-11-23

First public release: the single-file script with a hand-tuned PID became a pipeline that
can be measured, replayed and regression-tested.

### Added
- Layered architecture: `core / capture / perception / control / transport / engine / sim /
  tuning / telemetry / desktop`
- Decoders for three detection heads (end-to-end / YOLOv5 / YOLOv8) with per-class NMS and a
  class allow-list
- Kalman multi-object tracking (ID, world-frame velocity, position covariance, coasting)
- World frame plus a delay-sampled camera line; online latency identification with a
  physical floor and three rejection gates
- Velocity feed-forward + rate-normalised control law (same gains at 60 and 240 Hz)
- Protocol v2: CRC8, sequence numbers, device STATUS telemetry; sub-frame motion planner that
  carries the remainder
- Engine: four threads, immutable snapshots, health watchdog, hot reload
- Closed-loop simulation bench and regression gate (`aim2 bench`); offline auto-tuning
  (`aim2 autotune`)
- Session recording, closed-loop replay, `aim2 selftest` / `link-test` / `calibrate`
- PySide6 console whose UI is generated from the config registry
- ESP32-S3 firmware: 1000 Hz HID, CRC, deadman, STATUS telemetry
- English docs under `docs/` with a Chinese mirror under `docs/zh-CN/`

### Fixed (relative to the internal development build)
- The latency estimate could collapse to a physically impossible 2.5 ms, driving a delayed
  integrator into a limit cycle
- The UI refresh pushed the operator's toggles back to their previous state (signal loop)
- Migrating an old config copied control gains that belong to a different control law
- The firmware counted `Mouse.move()` calls instead of delivered reports, so a disconnected
  USB port still looked healthy

### Known limits
- Body-only models still need a manual head ratio and cannot identify teammates
- The visual path carries roughly two frames of inherent latency
- Latency identification has no observable signal while the target is static

### Release assets
- `7-wa-v26s-320x.onnx` (~36 MB): the author's 320×320 end-to-end example model. Body class
  only (`st`); head aim relies on `policy.head_ratio`. For personal study only — do not
  redistribute, do not use in competitive settings.
