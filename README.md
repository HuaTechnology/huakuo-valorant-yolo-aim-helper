<div align="right">

[简体中文](README.zh-CN.md) | **English**

</div>

<div align="center">

> ⚠️ **For learning, research and engineering study only.**
> Do not use this project in any competitive or ranked setting. Automated aiming
> violates the terms of service of most online games and may lead to permanent bans.
> See [DISCLAIMER](DISCLAIMER.md).

</div>

# huakuo · Valorant YOLO Aim Helper

> Turn "a target on screen" into a measurable, replayable, regressible real-time
> control problem. Capture card → ONNX detection → Kalman tracking →
> delay-compensated control → ESP32 USB HID.

[![ci](https://github.com/HuaTechnology/huakuo-valorant-yolo-aim-helper/actions/workflows/ci.yml/badge.svg)](https://github.com/HuaTechnology/huakuo-valorant-yolo-aim-helper/actions/workflows/ci.yml)
[![license: GPL-3.0](https://img.shields.io/badge/license-GPL--3.0-blue.svg)](LICENSE)
[![python](https://img.shields.io/badge/python-3.10%2B-3776ab.svg)](pyproject.toml)

```
┌──────────────┐  HDMI   ┌───────────────┐   USB    ┌──────────────────────────┐
│ Game PC      │────────▶│ Capture card  │─────────▶│ Inference host (macOS)   │
│              │         │  (UVC)        │          │ capture 60fps → detect → │
└──────▲───────┘         └───────────────┘          │ track → control → UART   │
       │ USB HID                                     └────────────┬─────────────┘
       │                                                          │ UART 921600
       │              ┌───────────────────────────────────────────▼──────────┐
       └──────────────│ ESP32-S3: protocol frames → 1000 Hz HID + telemetry   │
                      └──────────────────────────────────────────────────────┘
```

**This is not a DMA hardware build.** It is purely visual: it reads HDMI frames and
writes HID reports. The directory name `dma` in the original project was historical.

The hard part of a visual aim loop is not detection — it is the *closed loop*: 60 fps
sampling, ~30 ms latency, and an unknown count-to-pixel scale. A PID on the delayed
error either oscillates or lags. This repo splits the loop into six testable layers:

| Layer | Key idea |
|---|---|
| Config | Schema v2, a single field registry driving both validation and UI, migration, atomic writes, hot reload |
| Capture | Dedicated thread keeping only the newest frame; reconnect; staleness watchdog |
| Perception | Three output heads (end-to-end / YOLOv5 / YOLOv8) auto-detected; class allow-list; optional ROI; Kalman multi-object tracking (ID, world-frame velocity, covariance) |
| Control | World frame + delay-sampled camera line; **online latency identification**; **velocity feed-forward + rate normalisation**; anti-windup; dead-band hysteresis |
| Output | Protocol v2 (CRC8 / sequence / device telemetry); rate-limited accumulator that **carries the remainder** so sub-pixel error converges |
| Observability | JSONL logs, per-stage percentiles, session recording (gzip JSONL) + closed-loop replay, simulation bench, offline auto-tuning |

## Quick start

```bash
git clone https://github.com/HuaTechnology/huakuo-valorant-yolo-aim-helper
cd huakuo-valorant-yolo-aim-helper
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[ui,hotkeys]"

python -m dma.cli doctor --dry-run           # environment check (no hardware needed)
python -m dma.cli run --sim --aim --dry-run  # synthetic scene, full pipeline
python -m dma.cli bench                      # closed-loop regression gate
pytest -q                                    # 79 tests
```

On real hardware:

```bash
cp configs/settings.example.json configs/settings.json
# put your model in models/ and point vision.model at it
python -m dma.cli doctor                     # classes / serial port / inference benchmark
python -m dma.cli selftest --seconds 8       # 8-second closed-loop self test with a verdict
python -m dma.cli run --aim                  # run (space = aim, F = fire)
python -m dma.cli gui --aim                  # desktop console
```

## Models are not shipped

Weights are **not** committed to git. The author's 320×320 end-to-end export is attached to
the [latest release](https://github.com/HuaTechnology/huakuo-valorant-yolo-aim-helper/releases/latest) —
drop it into `models/`. Any other model works too; the detector auto-detects all three heads:

| Output tensor | Family | Decoding |
|---|---|---|
| `[1, N, 6]` | end-to-end export (NMS baked in) | `x1,y1,x2,y2,conf,cls` |
| `[1, 25200, 5+nc]` | YOLOv5 | objectness × class + per-class NMS |
| `[1, 4+nc, 2100]` or transposed | YOLOv8 | anchor-free, no objectness |

Classes come from the model metadata `names`; `vision.target_classes` is the
allow-list and `vision.ignore_classes` (e.g. `teammate`) can never become a target.

## Calibration (three steps, all required)

1. **Head point** — models with a `head` class are aimed automatically. Body-only
   models aim at `box.y1 + head_ratio · box.h`; drag `policy.head_ratio` until the
   red cross in the live preview sits on the head (typically 0.08–0.15).
2. **Counts per pixel** — `aim2 calibrate gain` measures how many mouse counts move
   the view by one screen pixel, then write it back to `output.gain`.
3. **Latency** — run `aim2 selftest` and watch the HUD's measured latency and
   confidence; keep the prior while confidence stays below 0.3.

## Documentation

Docs index: [`docs/`](docs/README.md) (English, canonical) · [`docs/zh-CN/`](docs/zh-CN/README.md) (中文).

| File | Contents |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Layers, thread model, four coordinate systems, one-tick sequence |
| [`docs/LATENCY_BUDGET.md`](docs/LATENCY_BUDGET.md) | End-to-end latency budget, gain stability limit |
| [`docs/MEASUREMENTS.md`](docs/MEASUREMENTS.md) | Every measured number and hard constraint, with reproduction commands |
| [`docs/CALIBRATION.md`](docs/CALIBRATION.md) | Head point / counts-per-pixel / latency |
| [`docs/TUNING.md`](docs/TUNING.md) | What happens when each parameter goes up or down |
| [`docs/OPERATIONS.md`](docs/OPERATIONS.md) | Runtime, health states, troubleshooting table |
| [`docs/adr/`](docs/adr) | Decision records: control law, world-frame tracking |
| [`firmware/esp32_hid/`](firmware/esp32_hid/) | ESP32-S3 firmware, wiring, flashing |

## Layout

```
src/dma/
├── core/          config / types / metrics / logging / clock & delay line
├── capture/       sources (card / video / synthetic) + capture thread
├── perception/    geometry / decoders / ONNX detector / Kalman tracker
├── control/       latency ID / target policy / controller / fire / v1 baseline
├── transport/     protocol v2 / sub-frame planner / serial link
├── engine/        multi-threaded engine, snapshot, health watchdog
├── sim/           closed-loop bench / synthetic detector
├── tuning/        offline auto-tune / calibration
├── telemetry/     session recording and replay
├── desktop/       PySide6 console (UI generated from the config registry)
└── cli.py         `aim2` entry point
```

## Hardware and wiring (four parts, four cables)

| # | Part | Qty |
|---|---|---|
| 1 | HDMI capture card (UVC, 1280×720@60 MJPG) | 1 |
| 2 | HDMI cable | 1 |
| 3 | ESP32-S3 dev board (two ports: `USB` + `UART`) | 1 |
| 4 | USB Type-C **data** cable | 2 |

```
Windows ──HDMI──▶ capture card ──USB──▶ Mac        video: read-only
Mac ──Type-C──▶ ESP32 `UART` port                  control: protocol frames @921600
ESP32 `USB` port ──Type-C──▶ Windows                output: HID mouse only
```

The two hosts have **no network or file path between them**; the only shared device is
the ESP32 (UART in, HID out).

- **Inference host**: anything that runs ONNX Runtime (validated on macOS + CoreML)
- **Flashing**: the ESP32-S3 ROM download mode works over *both* ports, so flashing the
  firmware needs **no cable swapping** (`pio run -t upload` on the Mac, or BOOT+RST on
  Windows).

Details, board settings, flashing and troubleshooting:
[`firmware/esp32_hid/README.md`](firmware/esp32_hid/README.md).
Note: the ESP32-S3 ROM download mode works over **both** its ports, so flashing does
not require moving cables.

## Known limits

- The visual path carries ~2 frames of inherent latency; raising the frame rate is the
  only qualitative win (see `docs/MEASUREMENTS.md` §1).
- Body-only models have no head/teammate classes: head aim needs a ratio and team
  modes suffer from teammates.
- Latency identification has no observable signal when the target is static and the
  controller sits inside its dead band; the prior is kept in that case.
- The simulator is a model of the plant, not the game; it exists to compare control
  laws and to prevent regressions.

## Legal / responsible use

This is an engineering implementation of a real-time vision + control system, intended
for studying closed-loop computer vision, real-time systems, HID protocols and embedded
development.

- Use it only in environments you own or are authorised to use.
- Most online games forbid third-party assistance; you are responsible for the
  consequences of using this project.
- The authors accept no liability for bans, data loss or legal consequences.

If you only care about the control problem, `aim2 sim --compare` and
`docs/MEASUREMENTS.md` need no game at all — that is the recommended entry point.

## License

[GPL-3.0](LICENSE). Third-party components and trademark notes: [NOTICE](NOTICE).
