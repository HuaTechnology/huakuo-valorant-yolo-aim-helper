<div align="right">

[简体中文](zh-CN/README.md) | **English**

</div>

# Documentation (English)

> ⚠️ **For learning, research and engineering study only.**
> Do not use this project in any competitive or ranked setting. See the
> [disclaimer](../DISCLAIMER.md).

The Chinese originals live one level up (`docs/*.md`); these are faithful translations.
If the two disagree, the Chinese file is the source of truth and the English one is a bug.

| Document | What it answers |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | Layers, thread model, the four coordinate systems, exact one-tick sequence |
| [LATENCY_BUDGET.md](LATENCY_BUDGET.md) | Where the milliseconds go, and why the gain has a stability limit |
| [MEASUREMENTS.md](MEASUREMENTS.md) | Every measured number and hard constraint, with reproduction commands |
| [CALIBRATION.md](CALIBRATION.md) | Head point, counts-per-pixel, latency identification |
| [TUNING.md](TUNING.md) | What happens to the feel when each parameter goes up or down |
| [OPERATIONS.md](OPERATIONS.md) | Runtime, health states, log/metric fields, troubleshooting |
| [adr/0001-predictive-control.md](adr/0001-predictive-control.md) | Why velocity feed-forward + rate normalisation beat plain PID |
| [adr/0002-world-frame-tracking.md](adr/0002-world-frame-tracking.md) | Why tracking happens in the world frame, and how the delay line is sampled |

Firmware: [`../firmware/esp32_hid/README.md`](../firmware/esp32_hid/README.md).
