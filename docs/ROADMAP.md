<div align="right">

[简体中文](zh-CN/ROADMAP.md) | **English**

</div>

# Roadmap

Items marked `help wanted` are open to PRs — see [CONTRIBUTING](../CONTRIBUTING.md).

> ⚠️ For learning, research and engineering study only. See [DISCLAIMER](../DISCLAIMER.md).

## Near term (self-contained)

- [ ] **English-first docs**: done — `docs/` is English, `docs/zh-CN/` mirrors it
- [ ] **Demo asset**: record 5 s of the synthetic scene (`aim2 run --sim`) as a GIF for the README
- [ ] **More output heads**: RT-DETR / YOLOv10 decoding plus shape detection (each with pure-numpy tests)
- [ ] **Issue template**: crash-log template (attach the tail of the JSONL session)

## Mid term (needs hardware or platforms)

- [ ] Windows-native capture (Desktop Duplication / DXGI) to remove the capture-card hop
      — the real latency win; see `docs/MEASUREMENTS.md` §1
- [ ] Higher frame rates: validate the control law at 120 fps capture (rate normalisation is
      already in place, measurements are missing)
- [ ] Linux UVC + V4L2 capture path with a CI smoke test
- [ ] Firmware: USB descriptor self-check, scripted flashing for the dual-port topology

## Long term (research)

- [ ] Multi-model fusion (body + head + teammate classes) with confidence weighting
- [ ] Adaptive latency estimation driven by Kalman innovations instead of the residual median
- [ ] Parameter self-tuning: Bayesian optimisation over **replayed real trajectories**

## Out of scope

- Memory reading, driver injection, anti-cheat evasion: this project only reads pixels and
  writes HID reports.
- Distributing model weights: the git tree never contains `.onnx` files.
