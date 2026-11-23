<div align="right">

[简体中文](DISCLAIMER.zh-CN.md) | **English**

</div>

# Disclaimer

**This project is for learning, research and engineering study only.**

## What this means in practice

- **Do not use it in any competitive, ranked, matchmade or score-bearing setting.**
  Automated aiming typically violates the terms of service of online games, can lead to
  permanent bans, and degrades the experience of other players.
- Run it only on hardware and in environments **you own or are explicitly authorised to use**.
- The project does exactly two things: it reads frames from your own capture device, and it
  sends HID reports to your own machine. It does **not** read game memory, inject into
  processes, or attempt to defeat anti-cheat systems.
- The authors and contributors accept no liability for bans, account loss, data loss,
  hardware damage or legal consequences.
- Model weights are not authored by this project; see [NOTICE](NOTICE) for their terms.

## If you are here to study the engineering

Start with the parts that never touch a game — they need no capture card and no game:

```bash
python -m dma.cli sim --compare     # closed-loop control-law comparison
pytest -q                           # 79 unit and integration tests
```

Every number in [docs/MEASUREMENTS.md](docs/MEASUREMENTS.md) is reproducible with in-repo
commands. The interesting problems here are latency compensation, state estimation and
rate-normalised control — none of which require aiming at anyone.

## Reporting misuse

This is a source-available engineering project under GPL-3.0. If you redistribute it, keep
this disclaimer and the [NOTICE](NOTICE) file intact.
