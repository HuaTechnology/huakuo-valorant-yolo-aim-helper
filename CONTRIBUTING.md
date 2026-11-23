<div align="right">

[简体中文](CONTRIBUTING.zh-CN.md) | **English**

</div>

# Contributing

## Setup

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[ui,hotkeys]" -r requirements-dev.txt
```

## Before you open a PR

```bash
pytest -q                      # 79 tests
ruff check .                   # lint
python -m dma.cli bench        # closed-loop regression gate
```

`bench` fails when a scenario's tracking error, on-target ratio or jitter gets worse than
the stored baseline. Explain why (new scenario? changed metric definition?) before touching
the threshold.

## Conventions

- **`core/config.py` is the single source of truth.** A new parameter needs three things:
  the dataclass field, a `FIELDS` registry entry (range / unit / help — the desktop UI is
  generated from it), and a line in the migration map if it replaces an old knob.
- **Four coordinate systems**: source / model / world / screen. Any value crossing a layer
  boundary must say which one it is. See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) §4.
- **Control-law changes need numbers.** Run `python -m dma.cli sim --compare` before and
  after and paste both tables. Pure reasoning is not evidence.
- **Never block the control thread with I/O.** Capture, inference and the serial link each
  own a thread; the control loop only does scalar maths.
- **Commit messages**: `<scope>: <what>`, e.g. `control: clamp the apply window to one frame`.
- **Language**: code, comments and user-facing strings are English. Docs are English in
  `docs/` with a Chinese mirror in `docs/zh-CN/`; keep both in sync in the same PR.

## Adding a model format

1. Write a decoder in `perception/decoders.py` (raw output → xyxy / score / class).
2. Teach `infer_format()` the tensor shape, and add a pure-numpy unit test in
   `tests/test_perception.py`.
3. Verify box alignment on a real frame with `python -m dma.cli models inspect <path>`.

## Reporting bugs

Attach `python -m dma.cli doctor --probe`, the session file
(`out/telemetry/session_*.jsonl.gz`) and the numbers from `docs/MEASUREMENTS.md` that look
wrong. "It feels off" cannot be debugged; a p95 error can.
