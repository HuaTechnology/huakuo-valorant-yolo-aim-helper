<div align="right">

[简体中文](SECURITY.zh-CN.md) | **English**

</div>

# Security policy

## Reporting

Please do **not** open a public issue for a security problem. Use the
[private vulnerability report](https://github.com/HuaTechnology/huakuo-valorant-yolo-aim-helper/security/advisories/new)
or open an issue that only asks for a private channel. Expect a reply within 48 hours.

## Attack surface

| Surface | Notes | Advice |
|---|---|---|
| Serial frame input | The firmware parses binary frames from the host; length and CRC are validated and the parser resynchronises | Do not attach untrusted devices |
| Session files | `out/telemetry/*.jsonl.gz` contains detection boxes and timestamps, which may indirectly reveal on-screen content | Scrub before sharing |
| Config / logs | May contain local paths, serial port names, model file names | Review before pasting into an issue |
| Model files | `models/*.onnx` come from third parties; parsing untrusted ONNX is a real risk | Only load models you are licensed to use and trust the source of |

## Do not commit

- GitHub tokens, serial device paths, internal hostnames (`.gitignore` covers the common
  build products, but not your judgement).
- `configs/settings.json` — it is generated locally; the repo keeps `settings.example.json`.
- Full session files in issues without checking what is inside them.

## Supported versions

Only the latest release (`v2.0.0`) is maintained. Please upgrade before reporting.
