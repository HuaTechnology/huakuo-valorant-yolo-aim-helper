<div align="right">

**简体中文** | [English](CONTRIBUTING.md)

</div>

# 贡献指南

## 环境

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[ui,hotkeys]" -r requirements-dev.txt
```

## 提交前必须通过

```bash
pytest -q
ruff check .
python -m dma.cli bench        # 闭环回归门禁：新控制律的相对指标不得退化
```

`bench` 失败意味着某个场景的跟踪误差、命中窗口或抖动比基线差，
请先解释原因（新场景？指标定义变更？），再改门禁阈值。

## 改动约定

- **`core/config.py` 是唯一配置来源**：新增参数要同时加到 dataclass、`FIELDS`
  注册表（范围/单位/说明）与迁移逻辑，UI 会自动出现。
- **坐标系统一**：source / model / world / screen 四套，任何跨层传递都要写明是哪套。
  见 `docs/ARCHITECTURE.md` §4。
- **控制律改动要有数字**：先在 `aim2 sim --compare` 上看变化，再提交；
  纯理论推导不算证据。
- **不要引入阻塞控制线程的 I/O**：串口、采集、推理各有线程，控制线程只做标量运算。
- **提交信息**：`<scope>: <what>`，例如 `control: 修正速率归一化的 apply 窗口下限`。

## 新增模型格式

1. 在 `perception/decoders.py` 写解码函数（输入原始输出，输出 xyxy/score/cls）。
2. 在 `infer_format()` 里加形状判定，并提供纯 numpy 单元测试（`tests/test_perception.py`）。
3. 用真实帧跑 `aim2 models inspect <path>` 确认框对齐。

## 报告问题

请附上：`aim2 doctor --probe` 输出、会话文件（`out/telemetry/session_*.jsonl.gz`）、
以及 `docs/MEASUREMENTS.md` 里对应的指标读数。没有数据的「感觉不准」很难定位。
