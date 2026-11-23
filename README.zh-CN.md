<div align="right">

**简体中文** | [English](README.md)

</div>

<div align="center">

> ⚠️ **仅供学习与研究。请勿用于任何竞技、排位或匹配场景。**
> 自动瞄准违反多数网络游戏的用户协议，可能导致永久封禁。
> 使用后果由使用者自行承担，详见 [DISCLAIMER](DISCLAIMER.zh-CN.md)。

</div>

# huakuo · Valorant YOLO Aim Helper

> 把「屏幕上的目标」变成一个可测量、可回放、可回归的实时控制问题。
> 采集卡 → ONNX 检测 → 卡尔曼跟踪 → 延迟补偿控制 → ESP32 USB HID。

```
┌──────────────┐  HDMI   ┌───────────────┐   USB    ┌──────────────────────────┐
│ 游戏主机/PC  │────────▶│   采集卡      │─────────▶│ 推理机 (macOS/Linux)     │
│              │         │  (UVC)        │          │ 采集 60fps → 检测 → 跟踪 │
└──────▲───────┘         └───────────────┘          │ → 延迟补偿控制 → 串口    │
       │ USB HID                                     └────────────┬─────────────┘
       │                                                          │ UART 921600
       │              ┌───────────────────────────────────────────▼──────────┐
       └──────────────│ ESP32-S3：协议帧 → 1000 Hz HID 鼠标 + STATUS 遥测     │
                      └──────────────────────────────────────────────────────┘
```

**这不是 DMA 硬件方案**（目录名 `dma` 是历史遗留）。它纯视觉：只读 HDMI 画面，只写 HID 输入。

视觉瞄准的难点不是检测，而是**闭环**：60 fps 采样、约 30 ms 延迟、一个未知的
计数到像素换算系数。对延迟误差做 PID，高增益就振荡、低增益就滞后。本仓库把闭环拆成
六个可测试的层：

| 层 | 关键点 |
|---|---|
| 配置 | schema 2 分组 + 字段注册表（同时驱动校验与 UI）+ 旧版迁移 + 原子写 + 热加载 |
| 采集 | 专用线程只保留最新帧；断流重连；陈旧度看门狗 |
| 感知 | 三种输出头（端到端 / YOLOv5 / YOLOv8）自动识别；类别白名单；可选 ROI；Kalman 多目标跟踪（ID、世界系速度、协方差） |
| 控制 | 世界坐标系 + 延迟采样线；**在线延迟辨识**；**速度前馈 + 速率归一化**；抗饱和；死区迟滞 |
| 输出 | 协议 v2（CRC8 / 序号 / 设备遥测）；报告率受限的累加器，**余数保留**（亚像素可收敛） |
| 观测 | JSONL 日志；分阶段百分位指标；会话录制（gzip JSONL）+ 闭环回放；闭环仿真台；离线自动调参 |

## 快速开始

```bash
git clone https://github.com/HuaTechnology/huakuo-valorant-yolo-aim-helper
cd huakuo-valorant-yolo-aim-helper
python3 -m venv .venv && source .venv/bin/activate
pip install -e ".[ui,hotkeys]"

python -m dma.cli doctor --dry-run          # 环境体检（无硬件也能跑）
python -m dma.cli run --sim --aim --dry-run # 合成画面：整条流水线可跑通
python -m dma.cli bench                      # 闭环回归门禁
pytest -q                                    # 79 个测试
```

真机：

```bash
cp configs/settings.example.json configs/settings.json
# 把模型放进 models/，改 vision.model 指向它
python -m dma.cli doctor                     # 模型类别 / 串口 / 推理基准
python -m dma.cli selftest --seconds 8       # 8 秒闭环自检，直接说明卡在哪一环
python -m dma.cli run --aim                  # 运行（空格切瞄准，F 切开火）
python -m dma.cli gui --aim                  # 桌面控制台
```

## 模型（不随仓库分发）

仓库**不在 git 里放权重**。作者自用的 320×320 端到端模型以 Release 附件提供
（[下载](https://github.com/HuaTechnology/huakuo-valorant-yolo-aim-helper/releases/latest)，
放进 `models/` 即可）。你也可以换成自己的模型，检测器自动识别三种输出头：

| 输出张量 | 家族 | 解码 |
|---|---|---|
| `[1, N, 6]` | 端到端导出（已内置 NMS） | `x1,y1,x2,y2,conf,cls` |
| `[1, 25200, 5+nc]` | YOLOv5 | objectness × class + 分类别 NMS |
| `[1, 4+nc, 2100]` 或转置 | YOLOv8 | anchor-free，无 objectness |

类别由模型 metadata 的 `names` 决定；`vision.target_classes` 是白名单，
`vision.ignore_classes` 里的类（如 `teammate`）永远不会成为目标。

## 标定（三步，缺一不可）

1. **头点**：模型有 `head` 类则自动锁头；只有身体类时，看实时画面里的红十字，
   拖 `policy.head_ratio` 到敌人头部（常见 0.08–0.15）。
2. **换算系数**：`aim2 calibrate gain`，量出「1 屏幕像素 = 多少鼠标计数」，
   写回 `output.gain`。这是唯一无法靠推理得到的常数。
3. **延迟**：`aim2 selftest` 跑一局，看 HUD 的「实测延迟 / 置信度」；
   置信度长期 < 0.3 时保持默认先验即可。

细节见 [`docs/CALIBRATION.md`](docs/zh-CN/CALIBRATION.md)。

## 文档

文档索引：[`docs/`](docs/README.md)（英文，标准版） · [`docs/zh-CN/`](docs/zh-CN/README.md)（中文镜像）。

| 文件 | 内容 |
|---|---|
| [`docs/zh-CN/ARCHITECTURE.md`](docs/zh-CN/ARCHITECTURE.md) | 分层、线程模型、四套坐标系、一次 tick 的时序 |
| [`docs/zh-CN/LATENCY_BUDGET.md`](docs/zh-CN/LATENCY_BUDGET.md) | 端到端延迟分解、增益稳定边界 |
| [`docs/zh-CN/MEASUREMENTS.md`](docs/zh-CN/MEASUREMENTS.md) | 所有实测数字与硬约束（含复现命令） |
| [`docs/zh-CN/CALIBRATION.md`](docs/zh-CN/CALIBRATION.md) | 头点 / 换算系数 / 延迟标定 |
| [`docs/zh-CN/TUNING.md`](docs/zh-CN/TUNING.md) | 每个参数「调大调小会发生什么」 |
| [`docs/zh-CN/OPERATIONS.md`](docs/zh-CN/OPERATIONS.md) | 运维、健康状态、故障排查表 |
| [`docs/zh-CN/adr/`](docs/zh-CN/adr) | 控制律与世界坐标系的决策记录 |
| [`firmware/esp32_hid/`](firmware/esp32_hid/) | ESP32-S3 固件、接线、烧录 |

## 目录

```
src/dma/
├── core/          配置 / 类型 / 指标 / 日志 / 时钟与延迟线
├── capture/       采集源（采集卡 / 视频 / 合成）+ 采集线程
├── perception/    几何变换 / 解码 / ONNX 检测器 / Kalman 跟踪
├── control/       延迟辨识 / 选靶 / 控制器 / 开火 / v1 基线（对照用）
├── transport/     协议 v2 / 子帧规划 / 串口链路
├── engine/        多线程引擎 + 快照 + 健康看门狗
├── sim/           闭环仿真台 / 合成检测器
├── tuning/        离线自动调参 / 标定
├── telemetry/     会话录制与回放
├── desktop/       PySide6 控制台（由配置注册表生成）
└── cli.py         aim2 统一入口
tests/             79 个用例：协议 / 几何 / 解码 / 跟踪 / 控制 / 引擎 / 仿真 / 回放
firmware/esp32_hid ESP32-S3 固件（UART 收协议、1000 Hz HID 出、STATUS 遥测）
```

## 硬件与接线（整套四样东西、四根线）

| # | 物料 | 数量 |
|---|---|---|
| 1 | HDMI 采集卡（UVC，1280×720@60 MJPG） | 1 |
| 2 | HDMI 线 | 1 |
| 3 | ESP32-S3 开发板（双口：`USB` + `UART`） | 1 |
| 4 | USB Type-C **数据线** | 2 |

```
Windows ──HDMI──▶ 采集卡 ──USB──▶ Mac          画面：Mac 只读画面
Mac ──Type-C──▶ ESP32 `UART` 口                控制：协议帧 921600，只走这根
ESP32 `USB` 口 ──Type-C──▶ Windows              输出：HID 鼠标，只走这根
```

两台机器之间**没有任何网络/文件通路**，唯一的共用器件是 ESP32：UART 收、HID 发。

- **推理机**：任意能跑 ONNX Runtime 的机器（本项目在 macOS + CoreML 上验证）
- **烧录**：ESP32-S3 的 ROM 下载模式两个口都支持，所以烧固件**不需要换线**
  （Mac 上 `pio run -t upload`，或 Windows 上 BOOT+RST 进下载模式）

细节、板级设置、烧录与排障见 [`firmware/esp32_hid/README.md`](firmware/esp32_hid/README.md)。
关键点：ESP32-S3 的 ROM 下载模式两个口都支持，**烧录不需要换线**。

## 已知限制

- 视觉链路有约 2 帧固有延迟，提帧率是唯一质变路径（见 `docs/MEASUREMENTS.md` §1）。
- 身体类模型没有头/队友类，锁头靠比例、团队模式会受队友干扰。
- 延迟辨识在「目标静止 + 控制器已进入死区」时没有可观测信号，此时保持先验。
- 仿真台是模型不是游戏本体，它只用于比较控制律与防止回归。

## 合规与免责

本项目是**实时视觉与控制系统的工程实现**，用于学习计算机视觉闭环、
实时系统、HID 协议与嵌入式开发。

- 在**你自己拥有或获授权**的环境中使用；
- 多数网络游戏的用户协议禁止第三方辅助，使用后果由使用者自行承担；
- 作者不对因使用本项目产生的任何封禁、数据损失或法律后果负责。

若你只是想研究「延迟补偿控制」，`aim2 sim --compare` 与 `docs/MEASUREMENTS.md`
里的实验不接触任何游戏，是最合适的入口。

## 许可证

[GPL-3.0](LICENSE)。第三方依赖与商标说明见 [`NOTICE`](NOTICE.zh-CN.md)。

## 致谢

- 控制律的取舍来自对延迟型积分对象的反复闭环实验，结论记录在 `docs/adr/0001`。
- 世界坐标系与延迟采样线的动机记录在 `docs/adr/0002`。
- 固件基于 Arduino-ESP32 core 3.x 的 `USBHIDMouse`。
