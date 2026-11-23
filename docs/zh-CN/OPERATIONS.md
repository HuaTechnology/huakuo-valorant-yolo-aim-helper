# 运维手册（OPERATIONS）

<div align="right">

**简体中文** | [English](../OPERATIONS.md)

</div>

> ⚠️ 仅供学习与研究，请勿用于任何竞技/排位场景。见 [DISCLAIMER](../../DISCLAIMER.md)。

> 代码基线：`src/dma`，schema 2，`aim2 2.0.0`。默认值取自 `src/dma/core/config.py`。
> 本文面向「把系统跑起来、看住、出问题能定位、能回退」的运维场景；原理见 [ARCHITECTURE](ARCHITECTURE.md)，参数语义见 [TUNING](TUNING.md)。

---

## 1. 安装与首次启动

### 1.1 依赖

| 项 | 要求 | 检查 |
|---|---|---|
| Python | ≥ 3.10（`cmd_doctor` 检查 `sys.version_info >= (3,10)`） | `aim2 doctor` |
| venv | 仓库根 `.venv`；`pip install -e .` 或 `PYTHONPATH=src` | `python -c "import dma"` |
| Python 包 | `pyproject.toml` 核心依赖（numpy/opencv-python/onnxruntime/pyserial）；可选 `[ui]` PySide6、`[hotkeys]` pynput、`[dev]` pytest。`requirements.txt` 是更全的固定版本清单（含 torch/ultralytics） | `aim2 doctor` 的 onnxruntime/opencv/pyserial 三项 |
| 模型 | `models/*.onnx` | `aim2 models list` |
| 摄像头权限 | macOS：系统设置 → 隐私与安全性 → 摄像头，勾选终端/iTerm/IDE | 无画面时先查 |
| 辅助功能权限 | 全局热键（`pynput`）需要 | 热键无效时查 |
| 串口 | `/dev/cu.usbmodem*`、`/dev/cu.wchusbserial*`、`/dev/ttyUSB*`、`/dev/ttyACM*`、`COM*` | `aim2 doctor` |

安装（推荐 editable 安装，拿到 `aim2` console script）：

```bash
cd ~/dma
python3 -m venv .venv
.venv/bin/python -m pip install -U pip
.venv/bin/python -m pip install -r requirements.txt     # 或：pip install -e ".[ui,hotkeys,dev]"
.venv/bin/python -m pip install -e .                    # 生成 .venv/bin/aim2
```

`pyproject.toml` 的 `[project.scripts]` 定义了 `aim2 = dma.cli:main`；当前这份 `.venv` 如果没做 editable 安装，`aim2` 不存在，用模块方式等价调用：

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python -m dma.cli --help
```

建议在 `~/.zshrc` 加别名（未安装 console script 时）：

```bash
export DMA_HOME="$HOME/dma"
alias aim2='PYTHONPATH="$DMA_HOME/src" "$DMA_HOME/.venv/bin/python" -m dma.cli'
```

`make` 目标（`Makefile`）：`make test` / `make bench` / `make sim` / `make doctor` / `make run` / `make autotune`，内部同样用 `.venv/bin/python` + `PYTHONPATH=src`。

### 1.2 配置来源与首次迁移

- 默认路径：`configs/settings.json`（`core/config.py::DEFAULT_PATH`）。
- 旧路径：`config.json`（`LEGACY_PATH`）。
- 首次 `load()` 时：
  - `configs/settings.json` 存在 → 直接读；
  - 不存在但 `config.json` 存在 → **内存中**迁移 v1（`migrate_v1`），不写盘；
  - 都不存在 → 用 dataclass 默认值。

全新安装、不想继承 v1 参数时，用仓库提供的示例配置：

```bash
cp configs/settings.example.json configs/settings.json
```

示例配置是纯 schema 2 默认值（`vision.model` 指向 `models/model.onnx`、`output.gain=1.0`）。若从 v1 迁移，模型路径与换算系数会沿用旧值，两者只能选一个作为起点。

迁移的关键结果（当前 `config.json` 实测）：

| v1 字段 | v2 字段 | 当前值 | 说明 |
|---|---|---|---|
| `SINK_GAIN` | `output.gain` | 3.0 | v1 用户拖满 |
| `max_step` | `aim.max_step` | 112.08 | |
| `kp_far` | `aim.kp_far` | 0.45 | |
| `kp_mid`/`kp_near`/`kp_touch` | `aim.kp_near` | 0.7297 | **最后写入的是 `kp_touch`**；迁移后 `kp_near` 取的是 v1 贴脸增益，需复核 |
| `kd`/`ki` | `aim.kd`/`aim.ki` | 0.388 / 0.2 | |
| `COAST_FRAMES` | `policy.coast_s` | 0.1 | 按 60 fps 换算（6/60） |
| `HEAD_RATIO` | `policy.head_ratio` | 0.1386 | |
| `FIRE_RADIUS` | `fire.radius` | 16.0 | |
| `PREDICT_*` / `ema_*` | — | 丢弃 | 预测控制器接管；写入 warnings |
| `latency_prior_s` | 强制 0.033 | 0.033 | 不信任 v1 的「1 帧」说法 |

`aim2 config show` 会把这些 warnings 打出来。**首次上线前先落盘一份 schema 2 基线**：

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli config migrate
PYTHONPATH=src .venv/bin/python -m dma.cli config validate
PYTHONPATH=src .venv/bin/python -m dma.cli config show > out/config_baseline.txt
```

`config migrate` 会把当前生效配置写到 `configs/settings.json`；之后 v2 只读写这个文件，`config.json` 保持原样（回滚用）。

### 1.3 模型选择

| 模型 | 适用 | 注意 |
|---|---|---|
| YOLOv8 320×320 导出（dataclass 默认） | 有 `head`/`teammate`，可锁头、可忽略队友 | `yolov8` 输出，必须 letterbox；`aim2 models inspect` 验证 |
| 端到端 320×320 导出（示例） | 只有 `st` 身体类，实测 60 fps 下够用 | 无 head/teammate；锁头靠 `head_ratio`；团队模式会受队友干扰 |
| YOLOv5 640×640 导出 | 有 head/body，640 输入 | `yolov5` 输出；640 推理更慢 |

切换：

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli config set vision.model YOLOv8 320×320 导出
PYTHONPATH=src .venv/bin/python -m dma.cli models inspect YOLOv8 320×320 导出 --json
# vision.* 变更需要重启进程（Engine.apply 只提示，不重建 detector）
```

### 1.4 协议与固件

主机侧支持两套帧：

| `output.protocol` | 帧格式 | 设备遥测 | 适用 |
|---|---|---|---|
| `v2`（默认） | `0xA5` + version/type + seq + len + payload + CRC8/ATM | 有（`STATUS`） | v2 固件 |
| `v1` | `0xAA` 10 字节 + XOR | 无 | 旧固件 |

**主机的 `output.protocol` 必须与固件匹配**；不匹配时表现为「串口能开、有写入、游戏不动」。固件在 `firmware/esp32_hid/`（`esp32_hid.ino` + `platformio.ini` + `README.md`），只做「串口帧 → 1000 Hz HID」，STATUS 为 13 字节 `"<IHHHHB"`，与 `protocol.STATUS_LEN=13` 一致。烧录设置见 README §2（必须 `USB-OTG (TinyUSB)` + `USB CDC On Boot: Enabled`）。

---

## 2. `run_mac.sh` 与 `aim2`

### 2.1 `run_mac.sh`

脚本硬编码 `PROJ="$HOME/dma"`，先 `cd` 到仓库根、设 `PYTHONPATH=src`，再按模式分派。默认模式是 `v2`。

**v2 入口（推荐）**

| 模式 | 实际动作 | 硬件 | 说明 |
|---|---|---|---|
| `./run_mac.sh run` | `python -m dma.cli run --aim`（其后参数原样透传） | 采集卡 + 串口 | 日志 tee 到 `out/logs/` |
| `./run_mac.sh run-dry` | `run --aim --dry-run` | 采集卡（串口可无） | MockLink，不写串口；安全联调 |
| `./run_mac.sh gui` | `python -m dma.cli gui --aim` | 采集卡 + 串口（或加 `--sim`） | PySide6 控制台 |
| `./run_mac.sh sim` | `run --sim --aim --dry-run` | 无 | 合成画面 + `SyntheticDetector`，完整回路 |
| `./run_mac.sh doctor` | `doctor --probe` | 串口（无串口会报阻塞项） | 环境/模型/串口体检 + 推理基准 |
| `./run_mac.sh selftest` | `selftest` | 采集卡 + 串口 | 8 秒闭环自检并给出结论 |
| `./run_mac.sh bench` | `bench` | 无 | 6 场景回归门禁；失败 exit 2 |

`run_mac.sh` 的路径全部相对脚本自身（`$(dirname "$0")`），clone 到任意目录都能用；
Python 解释器优先取 `.venv/bin/python`，没有则回退到 `python3`，可用 `PYTHON=` 覆盖。

`scripts/aim2` 是等价的免安装入口（自动设 `PYTHONPATH`，无参数即 `gui --aim`）。

---|---|---|---|
| `./run_mac.sh gui` | `launchctl asuser` 进 Aqua 会话，跑 `src/dma/main.py` → `qt_gui.py` | 是（可用 `MAC_PW`） | v1 PySide6 GUI，不驱动 v2 Engine |
| `./run_mac.sh tune` | `src/dma/gui.py` → `qt_gui.py`（实时调参 GUI） | 是 | v1 |
| `./run_mac.sh tune-demo` | `gui.py --demo`，合成画面 | 否 | v1 |
| `./run_mac.sh headless` | 打桩 `cv2.imshow`，跑 `main.py` | 否 | v1 无窗口 |
| `./run_mac.sh stop` | `pkill -f "dma.cli run"` 和 `pkill -f "dma/main.py"` | 否 | 同时停 v2/v1 |

注意：

- `run_mac.sh` 会 `shift` 掉第一个参数，所以 `./run_mac.sh run --duration 30` 会把 `--duration 30` 透传给 CLI。
- `v2` 已经带 `--aim`；想不开瞄准就手工用 CLI，或启动后按空格切换。
- `gui` 需要 `--sim` 时自己加：`./run_mac.sh gui --sim`。
- 日志 tee 只捕获 stdout/stderr 文本；JSONL 仍要按 §4.2 显式开启。

### 2.2 `aim2` 子命令

所有子命令通用：`--settings PATH`（全局，放在子命令前）、`--json`（放在子命令前后都可）。无参数时读默认配置。

| 子命令 | 作用 | 需要硬件 | 典型命令 |
|---|---|---|---|
| `version` | 打印 `aim2 2.0.0 (schema 2)` | 否 | `aim2 version` |
| `doctor` | 环境/模型/类别/串口体检；`--probe` 加推理基准 | 串口检查需要（`--dry-run` 跳过） | `aim2 doctor --dry-run --json` |
| `models list` | 扫描 `models/*.onnx`，逐个加载并报告格式/类别 | 否 | `aim2 models list` |
| `models inspect PATH` | 单模型详情 | 否 | `aim2 models inspect YOLOv8 320×320 导出 --json` |
| `models bench PATH` | 单模型基准（CLI 用随机 1280×720 帧；报告里的 2560×1600 数值见 `docs/MEASUREMENTS.md`） | 否 | `aim2 models bench 端到端 320×320 导出 --runs 20` |
| `sim` | 闭环仿真；`--compare` 同时跑 legacy/predictive | 否 | `aim2 sim --scenario strafe_mid --json` |
| `bench` | 回归门禁：6 个场景对比 v1，失败退出 2 | 否 | `aim2 bench` |
| `autotune` | 离线坐标下降；`--write` 写回 `aim` 段 | 否 | `aim2 autotune --passes 2 --write` |
| `replay PATH` | 回放会话；`--summary` 只看摘要；`--compare` 两控制器 | 否 | `aim2 replay out/telemetry/session_x.jsonl.gz --summary` |
| `run` | 启动 Engine；`--sim` 用合成画面 + `SyntheticDetector`（不需要 onnxruntime/相机），`--video` 用视频；其余 `--dry-run/--record/--aim/--fire/--duration/--hz/--report-hz/--no-hotkeys/--quiet` | 默认需要采集卡+串口；`--sim` 不需要 | `aim2 run --dry-run --duration 20` |
| `record` | 录制（`run --record` 的快捷方式，默认 10 秒） | 同上 | `aim2 record --sim --duration 10` |
| `config show/get/set/validate/migrate/diff` | 配置管理 | 否 | `aim2 config set aim.kp_far 0.8` |
| `calibrate model/latency/head` | 标定辅助 | `head` 不需要；`latency` 用合成源；`model` 不需要 | `aim2 calibrate head --path ...` |

退出码：

| 码 | 含义 |
|---|---|
| 0 | 成功 |
| 1 | `doctor` 有阻塞项 / `models inspect` 模型不 OK / `calibrate head` 样本不足 |
| 2 | `bench` 回归失败 / 配置错误（`ConfigError`）/ argparse 用法错误 |
| 130 | Ctrl-C |

### 2.3 桌面控制台（PySide6）

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python -m dma.desktop.app            # 真机
PYTHONPATH=src .venv/bin/python -m dma.desktop.app --sim      # 合成画面，无硬件
PYTHONPATH=src .venv/bin/python -m dma.desktop.app --dry-run  # 不写串口
# 或：
./run_mac.sh gui            # 真机；需要 sudo/launchctl
./run_mac.sh gui --sim      # 合成画面
```

`desktop/app.py` 的结构：

| 组件 | 作用 |
|---|---|
| `EngineThread`（QThread） | 独立线程跑 `Engine.run()`，UI 卡顿不影响控制 |
| `MainWindow._tick`（QTimer 33 ms） | 只读 `Engine.snapshot()`，刷新预览/HUD/状态灯 |
| `ParamRow` | 由 `core.config.FIELDS` 自动生成；范围/单位/帮助文本来自注册表 |
| `_on_param` | `set_fields` 校验 → `Engine.apply` 热加载；失败回滚控件值并写日志面板 |
| `PreviewWidget` | 画检测框、track 虚线框、目标框（红）、准星、死区圆 |
| 顶栏 | 健康状态、瞄准/开火开关、模型下拉（只改显示，切模型需重启）、保存/重载配置 |
| HUD | `cap/ctl/inf/err/mode/lat(conf)/det/trk/fire`，与 `Snapshot.top_line` 同源 |

注意：

- 桌面端 `setup_logging(console=False, log_dir=None)`：**不写控制台、不落盘**；日志面板只显示 UI 自己的配置/重启提示，不显示引擎日志。
- 改 `vision.*`/`capture.*`/`output.port|baud` 时 `Engine.apply` 只返回「需重启」提示，不会重建 detector/采集卡/串口。
- 关窗口会 `engine.stop()` 并等待 `EngineThread` 1 秒。
- 模型下拉只影响下次启动；不要指望在运行中切模型。

### 2.4 推荐启动顺序

```bash
cd ~/dma
export PYTHONPATH=src

# 1) 体检（不碰串口）
.venv/bin/python -m dma.cli doctor --dry-run

# 2) 配置基线（第一次）
.venv/bin/python -m dma.cli config migrate
.venv/bin/python -m dma.cli config validate

# 3) 无硬件冒烟：合成画面 + SyntheticDetector + MockLink（不需要 onnxruntime）
.venv/bin/python -m dma.cli run --sim --duration 10 --no-hotkeys

# 4) 干跑：真采集卡 + MockLink，不开瞄准
.venv/bin/python -m dma.cli run --dry-run --duration 20 --no-hotkeys

# 5) 实机（先不开火）
.venv/bin/python -m dma.cli run --aim --no-hotkeys
# 需要开火时再去掉 --no-hotkeys，用 F 键切换；或直接 --fire

# 6) 回归/单测（不碰硬件）
.venv/bin/python -m pytest -q
```

`run_mac.sh run` 等价于第 5 步并自动带 `--aim`；`run_mac.sh sim` 等价于第 3 步；`run_mac.sh gui` 打开桌面控制台。

`aim2 run` 默认启用全局热键：空格切换瞄准，`f` 切换开火（`cli._run_engine`）。没有辅助功能权限时热键监听失败但不会阻塞运行。

`aim2 run` 每 0.5 秒在终端刷新 `Snapshot.top_line()`，退出时打印 `engine.describe()`（健康、模型、采集、链路、延迟、开火次数、画面尺寸）。

---

## 3. 健康状态

`Health` 五态（`core/types.py`），`Engine._evaluate` 只产生 `OK/DEGRADED`：

| 状态 | 含义 | 是否允许瞄准/开火 | 典型场景 |
|---|---|---|---|
| `BOOT` | 构造完成、未启动 | 否 | 启动瞬间 |
| `OK` | 采集/推理/链路都健康 | 是 | 正常运行 |
| `DEGRADED` | 某一阶段不健康，`health_detail` 给出原因 | **否**（`can_aim=False`） | 见下表 |
| `FAILED` | 预留不可恢复错误；**当前没有赋值路径** | 否 | 不要依赖它告警 |
| `STOPPED` | `Engine.stop()` 之后 | 否 | 正常退出 |

`DEGRADED` 的四种原因与处置：

| `health_detail` | 触发条件（代码） | 阈值（默认） | 先查什么 | 处置 |
|---|---|---|---|---|
| `采集流中断` | `grabber.stale_s()*1000 > runtime.watchdog_ms` | 400 ms | 采集卡是否有 HDMI 信号；摄像头权限；是否被 OBS/其它程序占用；USB 供电/线缆 | 重新插卡/换口；关掉占用程序；`aim2 doctor` 看摄像头；恢复后自动回 `OK` |
| `等待首帧推理` | `det is None` | — | 模型是否加载成功；推理线程是否异常；采集是否刚启动 | 等 1–2 秒预热；持续则看日志 `推理失败` / `perception.errors` |
| `推理结果陈旧` | `now - det.capture_ts > runtime.max_infer_age_s` | 250 ms | `perception.infer_ms` p95 是否飙升；采集 fps 是否掉到 0；控制线程是否被阻塞 | 降分辨率/换小模型；查 CPU/GPU 占用；看 `control.hz` 是否掉 |
| `输出链路断开` | `not link.ok()` | 端口未开或连续 5 次写错误 | `link.status()["detail"]`（异常类型）；串口是否被占用；固件是否匹配 | 重插 ESP32；`aim2 doctor`；检查 `output.protocol` 与固件 |

阶段状态（`Snapshot.stages`）是另一组独立判据：

| StageStatus | 来源 | ok 条件 | detail |
|---|---|---|---|
| `capture` | `grabber.stale_s() < 0.2` | 200 ms 内有新帧 | 采集 fps |
| `inference` | `det` 非空且 `now - capture_ts < max_infer_age_s` | 结果新鲜 | 推理 ms |
| `link` | `link.status()["ok"]` | 端口打开且错误 <5 | 端口名或 `mock` |

注意：

- `DEGRADED` 只抑制瞄准/开火，不会停采集/推理/链路；所以 HUD 仍能看到检测框，便于定位。
- 启动前几百毫秒常见 `采集流中断`（还没读到第一帧）或 `等待首帧推理`，不是故障。
- `FAILED` 没有代码路径；告警应基于 `DEGRADED` + `stages` + 计数器，而不是等 `FAILED`。

---

## 4. 日志

### 4.1 控制台

`aim2` 启动时调用 `core.logging.setup(level="INFO", log_dir=None, console=True)`（`cli.main`），所以**当前 CLI 只写控制台，不写文件**。桌面端 `desktop/app.py::main` 调用 `setup_logging(level="INFO", log_dir=None, console=False)`，**既不写文件也不写控制台**，日志只进 UI 自己的消息面板（且只有配置/重启提示）。格式（`ConsoleFormatter`）：

```
HH:MM:SS L src                     msg key=value
```

日志源（`src` 字段）常见值：`dma.engine`、`dma.link`（如有）、`dma.cli`。

### 4.2 JSONL 文件（需要显式开启）

`core/logging.setup` 支持 `out/logs/aim.jsonl`，按 8 MiB × 3 轮转。CLI 没传 `log_dir`，要落盘时用包裹脚本：

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python - <<'PY'
from dma.core.config import load
from dma.core.logging import setup
from dma.engine.runtime import Engine

setup("INFO", "out/logs")          # 返回日志路径；文件名默认 aim.jsonl
engine = Engine(load())
try:
    engine.run(duration_s=60)
finally:
    engine.stop()
PY
```

字段（`JsonFormatter`）：

| 字段 | 含义 |
|---|---|
| `ts` | `record.created`（epoch 秒，浮点） |
| `t` | `HH:MM:SS` 本地时间 |
| `lvl` | `DEBUG/INFO/WARNING/ERROR` |
| `src` | logger 名（阶段） |
| `msg` | 人类可读消息 |
| `exc` | 异常堆栈（仅 `exc_info` 时） |
| 其它 | `extra=` 里的结构化字段，原样平铺 |

常见事件与 extras：

| msg | src | extras | 含义 |
|---|---|---|---|
| `引擎启动` | `dma.engine` | `model`, `control_hz` | 启动参数 |
| `健康状态变更` | `dma.engine` | `health`, `detail` | 只在状态变化时记录（`_set_health`） |
| `画面尺寸` | `dma.engine` | `w`, `h` | 首次/分辨率变化；同时清空延迟线 |
| `瞄准开关` | `dma.engine` | `enabled` | 空格/CLI |
| `开火开关` | `dma.engine` | `enabled` | `f`/CLI |
| `配置热加载` | `dma.engine` | `changed` | `Engine.apply` 的 diff 数量 |
| `参数变更` | `dma.engine` | `field`, `old`, `new` | 每个字段一条 |
| `推理失败` | `dma.engine` | `err` | 限频 1 秒（`StageLogger.throttled`） |
| `引擎停止` | `dma.engine` | `shots` | 开火次数 |

运维 grep 示例：

```bash
grep -c '"health":"degraded"' out/logs/aim.jsonl
grep '"detail":"推理结果陈旧"' out/logs/aim.jsonl | tail
grep '"msg":"参数变更"' out/logs/aim.jsonl | tail -20
```

### 4.3 会话录制（另一套文件）

`aim2 run --record` / `aim2 record` 会额外写 `out/telemetry/session_YYYYmmdd_HHMMSS.jsonl.gz`（`SessionRecorder`）：

| record `type` | 内容 |
|---|---|
| `header` | `ts`（epoch）、`schema`、完整 `settings`、`model` 描述 |
| `frame` | `capture_ts`、`frame_id`、`infer_ms`、`cam` 样本、`dets`（source 框 + cls_id + score） |
| `tick` | 控制 `ts`、`cam`、`u`（dx/dy）、`tid`、`err`、`mode`、`lat`（loop_ms/confidence/frames） |
| `fire` | `ts`、`fire`、`reason`、`dist` |

查看摘要：

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli replay out/telemetry/session_x.jsonl.gz --summary
```

注意：`replay` 把每条检测的类别名硬编码为 `"body"`（`telemetry/replay.py`），所以回放不能复现 head 类融合；它适合比较控制律/参数，不适合验证锁头策略。

---

## 5. 指标

`Metrics`（`core/metrics.py`）有三类：

| 类型 | 方法 | 含义 |
|---|---|---|
| 直方图 | `observe(name, v)` / `stats_summary()` | 固定窗口（默认 240 样本），输出 `n/mean/p50/p95/p99/min/max/std/last` |
| 计数器 | `inc(name)` / `counters()` | 单调累加，重启清零 |
| Gauge | `set(name, v)` / `gauges()` | 当前值 |

主要指标（来自 `Engine`/`OnnxDetector`/`SerialLink`/`FireController` 的调用点）：

| 指标 | 类型 | 含义 | 健康读法 |
|---|---|---|---|
| `capture.fps` | gauge | 采集线程 EMA 帧率 | 游戏中约 60；桌面 120 |
| `capture.frames` | counter | 成功读帧数 | 持续增长 |
| `capture.read_errors` | counter | 读失败次数 | 偶发可接受；持续增长查卡/线 |
| `capture.open_errors` | counter | 打开失败次数 | 启动阶段可偶发 |
| `capture.reconnects` | counter | 断流重开次数 | 0 最好；增长说明信号不稳 |
| `capture.interval_ms` | hist | 帧间隔 | p95 接近 1000/fps；突增说明掉帧 |
| `perception.infer_ms` | hist | `sess.run` 纯推理 | 320 模型 4–6 ms（实测） |
| `perception.total_ms` | hist | 预处理+推理+后处理 | 比 infer 多几 ms |
| `perception.pre_ms` / `post_ms` | hist | 预处理/后处理 | 用于定位慢在哪个阶段 |
| `perception.dets` | gauge | 最近一帧检测数 | 0 可能没目标/模型没检出 |
| `perception.errors` | counter | 推理异常次数 | 应为 0 |
| `control.hz` | gauge | 控制线程实际频率 | 接近 `runtime.control_hz`（240） |
| `aim.err_px` | hist | 控制误差（像素） | p50/p95 看跟枪质量；与场景有关 |
| `aim.kp` | hist | 当前调度后的 kp | 用于确认调度区间 |
| `aim.cam_x` / `cam_y` | gauge | 累计命令位移 | 世界系状态 |
| `aim.saturated` | counter | 输出饱和次数 | 大甩枪会增长；持续增长说明 `max_step` 偏小或 kp 偏大 |
| `aim.hold_ticks` | counter | 死区内保持的 tick 数 | 静止时增长正常 |
| `tracker.count` | gauge | 当前 track 数 | 多目标场景 1–5；暴涨说明误检/关联差 |
| `transport.frames` | counter | 串口写成功帧数 | 持续增长 |
| `transport.write_ms` | hist | 单次写耗时 | p95 应远小于 `write_timeout_s=20 ms` |
| `transport.write_errors` | counter | 写异常 | 应为 0 |
| `transport.reconnects` | counter | 串口重开次数 | 0 最好 |
| `transport.connected` | gauge | 端口是否打开 | 1 |
| `device.rx_frames` | gauge | 设备收到的帧数（STATUS） | 持续增长 |
| `device.crc_errors` | gauge | 设备侧 CRC 错误 | 应为 0；增长查线缆/波特率 |
| `device.queue_drops` | gauge | 设备队列丢弃 | 应为 0；增长说明报告率过高/固件忙 |
| `device.report_hz` | gauge | 设备报告率（STATUS） | 与 `output.report_hz` 对照 |
| `fire.shots` | counter | 开火次数 | 与 `shots` 一致 |
| `fire.reason.<reason>` | counter | 每种开火判定原因的计数 | 看分布，不只看 shots |
| `health.<state>` | counter | 健康状态切换次数 | 频繁切换说明链路/采集不稳 |

推理段基线（本机 CoreML，2560×1600 输入帧，`aim2 models bench`；见 `docs/MEASUREMENTS.md`）：

| 模型 | p50 | p95 |
|---|---|---|
| YOLOv8 320×320 导出 | 4.1 ms | 7.9 ms |
| 端到端 320×320 导出 | 5.0 ms | 6.3 ms |
| YOLOv5 640×640 导出 | 7.4 ms | 8.2 ms |
| 端到端 640×640 导出 | 16.6 ms | 20.6 ms |

CLI 的 `aim2 models bench` 当前用随机 1280×720 帧，数值会因预处理/缩放略有差异；上表用于判断「换模型是否显著变慢」，不是闭环延迟。

读取方式：CLI 没有 `metrics` 子命令，`describe()` 也不含完整指标。需要时用脚本：

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python - <<'PY'
import json, time
from dma.core.config import load
from dma.core.metrics import Metrics
from dma.engine.runtime import Engine
from dma.capture.sources import SyntheticSource
from dma.transport.link import MockLink

m = Metrics()
s = load()
e = Engine(s, metrics=m, source=SyntheticSource(s.capture.width, s.capture.height, 60),
           link=MockLink(s.output, m))
e.start()
try:
    for _ in range(300):
        e.step(); time.sleep(1/240)
finally:
    e.stop()
print(json.dumps(m.snapshot(), ensure_ascii=False, indent=2))
PY
```

`Metrics.to_prometheus()` 可以导出极简 text exposition；当前没有 HTTP 端点，需要自己接。

---

## 6. 常见故障排查

### 6.1 启动/环境

| 症状 | 可能原因 | 检查 | 处置 |
|---|---|---|---|
| `ModuleNotFoundError: dma` | 没设 `PYTHONPATH=src` | `echo $PYTHONPATH` | 按 §1.1 设置 |
| `aim2: command not found` | 没有安装 console script | `which aim2` | 用 `python -m dma.cli` 或别名 |
| `doctor` 报 onnxruntime 缺失 | venv 没装依赖/用错 Python | `which python`、`pip list` | 用 `.venv/bin/python` 重装 |
| `doctor` 报模型不存在 | `vision.model` 路径相对 cwd 不对 | `aim2 config get vision.model` | 从仓库根运行；或写绝对路径 |
| `doctor` 报「没有可射击类别」 | 模型 metadata 无 `names` 或 `target_classes` 不匹配 | `aim2 models inspect`、`config show` | 修正 `vision.target_classes`；换模型 |
| `doctor` 报「队友/人机类别被列为可射击」 | `target_classes` 误加 `teammate/ai` | `config show` | 从 target 移除；ignore 保留 |
| `doctor` 报 yolov8 用 stretch | `vision.letterbox=off/stretch` | `config get vision.letterbox` | 改回 `auto`/`on` |
| 配置非法 | 越界/枚举/near>=far/deadzone_exit<deadzone/宽高奇数 | `aim2 config validate` 的错误文本 | 按字段修正；不要手改 JSON 绕过校验 |
| `config migrate` 覆盖担心 | 写的是 `configs/settings.json`，不动 `config.json` | `ls -l configs/` | 需要保留旧档先 `cp` |

### 6.2 采集/画面

| 症状 | 可能原因 | 检查 | 处置 |
|---|---|---|---|
| 无画面 / `采集流中断` | 摄像头权限、HDMI 无信号、被 OBS 占用、USB 供电 | `capture.read_errors`、`grabber.describe()` | 系统设置授权；换线/换口；关 OBS；确认采集卡灯 |
| 首帧后一直 `等待首帧推理` | 推理线程异常/模型不兼容 | 日志 `推理失败`、`perception.errors` | 用 `aim2 models inspect` 验证；看异常文本 |
| `推理结果陈旧` | 推理 p95 超过 250 ms | `perception.infer_ms` | 换 320 模型/降 ROI/关其它占 CPU 的程序 |
| 采集只有 30 fps | 采集卡 720p 只支持 30/驱动协商失败 | `grabber.describe()` 的 `fps/width/height` | 降分辨率/换 fourcc；见docs/MEASUREMENTS.md |
| 桌面 120、进游戏 60 | 游戏独占全屏锁 HDMI 60 Hz | `capture.fps` | 改无边框全屏/不限帧/关垂直同步（docs/MEASUREMENTS.md） |
| 采集分辨率与游戏不一致 | 卡在缩放，多 1–2 帧 | `frame.shape` vs `capture.width/height` | 对齐游戏原生分辨率（docs/MEASUREMENTS.md） |
| `capture.reconnects` 持续增长 | 信号瞬断/USB 不稳 | 日志、`read_errors` | 换线/换口/加供电；确认 HDMI 线 |

### 6.3 瞄准/控制

| 症状 | 可能原因 | 检查 | 处置 |
|---|---|---|---|
| 瞄准开了但不动 | `health != OK`；没有目标；`aim_enabled` 未开 | `Snapshot.top_line()`、`fire.reason`、`health` | 先修健康/目标；空格切换 |
| 完全反向/乱飘 | 世界系尺度/符号错；增益标定错 | §3 增益标定；`output.gain` | 重标；不要在控制器里翻符号 |
| 目标附近来回过冲 | `kp_far` 高 + 延迟估计偏小；`max_step` 大 | `loop_ms`、`confidence`、`aim.saturated` | 降 `kp_far`/`max_step`；等延迟收敛 |
| 甩枪到位回弹 | 积分 windup/前馈过大 | `lead_max`、`integral_limit`、`aim.saturated` | 降 `lead_max`/`ki`；确认抗饱和生效 |
| 跟枪滞后 | 前馈不足/延迟大 | `lead_gain`、`loop_ms` | 提高 `lead_gain`（≤1.0）；先修延迟 |
| 静止抖动 | 死区太小/输出未平滑/检测噪声 | `aim.err_px` p50、`output_ema`、`deadzone` | 提高死区/降 `output_ema`/提高 `kd` |
| 多目标来回切 | `switch_margin` 太小；track 身份不稳 | `policy.switches`（无指标，需日志/UI）、`tracker.count` | 提高 `switch_margin`；提高 `tracker.iou_thr` 一致性；别同时调 `fov` |
| 往队友身上拉 | 模型无 teammate 类/类别配置错 | `models inspect` 的 targets | 换 sjz 模型；`ignore_classes` 含 teammate；团队模式先关瞄准 |
| 瞄头偏上/偏下 | `head_ratio` 未标定/换模型 | §2 标定 | 重标；有 head 类时不要用 head_ratio |
| 热加载后手感突变 | `Engine.apply` 因 `kind` 判断重建控制器，积分/EMA 清零 | 日志 `配置热加载` | 改完参数重启；或接受一次状态清零 |
| 关闭瞄准后还有小位移 | `set_aim_enabled(False)` 的 flush 把残留又 submit 回去（见 ARCHITECTURE §8） | 观察 `planner.pending` | 先 `config set output.dry_run true` 再关；或改代码修 flush |

### 6.4 开火

`FireController.update` 的 `reason` 是排查开火问题的第一手信息：

| reason | 含义 | 处置 |
|---|---|---|
| `off` | `fire.mode="off"` | 改 `single`/`auto` |
| `no-target` | 当前帧没有可用目标 | 查检测/选靶/FOV/遮罩 |
| `coasting` | 目标丢失，正在外推 | 正常保护；频繁出现查跟踪/遮挡 |
| `uncertain` | `track.sigma > fire.max_sigma` | 目标刚出现/刚切换；等稳定；或提高 `max_sigma`（谨慎） |
| `off-target` | 命中点不在内缩框内且残差 > `fire.radius` | 瞄准点/延迟/开火半径问题；先看 `aim.err_px` |
| `settling` | 在目标上但驻留 < `dwell_s` | 正常；目标抖动/刚切人会这样 |
| `cooldown` | 距上次开火 < `cooldown_s` | 正常；连发节奏 |
| `already-fired` | `single` 模式本次锁定已开过 | 正常；换目标或切 `auto` |
| `fire` | 开火 | — |

| 症状 | 检查 | 处置 |
|---|---|---|
| 一枪不开，reason=`off` | `Snapshot.fire_enabled`、`fire.mode` | CLI `--fire` 或配置 `fire.mode` |
| 一直 `off-target` | 命中点/框/延迟 | 先关火，看瞄准误差；重标头点/延迟 |
| 一直 `uncertain` | `tracker.count`、目标是否刚出现 | 等 2–3 帧；检查 `tracker.accel_noise/meas_noise` |
| `fire` 了但游戏没反应 | 串口/固件/按键掩码 | `transport.frames`、`device.rx_frames`、按键帧 |
| 连发太快/太慢 | `cooldown_s`/`dwell_s`/`tap_s` | 按 [TUNING](TUNING.md) 调；一次一个 |

### 6.5 链路/串口

| 症状 | 可能原因 | 检查 | 处置 |
|---|---|---|---|
| `未找到串口设备` | 没插/驱动/被占用 | `ls /dev/cu.usbmodem*`、`aim2 doctor` | 换线/装驱动/关占用程序 |
| 多个候选端口 | 多个 USB 串口 | `doctor` 的 detail | 设 `output.port` 显式指定 |
| 串口能开但游戏不动 | 协议不匹配/固件没跑 HID | `output.protocol`、`device.rx_frames` | 协议对齐；重刷固件 |
| `transport.write_errors` 增长 | 线缆/波特率/供电 | `link.status()["detail"]` | 换线/降波特率/加供电 |
| `transport.reconnects` 增长 | 端口反复掉 | 日志 | 换 USB 口；避免 hub |
| `device.crc_errors` 增长 | 线缆质量/波特率偏差 | STATUS | 换短线；确认 921600 |
| `device.queue_drops` 增长 | 报告率过高/固件忙 | `output.report_hz` | 降到 250/500；查固件队列 |
| 鼠标抖动/台阶感 | 报告率低/单报告限幅 | `planner.report()` 的 clipped/residual | 提高 `report_hz`；确认 `max_per_report=127` |

### 6.6 延迟/仿真/回放

| 症状 | 可能原因 | 检查 | 处置 |
|---|---|---|---|
| `confidence=0` | 目标/准星不动；合成画面无检测 | `samples`、`frames` | 用真实移动目标；见 [CALIBRATION](CALIBRATION.md#4-延迟标定) |
| `frames` 跳变 | 检测不连续/掉帧 | `capture.interval_ms`、`tracker.count` | 先修采集/跟踪 |
| `loop_ms` 与 `frames/fps` 不符 | 帧率不稳/`frame_dt` 中位数失真 | 同上 | 稳定帧率后再估 |
| `aim2 bench` 失败 | 预测律在某些场景不如 v1 | 输出 `failures` | 看 [TUNING §5](TUNING.md#5-仿真回归验证)；不要忽略门禁 |
| `aim2 replay` 结果异常 | 回放把类别名硬编码 `body`、中心固定 1280×720 | 会话分辨率 | 只用于控制律对比；不要用它验证 head |
| `autotune` 把参数推到边界 | 搜索空间上界/目标函数 | `--json` 的 history/rows | 用 `bench`/`replay` 复核；不要直接上线 |

---

### 6.7 桌面端

| 症状 | 可能原因 | 检查 | 处置 |
|---|---|---|---|
| `ModuleNotFoundError: PySide6` | venv 没装桌面依赖 | `python -c "import PySide6"` | `pip install PySide6`（`requirements.txt` 不含它） |
| 窗口打开但无画面 | 采集卡/模型未就绪；`snap.frame` 为空 | HUD 的 `cap`、`inf`；`health` | `--sim` 先验证 UI；真机查摄像头权限 |
| 改参数提示「配置被拒绝」 | `set_fields` 校验失败（越界/枚举/near>=far） | 日志面板文本 | 按提示改值；不要绕过 `FIELDS` 范围 |
| 改参数后手感突变 | 任何 diff 都触发控制器重建（`Engine.apply` 的 `kind` 判断） | 是否有「需重启」提示 | 大改后重启；小改接受一次积分清零 |
| 切模型没生效 | 模型下拉只影响下次启动 | 顶栏选择 | 保存配置后重启 `aim2`/桌面端 |
| GUI 起不来 | 无显示会话 | 终端提示 | 直接 `python -m dma.cli gui`，或在桌面里开终端 |
| 桌面端关不掉引擎 | `closeEvent` 等待 1 秒 | 进程是否存在 | `pkill -f dma.desktop.app`；再 `pkill -f "dma.cli run"` |

## 7. 回滚到 v1

### 7.1 先明确回滚的层次

| 层次 | 做法 | 影响 |
|---|---|---|
| 只回滚控制律 | `aim.controller=legacy` | v2 Engine/世界系/协议仍在；用 `dma.aim.AimController`（v1 PD+I）做 A/B；不是完整 v1 |
| 回滚主机程序 | 跑 `src/dma/main.py` / `run_mac.sh gui` | 回到 v1 单线程采集+推理+串口；`config.json` 生效 |
| 回滚固件 | 刷回 v1 固件 | v1 主机发 `0xAA`；v2 固件只认 `0xA5`，必须成对切换 |
| 回滚代码 | `git checkout <v1 的提交或 tag>`，或从你自己的备份取回单文件 | 不要整目录覆盖当前 `src/` |
| 回滚桌面端 | 关闭 `desktop/app.py`，用回 v1 `qt_gui.py` | 两套 UI 参数模型不同：v2 改 `configs/settings.json`，v1 改 `config.json` |

### 7.2 保留现场

```bash
cd ~/dma
BK="out/rollback_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BK"
cp configs/settings.json "$BK/" 2>/dev/null || true
cp config.json "$BK/" 2>/dev/null || true
cp -r out/logs out/telemetry "$BK/" 2>/dev/null || true
echo "现场已保存到 $BK"
```

v2 的配置在 `configs/settings.json`；v1 的配置在 `config.json`。两者独立，v2 的 `save` 不会改 `config.json`，所以**不要删 `config.json`**。

### 7.3 回滚步骤（主机）

1. 停止 v2：Ctrl-C（`aim2 run` 会走 `Engine.stop()`），或 `pkill -f "dma.cli run"`；桌面端另加 `pkill -f dma.desktop.app`。
2. 确认 `config.json` 还是 v1 的值：`python -c "import json;print(json.load(open('config.json'))['model_path'])"`。
3. 跑 v1：

```bash
cd ~/dma
./run_mac.sh check          # 需要串口，否则 exit 2
./run_mac.sh gui            # 需要 sudo/launchctl；或：
PYTHONPATH=src .venv/bin/python src/dma/main.py
```

4. v1 无窗口模式：

```bash
PYTHONPATH=src .venv/bin/python src/dma/main_cli.py
```

5. 验证：GUI 有预览、HUD 有「实测延迟 D」、空格/F 热键生效、开火原因正确。
6. 如果 v2 曾把固件升级到 v2 协议：v1 主机发 `0xAA`，v2 固件不认；要么刷回 v1 固件，要么先不要用 v1 主机。

### 7.4 关于旧版本代码

公开仓库不含任何 v1 备份包。需要对照旧实现时：

```bash
git log --oneline --all | tail            # 找到 v1 时代的提交
git show <commit>:src/dma/aim.py > /tmp/v1_aim.py    # 取回单个文件
```

`control/legacy_pid.py` 保留了 v1 的控制律本体，可随时对照。

### 7.5 回滚后的清理

- 不要让 v2 和 v1 同时开采集卡/串口：同一设备只能一个进程。
- v2 的 `out/telemetry`、`out/logs` 保留，不要覆盖。
- 若之后要回到 v2：`aim2 config validate`；`configs/settings.json` 里可能残留 v2 调参值，和 `config.json` 不同步，先 `aim2 config diff --other config.json`（注意后者是 v1 格式，`load` 会迁移）。

---

## 8. 运维检查清单

### 8.1 开机前

- [ ] `aim2 doctor --dry-run` 通过；串口检查用非 dry-run。
- [ ] `aim2 models inspect` 的 targets/heads 正确；无 teammate/ai。
- [ ] 用桌面端时 `python -c "import PySide6"` 通过。
- [ ] `python -m pytest -q` 全通过（当前 79 个用例）。
- [ ] 游戏无边框全屏、不限帧、关垂直同步；采集分辨率=游戏原生。
- [ ] 游戏内灵敏度/DPI 未变（变了重标增益/延迟）。
- [ ] 磁盘有空间（日志/录制；`out/` 已含大文件）。
- [ ] 只有本进程占用采集卡/串口。
- [ ] 固件按 `firmware/esp32_hid/README.md` §2 的 USB 设置烧录；`output.protocol` 匹配。

### 8.2 运行中

- [ ] `health=OK`；`stages` 三项 ok。
- [ ] `capture.fps` 符合预期（游戏中 60）。
- [ ] `perception.infer_ms` p95 与基线同量级。
- [ ] `latency.confidence ≥ 0.35`、`frames` 稳定。
- [ ] `aim.saturated` 不持续快速增长。
- [ ] `transport.write_errors=0`、`device.crc_errors=0`、`queue_drops=0`。
- [ ] `fire.reason` 分布合理（大部分 `fire/settling`）。

### 8.3 收工后

- [ ] 保存 `out/logs/aim.jsonl`（若按 §4.2 显式开启）与会话（如开启 `--record`）。
- [ ] 记录 `engine.describe()` 的 latency/link/shots。
- [ ] 若改过参数：`aim2 config diff --other 备份.json` 存档。
- [ ] 不要提交 `.venv/`、`out/`、模型权重到版本库（`.gitignore` 已覆盖；权重走 Release 附件）。

---

本文档对应 schema_version 2 与 src/dma 布局，改动代码请同步更新。
