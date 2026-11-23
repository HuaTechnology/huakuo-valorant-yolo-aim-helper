# 标定手册（CALIBRATION）

<div align="right">

**简体中文** | [English](../CALIBRATION.md)

</div>

> ⚠️ 仅供学习与研究，请勿用于任何竞技/排位场景。见 [DISCLAIMER](../../DISCLAIMER.md)。

> 代码基线：`src/dma`，schema 2。默认值取自 `src/dma/core/config.py`；实测结论见 `docs/MEASUREMENTS.md`。
> 标定顺序固定：**模型/类别 → 头部瞄准点 → 输出灵敏度/增益 → 闭环延迟 → 验证清单**。前一项没通过，不要做后一项。

---

## 0. 前置条件

| 条件 | 要求 | 检查 |
|---|---|---|
| 游戏显示 | 无边框全屏、帧率不限制、关闭垂直同步 | docs/MEASUREMENTS.md：独占全屏会把 HDMI 锁 60 Hz |
| 采集分辨率 | 与游戏原生分辨率一致（默认 1280×720） | `CvSource.describe()`、`capture.width/height`；docs/MEASUREMENTS.md：不对齐会多 1–2 帧缩放延迟 |
| 游戏内灵敏度 | 标定期间固定；关闭鼠标加速/原始输入以外的平滑 | 标定后不得再改 |
| 模型 | `aim2 models inspect` 通过 | `vision.model`、`vision.target_classes/ignore_classes` |
| 输出链路 | 串口能开、固件协议匹配 | `aim2 doctor`、`output.protocol` |
| 安全环境 | 训练场/离线房间，准星周围没有队友 | `vision.ignore_classes` 已含 `teammate/ai/crosshair`，但模型仍可能误检 |

---

## 1. 模型与类别标定

### 1.1 检查命令

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python -m dma.cli models list
PYTHONPATH=src .venv/bin/python -m dma.cli models inspect YOLOv8 320×320 导出 --json
PYTHONPATH=src .venv/bin/python -m dma.cli doctor --dry-run
```

`tuning/calibration.py::model_report` 会给出：

| 字段 | 含义 | 通过标准 |
|---|---|---|
| `format` | `e2e` / `yolov5` / `yolov8` | 与模型实际输出一致；`yolov8` 必须是 `letterbox` |
| `targets` | 被 `vision.target_classes` 命中的类别名 | 至少一个；不能包含 `teammate/ai` |
| `heads` | 被识别为 head 的类别 | 有 `head` 时可直接锁头；没有则要标定 `head_ratio` |
| `errors` | 阻塞项 | 必须为空 |
| `warnings` | 非阻塞提示 | 逐条确认（常见是「模型无 head 类」） |

本仓库三个模型的实测元数据：

| 模型 | 输入 | 输出 | 解码 | 类别 | head |
|---|---|---|---|---|---|
| YOLOv8 320×320 导出（`vision.model` 默认） | 320×320 | `[1,9,2100]` | `yolov8` | `body, head, teammate, ai, crosshair` | 有 |
| 端到端 320×320 导出（示例） | 320×320 | `[1,300,6]` | `e2e` | `st, qiu, dj, fyb, sg1..sg4`（示例模型的类别表） | 无 |
| YOLOv5 640×640 导出 | 640×640 | `[1,25200,7]` | `yolov5` | `head, body` | 有 |

### 1.2 类别白名单/黑名单

- `vision.target_classes` 默认 `("body","head","st","person","enemy","target")`。
- `vision.ignore_classes` 默认 `("teammate","ai","crosshair")`。
- `ClassMap.is_targetable` 要求「在 target 中且不在 ignore 中」；`OnnxDetector.detect` 在 NMS 前就过滤，**被忽略的类别不会进入 tracker**。
- 模型只有身体类（如 `st`）时，身体可打；没有 `head` 类，锁头只能靠 `head_ratio`。
- 模型有 `teammate` 时，即使 `target_classes` 误加，`ignore_classes` 也会兜底；但 `model_report` 仍会把「队友被列为 target」报成 error，要求你改配置而不是靠兜底。

### 1.3 实时确认

1. `aim2 run --dry-run --duration 30 --no-hotkeys`（不开瞄准，先看检测）。
2. 在 UI/日志里确认：框贴在身体/头上，类别名正确，远处小目标也能检出。
3. 调 `vision.conf`（默认 0.35）与 `vision.iou`（默认 0.45）：漏检多降 conf，重复框多降 iou；一次只改一个。
4. `vision.roi="center"/"target"` 只在目标始终在画面中心附近时使用；ROI 会改变 `Transform`，但反变换共用同一对象，不需要额外标定。

---

## 2. 头部瞄准点（head point）

### 2.1 有 head 类

`TargetPolicy._fuse` 与 `_aim_point`（`control/policy.py`）：

- 每个 head 框尝试配对一个 body 框；条件：
  - `head.area < body.area * 1.4`（head 不能比 body 还大）；
  - `body_box.contains(head.cx, head.cy, margin=-0.02)`（head 中心在 body 框内，允许 2% 外扩）。
- 配对成功：目标用 body 的身份（tid/速度来自 body），瞄准点用 **head 框中心**，`is_head=True`。
- 配对失败：head 作为独立目标，瞄准点仍是 head 中心。
- 选靶打分：`score = dist - head_bonus`（默认 28 px），`prefer_head=True` 时头部目标优先。

因此有 head 类时**不需要** `policy.head_ratio`，也不要用它去微调；改 `head_bonus` 只影响「选谁」，不影响「瞄哪里」。

### 2.2 只有 body 类

`_aim_point` 对非 head 目标返回：

```python
(box.cx, box.y1 + box.h * policy.head_ratio)
```

- 默认 `policy.head_ratio = 0.11`。
- 当前 `config.json` 迁移后的值是 `0.1386`（v1 `HEAD_RATIO`，docs/MEASUREMENTS.md §4 记录 ~0.139）。
- 只有身体类的模型锁头靠这个比例，**必须现场用红十字校准**；常见区间 0.08–0.15（见 `docs/MEASUREMENTS.md`）。

为什么不能用一个固定值：

- 不同模型对「body 框」的定义不同（是否含头/头发/肩）；
- 相机高度、FOV、目标距离会改变头顶在框内的相对位置；
- 分辨率变化会改变框的像素高度，但比例理论上不变；换模型/换游戏必须重标。

### 2.3 自动估计（有 head + body 的录制）

```bash
# 1) 录一段 head/body 同时出现的会话（必须用带 head 类的模型）
PYTHONPATH=src .venv/bin/python -m dma.cli record --duration 60 --dry-run --no-hotkeys

# 2) 用录制里的 head/body 配对估计中位数比例
PYTHONPATH=src .venv/bin/python -m dma.cli calibrate head --path out/telemetry/session_YYYYmmdd_HHMMSS.jsonl.gz
```

`tuning/calibration.py::head_ratio_from_session`：

- 只统计「head 中心落在 body 框内」的配对；
- 需要 ≥20 个样本，否则返回 `None`；
- 输出 `head_ratio` 建议和中位数样本数；
- 命令会打印 `aim2 config set policy.head_ratio X` 的提示。

注意：`SessionRecorder.frame` 记录的是**类别过滤后**的检测；如果 `head` 不在 `vision.target_classes`，录制里就没有 head，无法估计。做这个标定时先把 `head` 放进 target，标定后再按需要调整。

### 2.4 手动校准（模型无 head 类）

1. 打开 v1 的 PySide6 GUI（`qt_gui.py`，`./run_mac.sh gui` 或 `aim`），它提供预览 + 红十字叠加（docs/MEASUREMENTS.md §6）。
2. 站到训练场，让敌人（或假人）出现在中距离，保持静止。
3. 拖 `HEAD_RATIO`，直到红十字落在头部中心；记录值。
4. 换近距离/远距离各验证一次；如果同一个比例在远近都偏，说明模型框定义随距离变化，取中间值并接受误差。
5. 写入 v2：

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli config set policy.head_ratio 0.12
```

6. `aim2 config get policy.head_ratio` 复核；重启 `aim2 run`（`policy` 支持热加载，但重启最干净）。

### 2.5 头部标定验收

| 检查 | 通过标准 |
|---|---|
| 静止目标 | 准星稳定落在头部中心，不上下漂 |
| 左右横移 | 瞄准点在头部，不落到肩膀/胸口 |
| 远距离 | 框变小后仍在头部；误差随距离线性缩小 |
| 开火 | `fire.reason` 能到 `fire`，不是长期 `off-target`（内缩框 `inset` 默认 0.06，瞄脖子也可能被接受，见 §5） |
| 多目标 | 不因 head/body 重复框而两个 tid 抢锁 |

---

## 3. 灵敏度与增益标定

### 3.1 为什么必须标

世界坐标系的前提是：

```
1 px 控制输出  ==  1 px 画面平移
```

`MotionPlanner.add`（`transport/planner.py`）把控制器输出的 `dx` 乘以 `output.gain`，再按 HID counts 发送。游戏把 counts 转成画面像素的比例由「游戏内灵敏度 × DPI」决定。如果这个比例不是 1：

- 世界系 `world = source + cam` 的尺度错了；
- 跟踪器看到的目标速度被放大/缩小；
- 前馈 `v * apply_s` 和 `lead` 都错；
- 表现是「高增益震荡」或「怎么调都跟不上」，但根因不在控制律。

### 3.2 定义

设：

- `N`：我们发送的 HID counts；
- `P`：画面中一个静止特征移动的像素数（source 像素）；
- `k = N / P`：每像素需要的 counts（counts-per-pixel）。

`MotionPlanner.add` 发送 `gain * dx` counts。要让它等于 `k * dx`，必须：

```
output.gain = k = N / P
```

默认 `output.gain=1.0`（dataclass）；当前 `config.json` 的 `SINK_GAIN=3.0` 迁移后是 3.0。v1 的经验是「>1.5 易震荡」（docs/MEASUREMENTS.md §6），但那是在没有世界系/延迟补偿的 v1 里；v2 仍建议从标定值出发，不要凭手感拉满。

### 3.3 测量步骤

1. 进入训练场，面对一面有清晰纹理/直角的墙，准星对准一个固定特征。
2. 关闭瞄准（避免控制器插入位移），保持游戏内灵敏度不变。
3. 发送一个已知 counts 的测试位移。可以用下面的一次性脚本（不写入仓库）：

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python - <<'PY'
import time, serial
from dma.transport import protocol as P

PORT = "/dev/cu.usbmodemREPLACE"   # 用 aim2 doctor 看到的端口
N = 200                             # 发送 counts
with serial.Serial(PORT, 921600, timeout=0.01, write_timeout=0.05) as s:
    s.write(P.encode_move(N, 0, 1)); time.sleep(0.2)
    s.write(P.encode_move(-N, 0, 2))  # 回到原位，便于重复
    time.sleep(0.2)
print("sent", N)
PY
```

> 如果固件还是 v1 协议，把 `P.encode_move` 换成 `P.legacy_move`；`output.protocol` 只影响 Engine 的发送，不影响这个脚本。

4. 用采集画面（或 `aim2 run --dry-run` 的预览）量出特征在 x 方向的位移 `P` 像素。重复 3 次取平均。
5. 计算 `k = N / P`，写入：

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli config set output.gain 1.00   # 例：k=1.0
```

6. 复测：再发 `N=200`，位移应约为 200 px（±5%）。如果差得多，检查：
   - 游戏内鼠标加速/平滑是否关闭；
   - DPI 是否被驱动软件改过；
   - 采集分辨率是否与游戏原生一致（不一致时 `P` 的像素尺度不是游戏像素）；
   - 是否在游戏菜单/地图界面（灵敏度可能不同）。

### 3.4 验收与常见错误

| 现象 | 含义 | 处理 |
|---|---|---|
| 发 200 counts 画面只动 100 px | `k=2`，当前 gain 太小 | `output.gain=2.0` |
| 发 200 counts 画面动 400 px | `k=0.5`，当前 gain 太大 | `output.gain=0.5` |
| 准星移动但画面不动 | 串口/固件/HID 没生效 | `aim2 doctor`、`device.rx_frames` |
| 移动方向相反 | 坐标符号/游戏设置 | 检查协议 dx 符号与游戏 Y 轴；不要在控制器里翻符号 |
| 标定后仍震荡 | 延迟估计/跟踪问题 | 先做 §4，再降 `kp_far` |

标定完成后，**不要再改游戏内灵敏度/DPI**；改了就要重标 `output.gain`，并重跑延迟标定（世界系尺度变了）。

---

## 4. 延迟标定

### 4.1 目标

让 `LatencyEstimator` 收敛到当前链路的真实 `loop_s`，并确认 `confidence` 足够高，再动增益。

### 4.2 流程（实机，推荐）

1. 确认 `aim2 doctor --dry-run` 通过，模型能检出目标。
2. 进入训练场，让目标（假人/敌人）持续横移；我们也保持瞄准开启，让控制器产生位移。
3. 录制 60–90 秒：

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli run --record --duration 90 --aim --no-hotkeys --quiet
```

4. 结束后 CLI 打印 `describe()`，其中：

```json
"latency": {"loop_ms": ..., "frames": ..., "confidence": ..., "measure_ms": ..., "apply_ms": ..., "lead_ms": ...}
```

5. 判定（`tuning/calibration.py::latency_from_estimator`）：

| 条件 | 判定 |
|---|---|
| `confidence >= 0.35` 且 `samples >= 30` | 可信 |
| `0.2 <= confidence < 0.35` | 勉强，继续采集；不要据此调大增益 |
| `confidence < 0.2` | 不可信；`loop_s` 仍是先验/上次值 |

6. 复核：`frames × 1000 / capture.fps ≈ loop_ms`。60 fps 下 `frames=1` → 约 16.7 ms；`frames=2` → 约 33.3 ms。
7. 可选：把可信值写入先验，缩短下次收敛时间：

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli config set aim.latency_prior_s 0.017
```

但**不要**关掉估计器（没有开关；把 `latency_min_conf` 调高只会让它更少更新）。先验只是起点，运行时会继续更新。

### 4.3 流程（视频/离线）

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python - <<'PY'
from dma.core.config import load
from dma.capture.sources import VideoSource
from dma.engine.runtime import Engine
from dma.transport.link import MockLink
from dma.tuning.calibration import wait_for_latency

s = load()
eng = Engine(s, source=VideoSource("videos/REPLACE.mp4"), link=MockLink(s.output))
eng.start()
try:
    print(wait_for_latency(eng, seconds=30))
finally:
    eng.stop()
PY
```

要求视频里有模型能检出的目标，且目标在动。静止画面估不出延迟（见 §4.5）。

### 4.4 关于 `aim2 calibrate latency`

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli calibrate latency --seconds 8
```

它内部用 `SyntheticSource`（`cli.py::cmd_calibrate`），合成方块通常不在模型训练分布里，实测会得到 `samples=0 / confidence=0 / loop_ms=33.0`。把它当「Engine + 估计器能跑」的冒烟测试，不要当标定结果。

### 4.5 置信度低的典型原因

| 原因 | 现象 | 处理 |
|---|---|---|
| 目标/准星都不动 | 误差平坦，所有 lag 的 cost 接近 | 让目标横移，自己也跟枪 |
| 目标频繁丢失 | `on_detection` 样本不连续，`frames` 跳变 | 先修跟踪/选靶（`policy`/`tracker`），再做延迟 |
| 检测抖动大 | 残差被噪声主导 | 提高 `tracker.meas_noise` 或降 `vision.conf` 误检；确认模型稳定 |
| 采集帧率不稳 | `frame_dt` 中位数不可信 | 查 `capture.interval_ms` p95、游戏帧率、采集卡 |
| 世界系尺度错 | 自己的位移被错误放大/缩小 | 先做 §3 增益标定 |
| 延迟超过 `max_lag` | 估计器只能到 10 帧 | 60 fps 下 10 帧=167 ms；正常链路不应超过；查采集卡/显示 |

---

## 5. 开火门限与头部标定的关系

`FireController.update`（`control/fire.py`）当前判定：

```python
inner = preview.box.shrink(min(0.45, max(0.0, cfg.inset)))
inside = inner.contains(*preview.impact_xy)
near   = preview.residual <= cfg.radius
if not (inside or near):  # 两个条件满足一个即可
    ...
```

- `fire.inset` 默认 0.06：框内缩 6%，越靠边缘越保守。
- `fire.radius` 默认 12 px：即使命中点不在内缩框内，只要残差 ≤12 px 也允许开火。
- 这两个是「或」关系，不是「与」。如果瞄的是脖子（`head_ratio` 偏大），命中点可能在内缩框上缘之外但在 `radius` 内，仍能开火；如果两个都不满足，`fire.reason` 会是 `off-target`。
- `fire.dwell_s=0.04`、`cooldown_s=0.12`、`tap_s=0.03` 决定「稳定多久/多快连发/按多久」；标定头部时先保持默认，不要用开火门限去补偿瞄准点偏差。
- `fire.max_sigma=80 px`：Kalman 位置不确定度超过就不开火；目标刚出现/刚切换时常见 `uncertain`，属于正常。

---

## 6. 验证清单

按顺序执行；每项都要留记录（命令输出/截图/会话摘要）。

| # | 检查 | 命令/观察点 | 通过标准 |
|---|---|---|---|
| 1 | 配置合法 | `aim2 config validate` | 输出「配置合法」；无 warnings 未处理 |
| 2 | 模型可加载 | `aim2 doctor --dry-run` | `model ok=true`，`classes` 有 target |
| 3 | 类别正确 | `aim2 models inspect ... --json` | `targets` 不含 teammate/ai；`heads` 符合预期 |
| 4 | 串口可开 | `aim2 doctor`（非 dry-run） | `serial ok=true`，端口正确 |
| 5 | 采集有帧 | `aim2 run --duration 10 --dry-run` | `health=ok`，`capture` 30–60+ fps（游戏中 60） |
| 6 | 推理耗时 | 同上 / `perception.infer_ms` | p95 与docs/MEASUREMENTS.md 4–6 ms 同量级（320 模型） |
| 7 | 增益标定 | §3 脚本 | 发 200 counts，画面移动 200±10 px |
| 8 | 世界系跟踪 | `aim2 run --dry-run` + UI | 自己横移时目标框在世界系不跟着漂；`tracker.count` 稳定 |
| 9 | 头点 | §2 | 静止/横移/远近都在头部 |
| 10 | 延迟可信 | `aim2 run --record` 的 latency | `confidence>=0.35`、`samples>=30`、`frames` 稳定 |
| 11 | 无震荡 | 实机 30 秒跟枪 | 不出现周期性来回过冲；`aim.saturated` 不持续增长 |
| 12 | 开火合理 | `fire.reason` 分布 | 大部分 `fire`/`settling`；不出现 `uncertain` 长期占优 |
| 13 | 链路健康 | `link.status()` / `device.*` | `errors=0`、`reconnects=0`、`crc_errors=0`、`queue_drops=0` |
| 14 | 日志落盘 | `out/logs/aim.jsonl` | 有启动/健康/参数事件；无反复 error |
| 15 | 回放可复现 | `aim2 replay SESSION --compare` | 两个控制器跑同一会话，输出行可比 |

---

## 7. 什么时候必须重标

| 变更 | 头点 | 增益 | 延迟 |
|---|---|---|---|
| 换模型 | 是（head 类/框定义变了） | 否 | 可能（推理耗时变） |
| 换游戏分辨率/采集卡 | 可能（框尺度） | 是（像素尺度） | 是（采集卡延迟） |
| 改游戏内灵敏度/DPI | 否 | **是** | 是（世界系尺度变） |
| 改固件/HID 报告率 | 否 | 可能 | 是（应用延迟） |
| 改显示模式（60↔120 Hz） | 否 | 否 | 是（帧龄/输入 tick） |
| 换机器/换 USB 口 | 否 | 否 | 是（串口/驱动） |

每次重标前先备份当前配置：

```bash
cp configs/settings.json configs/settings.$(date +%Y%m%d_%H%M%S).json 2>/dev/null || true
PYTHONPATH=src .venv/bin/python -m dma.cli config show > out/config_before_$(date +%Y%m%d_%H%M%S).txt
```

（`configs/settings.json` 不存在时说明还在用 v1 `config.json` 迁移；先 `aim2 config migrate` 生成基线，再改。）

---

本文档对应 schema_version 2 与 src/dma 布局，改动代码请同步更新。
