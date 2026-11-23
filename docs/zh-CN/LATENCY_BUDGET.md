# 延迟预算（LATENCY_BUDGET）

<div align="right">

**简体中文** | [English](../LATENCY_BUDGET.md)

</div>

> ⚠️ 仅供学习与研究，请勿用于任何竞技/排位场景。见 [DISCLAIMER](../../DISCLAIMER.md)。

> 代码基线：`src/dma`，schema 2。默认值取自 `src/dma/core/config.py`。
> 「实测」只指数值来自 `docs/MEASUREMENTS.md` 或运行时字段；「推断」表示由帧率、配置或代码路径推导，未在台架上单独测量；「默认」表示配置默认值，不是测量值。

---

## 1. 环路定义

被控对象（`control/controller.py` 模块 docstring）：

```
e(t) = p(t - L_measure) - c(t - L_apply)
```

- `p`：目标位置（屏幕像素，准星为原点）。
- `c`：相机位置（我们累计发出的位移，屏幕像素）。
- `L_measure`：从「画面真实产生」到「控制器拿到检测结果」的延迟。
- `L_apply`：从「控制器发出命令」到「画面/准星真正移动」的延迟。
- `L = L_measure + L_apply`：闭环总延迟，在线辨识的对象。

`AimCfg.latency_split`（默认 `0.5`）把总延迟拆成两半：

| 字段 | 含义 | 默认 | 代码用法 |
|---|---|---|---|
| `latency_split` | 测量侧占比 | 0.5 | `measure_offset = split * loop_s`；`age_s += split*loop_s`；`apply_s = max(dt, (1-split)*loop_s)` |
| `latency_prior_s` | 收敛前先验 | 0.033 s | `LatencyEstimator(prior_s=...)` |
| `latency_window` | 辨识窗口 | 45 个检测样本 | `deque(maxlen=window)` |
| `latency_min_conf` | 更新门限 | 0.5 | `conf >= min_conf` 才平滑更新 `loop_s` |

`Engine.step` 里三个时间量的关系：

```
loop_s      = latency.loop_s
det_age     = now - det.capture_ts
age_s       = det_age + split*loop_s          # 目标外推到「现在」
apply_s     = max(dt, (1-split)*loop_s)       # 命令生效窗口
lead_s      = loop_s + det_age                # 开火预览用的总前视
```

三者的关系：

```
age_s + apply_s = det_age + loop_s = lead_s        （apply_s 未触发 max(dt, ...) 下限时）
```

`apply_s` 在控制频率很低（`dt > (1-split)*loop_s`）时会以 `dt` 为准，此时右侧会略大；正常 240 Hz 下不会触发。

---

## 2. 端到端分解

### 2.1 画面 → 检测

| # | 环节 | 代码位置 | 典型量 | 依据 | 观测方式 |
|---|---|---|---|---|---|
| 1 | 游戏渲染 + 显示扫描输出 | 游戏/HDMI，不在本仓库 | 0–1 个显示帧；游戏中 60 Hz → 0–16.7 ms | **推断**：docs/MEASUREMENTS.md（实测）「游戏把 HDMI 锁 60 Hz」；帧间隔由 `capture.fps` 观测 | `FrameGrabber.fps`、`capture.interval_ms` |
| 2 | HDMI → 采集卡信号 + 卡内缩放/编码 | 采集卡固件 | 1–5 帧（60 Hz 下 16.7–83 ms） | **实测/经验**：docs/MEASUREMENTS.md §7「采集卡固有信号延迟 1~5 帧；对齐分辨率可少 1~2 帧；MJPG 比 YUYV 多编解码延迟」 | 无法从主机侧单独读出；换卡/换分辨率做 A/B |
| 3 | 驱动缓冲 + `cv2.read` | `capture/sources.py::CvSource.read`、`capture/grabber.py::_run` | `buffer_size=1` 时 0–1 帧 | **默认 + 推断**：`capture.buffer_size=1`；采集线程只保留最新帧 | `grabber.stale_s()`、`capture.read_errors` |
| 4 | 帧龄（检测帧到控制 tick） | `Engine.step` 的 `det_age` | 0–1 个采集间隔；60 Hz 平均 ~8.3 ms，最坏 ~16.7 ms | **实测字段**：`snapshot.latency["frame_age_ms"]` | HUD/`latency.frame_age_ms` |
| 5 | 预处理（裁剪/letterbox/归一化） | `perception/detector.py::_preprocess` | 未单独给出基线 | **实测字段**：`perception.pre_ms`；无台架标称值 | 指标 p50/p95 |
| 6 | ONNX 推理 | `OnnxDetector.detect` → `sess.run` | 320 模型 4.1–5.0 ms p50；640 V5 7.4 ms；640 e2e 16.6 ms | **实测**：本机 CoreML、2560×1600 输入帧、`aim2 models bench`（`docs/MEASUREMENTS.md`） | `perception.infer_ms`、`aim2 models bench` |
| 7 | 解码 + NMS + 类别过滤 | `decoders.*`、`detector.detect` | 未单独给出基线 | **实测字段**：`perception.post_ms`；纯 numpy，随检测数增长 | 指标 p50/p95 |
| 8 | 跟踪 + 选靶 + 控制律 | `perception/tracker.py`、`control/policy.py`、`control/controller.py` | 不超过一个控制周期（默认 4.17 ms） | **推断**：控制线程固定 240 Hz，`Engine.step` 内同步执行；没有单独计时 | `control.hz`、`Snapshot.top_line` |

### 2.2 输出 → 画面

| # | 环节 | 代码位置 | 典型量 | 依据 | 观测方式 |
|---|---|---|---|---|---|
| 9 | 位移累加 + 分帧 | `transport/planner.py::add/take` | 按 `report_hz=500` → 2 ms/报告 | **默认**：`output.report_hz=500` | `planner.report()` 的 reports/clipped/residual |
| 10 | 串口写 | `transport/serial_link.py::_write` | 9 字节帧（4 头 + 4 payload + CRC）@921600 bps；理论 ~0.098 ms/帧；实测链路能力 9400 帧/秒 | **实测**：docs/MEASUREMENTS.md「串口实测 9400 帧/秒」；v2 帧更短 | `transport.write_ms` p95、`transport.write_errors` |
| 11 | ESP32 解析 + HID 报告排队 | 固件 | 未测量 | **推断**：取决于固件队列；`STATUS.queue_drops` 可观测丢包 | `device.queue_drops`、`device.report_hz` |
| 12 | USB HID 轮询 + 游戏输入 tick | Windows/游戏 | 0–1 个游戏帧；60 fps → 0–16.7 ms | **推断**：游戏在自己的输入 tick 应用 HID；docs/MEASUREMENTS.md 的 60 Hz 锁帧是同一来源 | 无法直接读；只能用闭环估计 |
| 13 | 闭环总延迟 `L` | `LatencyEstimator` | 实测 D=1 帧（v1 HUD，60 Hz ≈ 16.7 ms）；v2 先验 33 ms | **实测**：docs/MEASUREMENTS.md「实测控制延迟 D=1 帧」；33 ms 是 v2 先验默认值 | `snapshot.latency["loop_ms"/"frames"/"confidence"]` |

### 2.3 为什么不能把表里的数直接相加

- 采集、推理、控制是**流水线**：推理一帧的 4–8 ms（320/V5）与下一帧的采集重叠；`frame_age_ms` 已经包含「这一帧等控制线程」的时间。
- 表中的「采集卡 1–5 帧」「游戏输入 0–1 帧」「显示 0–1 帧」是**区间**，最坏情况不会同时发生。
- 因此工程上只认两个数：
  1. **单段可观测值**（`infer_ms`、`write_ms`、`frame_age_ms`、`capture.fps`）；
  2. **闭环估计值** `loop_ms`（`LatencyEstimator`），它把显示、采集卡、HID、游戏输入这些主机侧看不到的段一起估出来。
- 不要用「推理 5 ms + 采集 16.7 ms + 串口 0.2 ms = 22 ms」去覆盖 `loop_ms`；这个和只说明各段量级，不是闭环延迟。

---

## 3. 实测与推断清单

| 结论 | 类型 | 来源 | 备注 |
|---|---|---|---|
| 游戏中采集 60 fps（桌面可 120） | 实测 | docs/MEASUREMENTS.md | 原因是游戏把 HDMI 输出锁 60 Hz（多为独占全屏）；无边框全屏 + 不限帧 + 关垂直同步才可能 120 |
| 控制延迟 D=1 帧 | 实测 | docs/MEASUREMENTS.md | v1 GUI HUD 的「实测延迟 D」；60 Hz 下约 16.7 ms |
| 推理 4.1–5.0 ms p50（320 模型） | 实测 | `aim2 models bench`（`docs/MEASUREMENTS.md`） | v2 用 `perception.infer_ms` 的 p50/p95 复核 |
| 串口 9400 帧/秒 | 实测 | docs/MEASUREMENTS.md | v1 10 字节帧；v2 9 字节帧，量级相同 |
| 采集卡信号延迟 1–5 帧 | 实测/经验 | docs/MEASUREMENTS.md §7 | 分辨率不对齐会多 1–2 帧；MJPG 比 YUYV 多编解码延迟 |
| 60 fps 甩枪极限约 10 帧（167 ms） | 实测/经验 | docs/MEASUREMENTS.md | 120 fps 同帧数约 83 ms；提帧率是唯一质变路径 |
| 显示/游戏输入各 0–1 帧 | 推断 | 60 Hz 帧间隔 | 不是独立测量；用 `loop_ms` 覆盖 |
| 预处理/NMS/跟踪/选靶耗时 | 推断/字段 | 代码路径 | 有指标字段，但没有台架标称值；不要编数 |
| HID 报告 2 ms | 默认 | `output.report_hz=500` | 实际报告率看 `device.report_hz` |
| ESP32 队列/固件延迟 | 未测量 | 固件 | `device.queue_drops` 是唯一线索 |

推理段分模型实测（本机 CoreML，2560×1600 输入帧，`aim2 models bench`；见 `docs/MEASUREMENTS.md`）：

| 模型 | 格式 | p50 | p95 |
|---|---|---|---|
| YOLOv8 320×320 导出 | YOLOv8（head/teammate/body） | 4.1 ms | 7.9 ms |
| 端到端 320×320 导出 | 端到端 | 5.0 ms | 6.3 ms |
| YOLOv5 640×640 导出 | YOLOv5 | 7.4 ms | 8.2 ms |
| 端到端 640×640 导出 | 端到端 | 16.6 ms | 20.6 ms |

读法：换 sjz 不会因为 320 输入而变慢；640 的 V5 仍在 8 ms 级，640 的端到端则到 16–20 ms。推理 p95 超过 20 ms 时，`frame_age_ms` 和 `max_infer_age_s=250 ms` 的余量会被明显吃掉（640 e2e 的 p95 已到 20.6 ms），要优先换 320 模型或降 ROI。

> 报告条件是 2560×1600 输入帧；`aim2 models bench` 当前 CLI 用随机 1280×720 帧，预处理/缩放不同，数值会略有差异。用同一命令、同一输入尺寸做前后对比才有意义。

FPS 与帧时间换算（只用于把「帧」换成毫秒）：

| 采集/显示帧率 | 1 帧 |
|---|---|
| 60 Hz | 16.7 ms |
| 120 Hz | 8.3 ms |
| 240 Hz | 4.17 ms |

---

## 4. 怎么读延迟

### 4.1 `aim2 doctor`

```bash
cd ~/dma
PYTHONPATH=src .venv/bin/python -m dma.cli doctor --dry-run
PYTHONPATH=src .venv/bin/python -m dma.cli doctor --dry-run --probe --json
```

- `doctor` 检查 python/onnxruntime/opencv/pyserial/模型/类别/串口（`cli.py::cmd_doctor`）。
- `--probe` 用一张合成图跑 `OnnxDetector.bench`，输出 `p50_ms/p95_ms/fps`；这是**推理段**的基准，不是闭环延迟。
- 模型无 head 类、`head_ratio` 可疑、`report_hz<125`、`control_hz<120`、`latency_prior_s<=0` 会出现在「建议」里（`tuning/calibration.py::suggest`）。
- **doctor 不测闭环延迟**；闭环延迟只能在有目标的实时回路里估。

### 4.2 HUD / Snapshot 的「实测延迟」

`Engine._publish` 的 `Snapshot.latency` 字段（`LatencyEstimator.report`）：

| 字段 | 含义 | 读法 |
|---|---|---|
| `loop_ms` | 闭环总延迟估计，毫秒 | 稳定后与 `frames × 1000/采集fps` 对照 |
| `frames` | 延迟折合多少个**检测帧** | 60 fps 下 `frames=1` ≈ 16.7 ms；**不是控制 tick 数** |
| `confidence` | 辨识置信度 0–1 | 低于 `latency_min_conf`（默认 0.5）不更新，只有先验；`>=0.35` 且 `samples>=30` 才算可信（`calibration.latency_from_estimator`） |
| `measure_ms` | `loop_ms * split` | 画面陈旧部分 |
| `apply_ms` | `loop_ms * (1-split)` | 命令生效部分 |
| `lead_ms` | 等于 `loop_ms` | 用于显示「前视总量」 |
| `stage_infer_ms` | 当前检测帧推理耗时 | 与 `perception.infer_ms` 对照 |
| `frame_age_ms` | 控制 tick 时检测帧的年龄 | 60 fps 下应在 0–17 ms 内，偶发更大要查调度 |

v1 GUI 的「实测延迟 D」是同一类估计的早期版本（HUD 上按帧显示）；v2 的 `frames` 语义相同，但多了置信度和毫秒值。**不要拿 v1 的 D 直接填进 `aim.latency_prior_s`**：v1 的 D 是帧计数，v2 的先验是秒，且 v2 默认 33 ms 是保守值（见 §5.1）。

桌面端 HUD（`desktop/app.py::MainWindow._tick`）把同一组字段压成一行：

```
cap 60.0Hz  ctl 240.0Hz  inf  4.7ms   err  12.3px  mode TRACK  lat  33.0ms(conf 0.00)   det 1  trk 1  fire settling
```

- `lat` 是 `snap.latency["loop_ms"]`，括号里是 `confidence`；`conf 0.00` 表示辨识器还没学到东西（没有移动目标/没开瞄准），不是「延迟 0」。
- `inf` 是当前检测帧的 `infer_ms`，不是 `loop_ms`。
- `err` 是控制误差，不是延迟；不要用 `err` 判断延迟。
- v1 的「实测延迟 D」只在 `qt_gui.py` 的旧 HUD 里；v2 桌面端/CLI 用 `loop_ms + confidence`。

### 4.3 标定命令的真实能力

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli calibrate latency --seconds 8
```

`cmd_calibrate("latency")` 用 `SyntheticSource` 跑 Engine，再 `wait_for_latency`。合成画面里的方块**不是**模型训练分布里的目标，实测常常 `samples=0`、`confidence=0`，`loop_ms` 停在先验 33 ms。因此：

- 把它当**冒烟测试**（Engine 能启动、延迟估计器不崩）；
- 真正标定要用有目标的视频或实机：见 [CALIBRATION](CALIBRATION.md#4-延迟标定)。

有目标的离线用法（视频文件）：

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

实机用法：`aim2 run --record --duration 90`，结束后 CLI 打印的 `describe()` 里有 `latency`；录制文件 `out/telemetry/session_*.jsonl.gz` 的 `tick` 记录里也带 `lat.loop_ms/confidence/frames`。

### 4.4 置信度怎么算

`control/latency.py::_estimate`：

1. 取最近 `latency_window`（默认 45）个检测事件，每个事件有屏幕误差 `(ex,ey)` 和这期间累计发出的位移 `(ux,uy)`。
2. 对候选 lag `m ∈ [0, max_lag]`（`max_lag` 默认 10，且不超过 `n-8`），计算残差中位数：
   `cost(m) = median(|Δex_k + ux_{k-m}| + |Δey_k + uy_{k-m}|)`。
3. 取 cost 最小的 `m`；`confidence = 1 - best_cost / median(all_costs)`，钳位 0–1。
4. `conf >= latency_min_conf` 时，`loop_s` 以 `smoothing=0.25` 向 `m * frame_dt` 平滑；否则置信度再乘 0.5（表示「这次没学到东西」）。
5. `frame_dt` 是检测间隔的中位数；所以 `frames` 是检测帧数，`loop_ms = frames * frame_dt`。

读法：

- 目标静止、我们也不动 → 误差曲线是平的，所有 lag 的 cost 接近，`confidence` 低，`loop_ms` 停在上次值/先验。**必须让目标和准星都在动**。
- 目标频繁丢失/切换 → `on_detection` 的样本不连续，`frames` 会漂；先解决跟踪/选靶，再看延迟。
- 想复核：`frames × 1000 / capture.fps` 应约等于 `loop_ms`（两者都来自检测间隔，理论上一致）。

---

## 5. 延迟与增益的稳定边界

### 5.1 被控对象与「死拍」增益

把屏幕误差看成「我们自己输出的积分」，延迟为 `L`：

```
e_{k+1} = e_k - u_{k-m} + Δp_k        （m = L / 采样周期）
```

对纯延迟积分器，理想控制是**死拍**：一次把误差打完，即 `u = e`（增益 1）。增益 1 同时是稳定边界：任何大于 1 的等效增益都会在延迟下过冲，形成极限环；小于 1 则稳定但收敛慢。

`PredictiveAim` 的设计增益：

| 参数 | 默认 | 作用区间 |
|---|---|---|
| `kp_near` | 0.50 | 误差 < `near_px=26` |
| `kp_far` | 0.85 | 误差 > `far_px=260` |
| 过渡 | `smoothstep(26, 260, dist)` | 26–260 px 之间平滑 |

这与 `control/controller.py` docstring 里写的「k in [0.42, 0.75]」不一致；**以 dataclass 默认值为准**（0.50 / 0.85）。默认值已经接近但低于 1，留了约 15% 的相位/延迟误差余量。

### 5.2 速率归一化

`PredictiveAim.step` 的第一行：

```python
scale = clamp(dt / max(1e-3, apply_s), 0.02, 1.0)
u = scale * (kp*e + ki*I + ff) + kd*(e - e_prev)
max_u = max_step * scale
```

为什么必须乘 `scale`：

- 控制律的语义是「在命令生效窗口 `apply_s` 内应该走多少像素」，不是「每个 tick 走多少像素」。
- v1 的增益是在 60 Hz 循环里手调的；同一组 `kp/max_step` 直接搬到 240 Hz，**每秒位移会变成 4 倍**，延迟下必然震荡。这正是docs/MEASUREMENTS.md「高增益 + 延迟 → 过冲极限环」的机制。
- 乘 `scale` 后，`kp` 的物理意义与循环频率解耦：60/120/240 Hz 下每秒的最大位移一致。

默认参数下的数值（`loop_s=33 ms, split=0.5, apply_s=16.5 ms, max_step=140`）：

| 控制频率 | `dt` | `scale = dt/apply_s` | 单 tick 最大位移 `140*scale` | 每秒最大位移 |
|---|---|---|---|---|
| 60 Hz | 16.7 ms | 1.0（钳位） | 140 px | ≈ 8.4 k px/s |
| 120 Hz | 8.33 ms | 0.505 | 70.7 px | ≈ 8.5 k px/s |
| 240 Hz | 4.17 ms | 0.253 | 35.4 px | ≈ 8.5 k px/s |
| 500 Hz | 2.0 ms | 0.121 | 17.0 px | ≈ 8.5 k px/s |

（每秒值 = 单 tick × 频率；四舍五入差异来自 `scale` 的钳位。）

### 5.3 增益上限怎么定

工程规则（不是测量值，是从上面的模型推出来的）：

1. **等效环路增益 = kp**（在 `scale` 归一化后）。要稳定，必须 `kp < 1`；默认 `kp_far=0.85` 已接近上限。
2. **延迟估计误差会放大等效增益**：如果实际 `L` 比 `loop_s` 大 `ε`，前馈/外推不足，误差表现为额外相位滞后，极限环更容易起振。置信度不足、仍在使用先验时按「延迟未知」处理，`kp_far` 不要再往上加。
3. **`max_step` 是第二道限幅**：它限制单 tick 位移，不改变环路增益；`max_step` 太小会让大误差也走不动（甩枪变慢），太大则饱和时前馈+积分一起冲。
4. **`lead_gain` 默认 0.9**：前馈只补偿 90% 的目标速度，留 10% 余量给速度估计误差；`lead_max=70 px` 防止一次坏速度估计变成巨大前馈。
5. **`latency_split` 影响的是分配，不是总量**：`split` 偏大 → 更多延迟被当成「画面陈旧」，外推更多；偏小 → 更多被当成「命令生效」，`apply_s` 更大、`scale` 更小。总量 `loop_s` 不变，但残差自运动不同；默认 0.5 是起点。
6. **`deadzone=1.5` / `deadzone_exit=5.0`**：死区有滞回，避免在 1.5 px 附近抖进抖出；`output_ema=0.9` 的群延迟约 `(1-α)/α * dt ≈ 0.46 ms @240 Hz`（代码注释），可忽略，但不要用它替代 `kd`。
7. **`max_accel=0` 默认关闭**：开启后是额外的斜率限制，量纲是 px/tick，且同样乘 `scale`；它会让大甩枪变慢，不能用来治震荡（治震荡先降 `kp`/`max_step`）。

### 5.4 延迟变大时怎么改

| 现象 | 先查 | 调整 |
|---|---|---|
| 目标附近来回过冲（极限环） | `loop_ms` 是否比上次大；`confidence` 是否低 | 降 `kp_far`（0.85→0.7→0.6）、降 `max_step`；不要先加 `kd` |
| 甩枪到位后回弹 | `aim.saturated` 计数、`fire.reason` | 降 `max_step` 或 `kp_far`；检查 `lead_max` 是否过大 |
| 跟枪持续滞后 | `loop_ms`、`lead_gain` | 提高 `lead_gain`（≤1.0）或 `ki`（在 `integral_gate` 内）；先确认延迟估计可信 |
| 静止时抖 | `output_ema`、`deadzone`、`tracker` 噪声 | 提高 `deadzone`/`deadzone_exit`，降低 `output_ema`（更多平滑），或提高 `kd` |
| 采集 fps 从 120 掉到 60 | `capture.fps`、游戏显示模式 | 先把游戏改成无边框全屏/不限帧/关垂直同步；采集卡分辨率对齐原生 |
| `frame_age_ms` p95 突然变大 | 控制线程是否被热加载/UI 阻塞 | 看 `control.hz`；热加载会重建控制器（见 [OPERATIONS](OPERATIONS.md#6-常见故障排查)） |

**不要**用提高 `max_step` 来补偿「延迟大」；延迟大时提高步幅只会把极限环放大。docs/MEASUREMENTS.md的结论是「D 越大，稳定允许的 `max_step/kp` 越小」。

---

## 6. 预算核算的推荐做法

1. 冷启动后先看 `aim2 doctor --probe` 的推理 p95，确认计算段没退化。
2. 跑 60–90 秒有移动目标的实机/视频，等 `confidence ≥ 0.35`、`frames` 稳定。
3. 记录 `loop_ms`、`frames`、`frame_age_ms` p95、`perception.infer_ms` p95、`transport.write_ms` p95。
4. 用 `frames × 1000/capture.fps` 复核 `loop_ms`；偏差大说明检测间隔不稳（掉帧/选靶跳变）。
5. 只有在 `loop_ms` 稳定后才动 `kp_far/max_step/lead_gain`；每次只改一个，改完用 `aim2 replay` 或实机 A/B。
6. 把结果写进 `docs/` 或 `out/telemetry` 的会话摘要，不要只留在脑子里。

---

本文档对应 schema_version 2 与 src/dma 布局，改动代码请同步更新。
