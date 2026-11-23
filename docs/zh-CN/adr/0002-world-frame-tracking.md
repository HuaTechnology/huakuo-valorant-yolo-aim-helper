# ADR-0002：在世界坐标系跟踪目标，而不是屏幕坐标系

<div align="right">

**简体中文** | [English](../adr/0002-world-frame-tracking.md)

</div>

- 状态：已接受（Accepted）
- 日期：2026-11-21
- 决策范围：`src/dma/core/clock.py`（DelayLine）、`src/dma/engine/runtime.py`、`src/dma/perception/tracker.py`、`src/dma/control/policy.py`
- 相关：ADR-0001（预测控制）、[ARCHITECTURE §4](../ARCHITECTURE.md#4-坐标系)、[CALIBRATION §3](../CALIBRATION.md#3-灵敏度与增益标定)

---

## 1. 背景

v1 在屏幕坐标里维护锁点：

```python
# main_cli.py（v1）
lock = (aim_x, aim_y)          # 屏幕像素
...
dx, dy = ctrl.update(aim_x, aim_y, cx, cy, dt)
```

屏幕误差 `e = p_screen - center` 的差分同时包含两部分：

```
Δe = Δp_enemy - Δc_self
```

- `Δp_enemy`：敌人自己的运动；
- `Δc_self`：我们发出的鼠标位移造成的画面平移。

后果：

1. **速度估计被污染**。任何基于 `Δe` 的速度/前馈/D 项都会把自己的输出当成目标运动。高增益时形成正反馈（「越追越抖」），低增益时滞后。
2. **延迟补偿没有基准**。要预测「命令生效时敌人在哪」，必须知道敌人相对世界怎么动；屏幕系里这个量随我们每一步变化。
3. **身份不稳定**。目标框在屏幕上一跳，锁点就跳；v1 用 `SWITCH_JUMP` 清积分、`COAST_FRAMES` 续命，都是补丁。
4. **回放不可复现**。只存屏幕框，换一个控制器就得到不同的自身运动，目标的表观轨迹也变了。

v1 的「D=1 帧」「高增益弹簧」「跟枪滞后」都可以归因到同一个结构问题：**控制器没有区分「目标在动」和「我们在动」**。

---

## 2. 决策

### 2.1 定义

```
world  = source + cam_meas        （画面产生时刻的相机位置）
screen = world  - cam_now         （当前看到的画面）
```

- `source`：采集帧像素坐标（`Detection.box`）。
- `cam`：自瞄准开启以来我们累计发出的位移（屏幕像素，`Engine.cam`）。
- `cam_meas`：`cam` 在**画面产生时刻**的采样值，来自 `DelayLine`。
- `world`：目标在世界里的位置（增量世界系，原点=瞄准会话开始）。
- `screen`：玩家看到的画面；准星恒在 `center=(w/2,h/2)`。

`Track.box`、`Track.vx/vy` 全部是 **world** 坐标（`core/types.py::Track` docstring）；`TargetPolicy.select` 在 world 里选靶，再转成 screen 做 FOV/遮罩/误差。

### 2.2 延迟线怎么取

`Engine.step` 每处理一个新检测帧：

```python
measure_offset = aim.latency_split * latency.loop_s
cam_meas = cam_line.sample(det.capture_ts - measure_offset)
world_dets = [d.box.translate(*cam_meas) for d in det.detections]
tracks = tracker.update(world_dets, det.capture_ts)
```

`DelayLine`（`core/clock.py`）：

- `push(now, cam_x, cam_y)`：控制线程在每次实际发出位移后调用；只在 `can_aim and target` 时 push。
- `sample(ts)`：在历史缓冲区里二分查找 + 线性插值；`ts` 超出两端时钳位到端点；缓冲区 `maxlen=512`，240 Hz 下约 2.1 秒历史。
- `span_s`：当前历史跨度，可用于诊断。

**为什么采样在 `capture_ts - measure_offset`，而不是 `capture_ts`**：

- `capture_ts` 是 `grabber` 读到这一帧的时刻，不是这一帧内容真正产生的时刻；采集卡/驱动/显示还有延迟。
- 画面内容对应的相机位置，比「读到帧」更早；`measure_offset` 是总延迟中「测量侧」的估计（默认 `0.5 × 33 ms = 16.5 ms`）。
- 如果直接在 `capture_ts` 取样，我们会把「帧产生之后」的自身运动加回去，世界系里仍残留一部分自运动；表现是目标速度被低估/高估，前馈方向错。

`latency_split` 的作用是分配，不是总量：默认 0.5 是起点；如果采集卡延迟大、推理快，可以把 `split` 调大（更多算作画面陈旧）；如果命令生效慢（HID/游戏输入 tick），调小。总 `loop_s` 不变。

### 2.3 世界系的生命周期

| 事件 | `cam` | `cam_line` | `tracker` | 说明 |
|---|---|---|---|---|
| 启动 | (0,0) | 空 | 空 | 原点 |
| 瞄准开启 | 保持 | 保持 | 保持 | 继续累计 |
| 瞄准关闭 | 重置 (0,0) | `reset()` | **不重置** | 控制器 reset；世界原点改变 |
| 画面尺寸变化 | 保持 | `reset()` | **不重置** | 同时 `latency.reset()`；见 §4 风险 |
| 目标丢失 | 不再累加 | 不再 push | coast 到 `max_age` | 世界系保留最后状态 |
| 进程重启 | (0,0) | 空 | 空 | 不持久化 |

**重要**：这是**增量世界系**，不是绝对地图。原点在最近一次「瞄准开启/关闭」或进程启动；玩家自己用手移动鼠标、后坐力、游戏镜头抖动都不在 `cam` 里，因此不会被补偿。它不是「地图坐标」，只是「把我们的命令从观测里扣掉」的参考系。

---

## 3. 为什么不用其他方案

| 方案 | 问题 |
|---|---|
| **屏幕系跟踪 + 减去命令速度** | 命令速度本身有延迟/量化；`Δe - u` 仍有相位误差，且需要知道 `u` 的生效时刻。等于把世界系的事后做一遍，但更容易不稳定 |
| **图像自运动估计（相位相关/光流）** | 游戏镜头平移时背景整体移动，理论上可估；但低纹理墙面/天空会失败，计算量在 4 ms 控制预算里不划算，且仍需要延迟对齐。可作为未来冗余，不作为主路径 |
| **IMU / 游戏 API / 驱动钩子** | 目标机器上不可用或不可依赖；跨平台成本高 |
| **完整单应矩阵** | 固定 FOV、无 roll 的 FPS 相机，yaw/pitch 近似为平移；单应会引入更多参数和标定，收益不足 |
| **背景特征点跟踪** | 视差、遮挡、动态物体干扰；实时性和鲁棒性都不如「已知命令位移」直接 |

---

## 4. 自运动补偿失败的后果

这是本 ADR 最重要的部分：世界系不是免费的，以下任一条件破坏，跟踪器都会把「我们的运动」当成「敌人的运动」。

| 失败模式 | 机理 | 现象 | 检测/缓解 |
|---|---|---|---|
| **输出增益未标定** | `MotionPlanner.add` 发 `gain*dx` counts；游戏把 counts 转成像素的比例不是 1 → `cam` 与真实画面位移尺度不一致 | 世界系速度被放大/缩小；前馈过冲或不足；高增益震荡、低增益拖尾 | 按 [CALIBRATION §3](../CALIBRATION.md#3-灵敏度与增益标定) 标定；`aim2 run --dry-run` 看横移时目标框是否漂 |
| **`latency_split` 错** | `cam_meas` 采样时刻不对 → 世界系里残留 `Δc_self` | 目标速度出现「我们一甩它就加速」的假象；延迟辨识 `confidence` 低/`frames` 漂 | 先用默认 0.5；对比不同 `split` 下的 `aim.err_px`/`confidence`；不要把它当增益调 |
| **`loop_s` 估计错** | `measure_offset` 与实际不符；`apply_s` 也错 | 外推/前馈方向偏；极限环 | 看 `confidence`；低于 0.2 时退回先验；先修采集/跟踪再估 |
| **延迟线被 reset** | `cam_line` 空，`sample` 返回 (0,0)，但 `tracker` 里的 world track 还是旧原点 | 下一帧世界坐标跳变；策略可能换靶/coast；开火误判 | 分辨率变化/瞄准开关后给 2–3 帧稳定期；必要时重启进程 |
| **瞄准关闭期间的世界漂移** | `cam` 重置为 0，但玩家手动移动了鼠标；重新开启后原点与旧 track 不一致 | 重新开启瞬间目标跳 | 关闭瞄准时同时清 tracker（当前代码未做，见 §5）或在 UI 上提示 |
| **未建模的自身运动** | 玩家手动鼠标、后坐力、镜头抖动、游戏镜头平滑/加速 | 世界系里敌人出现额外运动；长时间跟踪累积漂移 | 这是残差，不是 bug；靠 `tracker.accel_noise`/`meas_noise` 和短 coast 限制影响 |
| **采样超出历史** | `sample` 钳位到端点；控制线程长时间未 push（无目标/链路卡） | 世界系停更，目标位置冻结 | 看 `cam_line.span_s`；长时间无目标时不要 coast 太久 |
| **帧尺寸变化** | `cam_line.reset()`/`latency.reset()`，但 `tracker` 保留旧 world track | 一次跳变，可能误锁 | 改分辨率后重启；至少等策略重新选靶 |

### 4.1 为什么这些失败会「看起来像控制参数问题」

- 世界系尺度错 → 速度估计错 → 前馈错 → 表现和「`kp_far` 太高」一样；
- 采样时刻错 → 自运动漏进 tracker → 表现和「检测抖动大」一样；
- 延迟线 reset → 目标跳 → 表现和「换靶迟滞不够」一样。

所以排查顺序必须是：**先确认世界系/标定，再调控制参数**。这也是 [CALIBRATION](../CALIBRATION.md) 把增益标定放在延迟标定之前的原因。

---

## 5. 后果与遗留问题

### 5.1 正面

- 跟踪器速度/协方差只反映目标自身运动，前馈和开火预览才有意义。
- `Track.tid` 身份稳定；策略的 `switch_margin` 是真正的「换人迟滞」，而不是「屏幕跳变补丁」。
- 录制里存 `cam` 样本（`SessionRecorder.frame`），回放可以重建世界轨迹，换控制器不影响目标轨迹（`telemetry/replay.py`）。
- 仿真台可以把「自身运动」和「目标运动」分开注入，回归才有意义。

### 5.2 成本

- 必须标定 `output.gain`，否则世界系整体缩放。
- 必须在线估 `loop_s`/`split`，否则采样时刻错。
- 世界系是增量/局部的，不能跨瞄准会话复用；进程重启/分辨率变化会丢原点。
- 录制文件比屏幕系多存一个 `cam` 字段；回放要按世界系重建（当前 `replay.py` 还有类别硬编码问题）。

### 5.3 当前代码的已知缺口

| 缺口 | 影响 | 位置 |
|---|---|---|
| `set_aim_enabled(False)` 重置 `cam`/`cam_line`，但不清 `tracker` | 重新开启时旧 world track 与新原点不一致，可能跳一下 | `Engine.set_aim_enabled` |
| 帧尺寸变化只清 `cam_line`/`latency`，不清 `tracker` | 同上 | `Engine.step` 的尺寸变化分支 |
| `SyntheticSource` 需要 `set_camera` 才能闭环 | 合成画面里如果不回传相机位置，目标会「永远加速逃逸」；`Engine.step` 已通过 `hasattr(source, "set_camera")` 调用 | `capture/sources.py::SyntheticSource.set_camera` |
| 回放把类别名硬编码为 `body` | 无法复现 head 融合；世界轨迹本身仍可重建 | `telemetry/replay.py` |
| 玩家手动鼠标/后坐力未建模 | 世界系残差；长时间跟踪会漂 | 设计边界，不是短期 bug |

---

## 6. 代码位置与验证

| 内容 | 位置 |
|---|---|
| 世界系定义 | `core/types.py` 模块 docstring、`Track` docstring |
| 延迟线 | `core/clock.py::DelayLine.push/sample/span_s` |
| 采样与平移 | `engine/runtime.py::Engine.step`（`measure_offset`、`world_dets`） |
| 屏幕系选靶 | `control/policy.py::TargetPolicy.select`（`sx = wx - cam_x`） |
| 控制器误差 | `control/controller.py::PredictiveAim.step`（`ex = tx - cam_now - center`） |
| 开火预览 | `engine/runtime.py::Engine.step`（`ImpactPreview`） |
| 录制 | `telemetry/recorder.py::SessionRecorder.frame`（存 `cam` 样本） |
| 回放 | `telemetry/replay.py::replay` |
| 合成源自运动 | `capture/sources.py::SyntheticSource.set_camera` |

验证方法：

1. 实机 `aim2 run --dry-run`，自己左右横移，观察目标框在世界系是否稳定（UI 的 track 虚线框不应跟着你的移动整体漂移）。
2. 录一段横移 + 跟枪，`aim2 replay --compare`：世界轨迹应与你当时的鼠标运动无关；两个控制器应跑在同一目标轨迹上。
3. 改 `output.gain` ±10%，看 `aim.err_px`/`confidence` 是否显著变差——如果变差，说明世界系尺度对增益敏感，必须重标。
4. 改 `latency_split` 0.3/0.5/0.7，看 `confidence` 和 `aim.err_px`；选置信度高且误差小的值，不要凭感觉。

---

## 7. 何时重新评估

- 如果未来能从驱动/游戏 API 拿到精确的「输入生效时刻」或相机姿态，可以把 `cam` 从「命令累计」升级为「实测相机位移」，世界系会更准。
- 如果玩家手动鼠标占比很高（比如半自动武器点射时的手动修正），需要把物理鼠标输入也纳入 `cam`，否则世界系残差会变大。
- 如果游戏镜头有非线性平滑/加速，纯平移模型会失效，需要考虑带速度/加速度状态的相机模型。
- 如果采集分辨率/灵敏度在会话中变化，必须重置世界系与 tracker；当前只重置了延迟线。

---

本文档对应 schema_version 2 与 src/dma 布局，改动代码请同步更新。
