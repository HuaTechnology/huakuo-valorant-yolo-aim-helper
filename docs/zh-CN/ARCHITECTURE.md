# 架构说明（v2）

<div align="right">

**简体中文** | [English](../ARCHITECTURE.md)

</div>

> ⚠️ 仅供学习与研究，请勿用于任何竞技/排位场景。见 [DISCLAIMER](../../DISCLAIMER.md)。

> 代码基线：`src/dma`，配置 `schema_version = 2`，CLI 版本 `aim2 2.0.0`（`src/dma/core/__init__.py`）。
> 本文的默认值全部取自 `src/dma/core/config.py` 的 dataclass 默认值；标「实测」的数字来自运行时可观测字段或 `docs/MEASUREMENTS.md`。推断值单独标注。
> 相关文档：[延迟预算](LATENCY_BUDGET.md)、[标定](CALIBRATION.md)、[运维](OPERATIONS.md)、[调参](TUNING.md)、[ADR-0001](adr/0001-predictive-control.md)、[ADR-0002](adr/0002-world-frame-tracking.md)。

---

## 1. 系统边界

物理链路（与 v1 相同，v2 只重写了 Mac 侧软件）：

```
Windows(游戏) ──HDMI──▶ USB 采集卡 ──▶ Mac(采集 + ONNX 推理 + 跟踪 + 控制)
      ▲                                            │
      └────────── ESP32 USB HID 鼠标 ◀── 串口 ◀────┘
```

v2 的职责划分：

| 侧 | 负责 | 不负责 |
|---|---|---|
| Mac / `src/dma` | 取帧、推理、跟踪、选靶、控制、开火决策、协议打包、指标与日志 | 游戏内输入注入、HID 枚举 |
| ESP32 固件 | 解析串口帧、按 HID 报告率注入鼠标位移/按键、回传 STATUS | 视觉、控制律、参数整定 |
| 游戏 | 把 HID 输入在下一个输入 tick 应用、把画面输出到 HDMI | 任何配合 |

---

## 2. 分层

依赖方向只能向下；`control` 不 import `cv2`/`onnxruntime`，`transport` 不 import `control`，`core` 不 import 上层。

```
┌──────────────────────────────────────────────────────────────────────┐
│ 接口层   cli.py (aim2)   desktop/app.py (PySide6)   qt_gui.py(v1 遗留) │
│          sim/  tuning/  telemetry/  (离线工具，不参与实时回路)          │
├──────────────────────────────────────────────────────────────────────┤
│ 编排层   engine/runtime.py   Engine：装配、控制 tick、健康、快照        │
├──────────────────────────────────────────────────────────────────────┤
│ 控制层   control/latency.py  control/policy.py                         │
│          control/controller.py  control/fire.py                        │
├──────────────────────────────────────────────────────────────────────┤
│ 感知层   perception/geometry.py  perception/decoders.py                │
│          perception/detector.py  perception/tracker.py                 │
├──────────────────────────────────────────────────────────────────────┤
│ 采集层   capture/sources.py  capture/grabber.py                        │
│ 链路层   transport/protocol.py  transport/planner.py                   │
│          transport/link.py  transport/serial_link.py                   │
├──────────────────────────────────────────────────────────────────────┤
│ 基础层   core/config.py  core/types.py  core/metrics.py                │
│          core/logging.py  core/clock.py                                │
└──────────────────────────────────────────────────────────────────────┘
```

分层规则的具体表现：

- `perception/detector.py` 只依赖 `core`、`geometry`、`decoders`，返回 `DetectionFrame`，不知道控制器存在。
- `control/controller.py` 只接收 `ControlInput`（纯数值），不读帧、不写串口；因此可以在 `sim/harness.py` 里用假时间驱动。
- `transport/planner.py` 只做「像素 → HID counts」的量化与速率拆分；不决定往哪瞄。
- `engine/runtime.py` 是唯一的组合根：模型、采集、链路、跟踪器、策略、控制器、开火、延迟估计器都在 `Engine.__init__` 里装配。

---

## 3. 线程模型

### 3.1 线程清单

| 线程 | 入口 | 节拍 | 读 | 写 | 阻塞点 |
|---|---|---|---|---|---|
| 主/控制线程 | `Engine.run` → `FixedRate(runtime.control_hz)` → `Engine.step` | 240 Hz（默认） | `_det`、`cam_line`、`link` 状态 | `_snap`、`cam`、`cam_line`、`link` 累加器、开火状态 | 无阻塞 I/O；`step` 内只做数值计算与入队 |
| 采集线程 | `FrameGrabber._run`（`capture/grabber.py`） | 采集卡实际帧率（游戏中实测 60 Hz） | `FrameSource.read` | `_packet`（锁内替换） | `cv2.VideoCapture.read` 阻塞 |
| 推理线程 | `Engine._infer_loop`（`engine/runtime.py`） | 有新 `seq` 就跑 | `grabber.latest()` | `_det`（锁内替换） | `onnxruntime` 推理 |
| 链路线程 | `SerialLink._run`（`transport/serial_link.py`） | `output.report_hz`（默认 500 Hz）+ 心跳 | `planner` 累加器、按键状态、串口 | 串口写、`planner.stats`、设备遥测 | 串口写（有 `write_timeout_s`） |
| UI 线程 | `desktop/app.py::MainWindow._tick`（QTimer）轮询 `Engine.snapshot()` | 30 Hz（33 ms） | `_snap` | 无 | 只读；`EngineThread` 单独跑 `Engine.run()`，UI 不阻塞控制 |
| 热键线程 | `pynput.keyboard.Listener`（仅 `aim2 run` 启用） | 事件驱动 | 无 | 通过 `Engine.set_*_enabled` 改开关 | 无 |

`aim2 run` 的主线程就是控制线程；`Engine.start()` 才创建采集/推理线程，`SerialLink.start()` 由 `Engine.start()` 调用。`Engine.stop()` 依次停推理、采集、串口，健康状态置 `STOPPED`。

### 3.2 线程与共享状态

```
 HDMI / 采集卡
      │  cv2.read()
      ▼
┌──────────────┐  FramePacket(ts, seq, frame)   ┌──────────────┐
│ dma-grab     │ ──────── _packet (Lock) ─────▶ │ dma-infer    │
│ FrameGrabber │                                │ detector     │
└──────────────┘                                └──────┬───────┘
                                                       │ DetectionFrame
                                                       ▼
                                             ┌──────────────────┐
                                             │ _det (Lock)      │
                                             └────────┬─────────┘
                                                      │ 只在 frame_id 变化时消费
                                                      ▼
┌──────────────────────────────────────────────────────────────────────┐
│ 主/控制线程 240 Hz：tracker → policy → controller → fire → publish    │
│   cam 累加器 / DelayLine 只由本线程写                                  │
└───────────────┬──────────────────────────────────────────────────────┘
                │ submit_motion(dx,dy) / submit_button(pressed)
                ▼
┌──────────────────────────┐    planner.take(now)     ┌────────────────┐
│ dma-link 500 Hz          │ ◀─────────────────────── │ MotionPlanner  │
│ SerialLink._run          │ ───── 串口 v2 帧 ──────▶ │ ESP32 → HID    │
│ STATUS 解析 / 心跳 / 重连 │ ◀──── STATUS / 遥测 ───── │                │
└──────────────────────────┘                          └────────────────┘
                │
                ▼
        Engine.snapshot() ──▶ UI / CLI 打印 / 录制
```

共享状态与同步方式：

| 状态 | 生产者 | 消费者 | 同步 |
|---|---|---|---|
| `FrameGrabber._packet` | 采集线程 | 推理线程、控制线程（取 `frame` 引用） | `threading.Lock`，整包替换；旧帧没有队列，直接丢弃 |
| `Engine._det` | 推理线程 | 控制线程 | `_det_lock`；控制线程按 `frame_id` 判重 |
| `Engine._snap` | 控制线程 | UI/CLI | `_lock`，快照整体替换 |
| `Engine.cam`、`cam_line` | 控制线程 | 控制线程 | 无锁，单写者 |
| `planner` 累加器 | 控制线程 `submit_motion` | 链路线程 `take` | 单生产者/单消费者；`MotionPlanner` 内部持 `threading.Lock`，`add/take/reset/report` 互斥 |
| `SerialLink` 设备遥测 | 链路线程 | `status()` / 指标 | 简单属性赋值（GIL 下原子） |
| `fire`/`policy`/`tracker` 状态 | 控制线程 | 控制线程 | 无锁，单写者 |

设计取舍：

- **没有帧队列**。采集线程只保留最新一帧；推理慢时丢旧帧，而不是积压。积压就是输入延迟。
- **没有输出队列**。`LinkBase.submit_motion` 只把位移加进 `MotionPlanner` 累加器，链路线程按报告率取走；串口卡顿只会合并位移，不会让延迟随队列长度增长。
- **控制线程不做 I/O**。串口写、采集读、推理都在别的线程；控制线程只做 numpy/标量运算和 `submit_*`。

### 3.3 一次 tick 的时序

`Engine.step()`（`engine/runtime.py`）的精确顺序：

```
t = monotonic()
dt = clamp(t - last_tick, 1e-4, 0.2)

1. det = _det（锁内取引用）
2. health, detail = _evaluate(t, det)
     - grabber.stale_s()*1000 > runtime.watchdog_ms      → DEGRADED "采集流中断"
     - det is None                                       → DEGRADED "等待首帧推理"
     - t - det.capture_ts > runtime.max_infer_age_s      → DEGRADED "推理结果陈旧"
     - not link.ok()                                     → DEGRADED "输出链路断开"
3. 若 det.frame_id != _last_det_used（新检测帧）：
     a. 用最新帧尺寸更新 center、policy.frame_w/h；尺寸变化时清空 cam_line/latency
     b. t_sample = det.capture_ts - latency_split * latency.loop_s
        cam_meas = cam_line.sample(t_sample)
     c. world_dets = det.detections 平移 (+cam_meas)
     d. tracks = tracker.update(world_dets, det.capture_ts)
     e. target = policy.select(tracks, cam_meas, center, now)
     f. 若 target 非空且非 coasting：latency.on_detection(det.capture_ts, target.screen - center)
     g. 若 target.tid 变化：controller.reset()（积分/EMA 属于旧目标）
4. can_aim = aim_enabled and health == OK
   若 can_aim and target：
     age    = now - det.capture_ts
     apply  = max(dt, loop_s * (1 - latency_split))
     input  = ControlInput(world_xy, vel, cam_now=cam, cam_meas,
                           age_s=age + loop_s*latency_split, apply_s=apply,
                           sigma=track.sigma, coasting=target.coasting)
     out    = controller.step(input)
     link.submit_motion(out.dx, out.dy)
     latency.add_motion(out.dx, out.dy)
     cam   += (out.dx, out.dy)
     cam_line.push(now, cam)
     若 source 有 set_camera：source.set_camera(*cam)   # SyntheticSource 闭环用
   若 target 为 None：controller.reset()
5. 开火：
     lead = loop_s + (now - det.capture_ts)
     target_pt = aim_world + vel*lead - cam
     box_impact = screen_box 平移 (vel*lead + cam_meas - cam)
     decision = fire.update(target, preview, track.sigma, now)
     link.submit_button(fire.pressed(now) and fire_enabled and can_aim)
6. _publish(now, det) → 组装 Snapshot（健康、延迟、指标缓存、阶段状态、最新帧引用）
```

时序要点：

- **跟踪只在检测帧更新**（`det.frame_id` 变化），控制器每个控制 tick 都跑。跟踪器的时间戳用 `det.capture_ts`，不是 `now`；Kalman 的 `dt` 因此等于采集间隔。
- **世界坐标在跟踪器之前建立**：`world_dets = source box + cam_meas`。`cam_meas` 是「画面这一帧产生时，我们累计已发出的位移」，由延迟线在 `capture_ts - split*loop_s` 处采样。
- **延迟估计只吃实测位置**：coasting 目标是外推值，喂给 `LatencyEstimator` 会教出一个不存在的延迟；`Engine.step` 只在 `not target.coasting` 时调用 `on_detection`。
- **换靶清控制器**：`target.tid` 变化时 `controller.reset()`，避免积分把准星往旧目标方向拽；这也是 v1 `SWITCH_JUMP` 补丁在 v2 的结构化替代。
- **控制器输入同时包含测量延迟和应用延迟**：`age_s` 让控制器把目标外推到「现在」，`apply_s` 让增益按「命令生效窗口」归一化。两者之和约等于 `loop_s + frame_age`。
- **开火预览用整段环路延迟**：`lead_s = loop_s + det_age`，比控制器的 `age_s` 多出 `(1-split)*loop_s`，因为子弹落地时命令还要经过应用延迟。
- **健康不是 OK 时只停止瞄准/开火，不停止采集、推理和链路**：便于恢复与观测。`can_aim` 为假时 `target` 仍会更新，HUD 仍能看到检测/跟踪结果。

启动与停止：

```
Engine.start:
  grabber.ensure_open() → grabber.start() → link.start()（若有）
  _running.set() → infer 线程 start() → health=OK
Engine.run(duration):
  FixedRate(control_hz) 循环 step()；到时或 Ctrl-C 退出
Engine.stop:
  _running.clear() → join infer → grabber.stop → source.close → link.close → health=STOPPED
```

启动后的前几百毫秒通常是 `DEGRADED / 采集流中断` 或 `DEGRADED / 等待首帧推理`，属于正常预热；见 [OPERATIONS](OPERATIONS.md#3-健康状态)。

---

## 4. 坐标系

`core/types.py` 的模块 docstring 定义了四套坐标；这里是工程口径。

| 坐标系 | 定义 | 原点/单位 | 生产者 | 消费者 | 变换 |
|---|---|---|---|---|---|
| `source` | 采集帧像素坐标 | 左上角，px | `Transform.to_source_boxes`、`Detection` | 跟踪器输入前、录制、UI 绘制 | 模型坐标反变换 |
| `model` | 网络输入张量坐标（N×N） | 左上角，px | `Transform.to_model_xy`、`detector._preprocess` | ONNX 输出解码 | `x_m = (x_s - crop_x0)*sx + pad_x` |
| `world` | 去掉自身相机运动后的源像素坐标 | 与 source 同尺度；origin 是「瞄准会话开始时的画面」 | `Engine.step` 的 `world_dets`、`Track` | 跟踪器、策略、控制器、录制/回放 | `world = source + cam_meas` |
| `screen` | 玩家实际看到的画面坐标 | 左上角，px；准星在 `center=(w/2,h/2)` | `TargetPolicy.select`、控制器误差 | 控制器、开火预览、UI | `screen = world - cam_now` |

### 4.1 source ↔ model

`perception/geometry.py` 的 `Transform` 是同一条仿射变换的正反两向：

- `make_transform(src_w, src_h, in_w, in_h, crop, mode)`：
  - `mode="letterbox"`：保持宽高比，`sx=sy=min(in_w/cw, in_h/ch)`，灰边 114；
  - `mode="stretch"`：直接拉伸，`sx=in_w/cw, sy=in_h/ch`，无 pad；
  - `crop=(x0,y0,x1,y1)` 支持 ROI。
- `OnnxDetector._preprocess` 用同一个 `transform.crop`、`sx/sy`、`pad_x/pad_y` 做裁剪/缩放/填边；`detect` 最后用 `transform.to_source_boxes` 把框反变换回 source。**预处理和反变换共用同一个对象**，这是 v1 两个模型各自手写映射、框偏几个百分点的根因修复。
- `letterbox="auto"`（`OnnxDetector._resolve_letterbox`）：`e2e` 模型用 `stretch`，`yolov5/yolov8` 用 `letterbox`。`model_report`（`tuning/calibration.py`）会直接报错「yolov8 模型用 stretch 预处理会导致框偏」。

### 4.2 source ↔ world

`Engine.step` 中：

```python
cam_meas = self.cam_line.sample(det.capture_ts - measure_offset)
world_dets = [det.box.translate(cam_meas[0], cam_meas[1]) ...]
```

- `cam` 是「自瞄准开启以来我们累计发出的位移」，单位是屏幕像素；`cam_line` 是 `(ts, cam_x, cam_y)` 的历史，`DelayLine.sample` 线性插值。
- `cam_meas` 取的是**画面产生时刻**的相机位置，而不是当前时刻；`measure_offset = latency_split * loop_s`（默认 0.5 × 33 ms = 16.5 ms）。
- `world = source + cam_meas` 的含义：把「相机移动造成的画面平移」加回去，得到目标在地图/世界里的位置。
- 这是一个**增量世界系，不是绝对地图**：原点在瞄准会话开始时；玩家自己用手移动鼠标、后坐力、游戏镜头抖动都不在 `cam` 里，因此不会被补偿。`set_aim_enabled(False)`、画面尺寸变化都会清空 `cam`/`cam_line`。详见 [ADR-0002](adr/0002-world-frame-tracking.md)。
- 合成源要形成闭环，必须把 `cam` 回灌给场景：`Engine.step` 检测到 `source.set_camera` 就调用（`SyntheticSource.set_camera`）。否则画面不动、目标会「永远加速逃逸」，`aim2 run --sim` 的跟踪结果没有意义。

前置条件（必须标定）：

- 输出 1 px 必须对应画面 1 px 的平移。`MotionPlanner.add` 把 `dx` 乘以 `output.gain` 再转成 HID counts；如果「counts → 画面像素」的比例不是 1，世界系会被整体缩放，跟踪速度和前馈都会错。标定方法见 [CALIBRATION](CALIBRATION.md#3-灵敏度与增益标定)。

### 4.3 world ↔ screen

```python
# policy.select
sx, sy = wx - cam_x, wy - cam_y
# controller.step
ex = (world_x + vx*age) - cam_now_x - center_x
```

- `screen` 是玩家看到的画面；准星固定在 `center`。
- 策略用 `screen` 做 FOV、顶部/底部遮罩过滤；控制器用 `screen` 误差计算输出。
- 开火预览把目标框从 `screen_box` 再平移 `vel*lead + cam_meas - cam`，得到「命令全部生效后」目标会在哪里；见 `Engine.step` 的 `ImpactPreview` 构造。

### 4.4 坐标不变量

1. `Box` 在 `Detection`/`DetectionFrame` 里永远是 **source** 坐标；`Track.box` 永远是 **world** 坐标（`core/types.py::Track` docstring）。
2. `Transform` 的 `to_source_*` 与 `to_model_*` 必须互逆；任何新预处理都必须走 `make_transform`，不允许在 detector 里另写一套缩放。
3. `world = source + cam_meas` 与 `screen = world - cam_now` 互为逆；禁止在策略/控制器里混用 `cam` 和 `cam_meas`。
4. 准星中心在 `screen` 系恒为 `(w/2, h/2)`；`policy.frame_w/h` 在帧尺寸变化时更新，同时清空延迟线与延迟估计器（`Engine.step`）。

---

## 5. 模块职责与关键不变量

### 5.1 core

| 模块 | 关键函数/类 | 职责 | 不变量 / 失败模式 |
|---|---|---|---|
| `core/config.py` | `Settings`、`load`、`save`、`from_dict`、`migrate_v1`、`validate`、`set_fields`、`diff`、`FIELDS` | 版本化配置、迁移、校验、原子保存、字段级 diff | 所有配置段 frozen；未知字段进 `warnings` 而不是静默丢弃；`save` 用临时文件 + fsync + rename；`validate` 检查范围/枚举/`near_px<far_px`/`deadzone_exit>=deadzone`/宽高偶数 |
| `core/types.py` | `Box`、`Detection`、`DetectionFrame`、`Track`、`ControlOutput`、`FireDecision`、`Health`、`Snapshot` | 跨阶段数据契约 | 全部 frozen/slots；`Box` 是 source，`Track` 是 world；`Health` 五态 |
| `core/metrics.py` | `RollingStats`、`Metrics`、`Stopwatch` | 固定窗口百分位、计数器、gauge | `RollingStats.add` O(1)，`summary()` 才排序；控制线程只加样本不读 |
| `core/logging.py` | `setup`、`JsonFormatter`、`ConsoleFormatter`、`StageLogger` | JSONL + 控制台日志、限频 | 文件按 8 MiB × 3 轮转；`StageLogger.throttled` 按 key 限频 |
| `core/clock.py` | `FixedRate`、`DelayLine`、`RateMeter`、`smoothstep`、`clamp` | 单调时钟、无漂移节拍、延迟线 | 所有 `ts` 是 `time.monotonic()`；`FixedRate` 过载后重同步而不是累积追赶；`DelayLine.sample` 端点是钳位+线性插值 |

### 5.2 capture

| 模块 | 关键函数/类 | 职责 | 不变量 |
|---|---|---|---|
| `capture/sources.py` | `CvSource`、`VideoSource`、`SyntheticSource`、`open_source` | 设备/视频/合成帧源 | 源是「哑」的：只 `open/read/close`；重连、时间戳、丢帧策略都在 grabber |
| `capture/grabber.py` | `FrameGrabber._run`、`latest`、`stale_s` | 专用采集线程、最新帧槽、FPS、断流重连 | 只保留最新一帧；连续 8 次读失败才关/重开（`_fails >= 8`）；每次成功读都打 `capture_ts` |

`CvSource` 的打开参数来自 `capture.*`：`fourcc` 默认 `MJPG`，`width/height` 1280×720，`fps` 120，`buffer_size` 1。注意 `capture.fps` 是**请求值**，实际帧率由采集卡/游戏 HDMI 输出决定；游戏中实测被锁到 60 Hz（docs/MEASUREMENTS.md）。

### 5.3 perception

| 模块 | 关键函数/类 | 职责 | 不变量 |
|---|---|---|---|
| `perception/geometry.py` | `Transform`、`make_transform`、`center_crop` | source↔model 仿射、ROI | 预处理与反变换共用一个 `Transform`；letterbox 的 pad 是 114 灰 |
| `perception/decoders.py` | `infer_format`、`decode_e2e`、`decode_yolov5`、`decode_yolov8`、`nms`、`per_class_nms` | 三种输出头解码 + NMS | 纯 numpy，无 onnxruntime 依赖；e2e `(N,6)`；v5 `obj*cls`；v8 无 objectness、自动转置 |
| `perception/detector.py` | `OnnxDetector.__init__`、`_load`、`_roi`、`_preprocess`、`detect`、`bench` | 会话管理、provider 链、元数据类别、ROI、解码、类别过滤 | `resolve_providers` 总是追加 CPU 兜底；类别白名单在 NMS 前过滤；`teammate/ai/crosshair` 在默认 `ignore_classes` 中，永远进不了 tracker |
| `perception/tracker.py` | `KalmanBox`、`Tracker.update`、`predicted_box` | 6 状态 Kalman + 贪心 IoU 关联 | 状态 `[cx,cy,vx,vy,w,h]`；`Q` 用白噪声加速度模型；`update` 用 Joseph 形式保持 PSD；按类别关联（`det.cls_id == trk.cls_id`）；`misses > max_age` 才删 |

模型家族（来自本机 `aim2 models inspect` / 元数据）：

| 模型 | 输入 | 输出 | 解码 | 类别 | 实测 p50/p95（CoreML） |
|---|---|---|---|---|---|
| YOLOv8 320×320 导出 | 320×320 | `[1,9,2100]` | `yolov8` | `body, head, teammate, ai, crosshair` | 4.1 / 7.9 ms |
| 端到端 320×320 导出（示例） | 320×320 | `[1,300,6]` | `e2e` | `st, qiu, dj, fyb, sg1..sg4`（示例模型的类别表） | 5.0 / 6.3 ms |
| YOLOv5 640×640 导出 | 640×640 | `[1,25200,7]` | `yolov5` | `head, body` | 7.4 / 8.2 ms |
| 端到端 640×640 导出 | 640×640 | `[1,300,6]` | `e2e` | 同类端到端导出 | 16.6 / 20.6 ms |

> 实测条件：本机 CoreML、2560×1600 输入帧、`aim2 models bench`（`docs/MEASUREMENTS.md`）。这不是闭环延迟，只是推理段。

`ClassMap.is_targetable` 同时要求「在 `vision.target_classes` 中」且「不在 `vision.ignore_classes` 中」；默认 target 包含 `body/head/st/person/enemy/target`，ignore 包含 `teammate/ai/crosshair`。这是「队友永远不能成为目标」的代码级保证，而不是靠选靶逻辑碰运气。

### 5.4 control

| 模块 | 关键函数/类 | 职责 | 不变量 |
|---|---|---|---|
| `control/latency.py` | `LatencyEstimator.add_motion`、`on_detection`、`_estimate`、`report` | 闭环延迟辨识 | 只在**检测事件**上估计；候选 lag `0..max_lag`；残差 `|Δe + u_{k-m}|` 中位数最小者胜；`confidence = 1 - best/median(all)`；仅在 `conf >= min_conf` 时平滑更新 `loop_s` |
| `control/policy.py` | `TargetPolicy.select`、`_fuse`、`_aim_point` | 头/身体融合、选靶、迟滞、coast | 有 head 类时瞄 head 框中心；无 head 类时 `y1 + h*head_ratio`；head 框必须在 body 框内且面积 < 1.4×body；`switch_margin` 给非当前目标加罚分；只有无候选时才 coast |
| `control/controller.py` | `PredictiveAim.step`、`LegacyAim.step`、`build_controller` | 预测控制律 | 增益按误差调度；前馈在 kp 之外；积分仅在小误差门限内；饱和时按 headroom 投影抗 windup；输出 EMA 与 `max_step` 限幅；`max_accel` 可选 |
| `control/fire.py` | `FireController.update`、`pressed` | 开火状态机 | 命中条件为「内缩框命中 **且** 残差 ≤ `fire.radius`」，再叠加驻留、sigma、非 coasting、冷却/单发；每个拒绝原因都计数 |

控制器关键公式（`PredictiveAim.step`）：

```
scale = clamp(dt / apply_s, 0.02, 1.0)
kp    = kp_near + (kp_far-kp_near) * smoothstep(near_px, far_px, dist)
ff    = clamp(v * apply_s * lead_gain, ±lead_max)   # coasting 时 ×0.4
u     = scale * (kp*e + ki*I + ff) + kd*(e - e_prev)
u     = slew_limit(u); u = clamp_magnitude(u, max_step*scale)
I    += e*dt（仅 dist < integral_gate），并 clamp 到 integral_limit
饱和时 I 的允许上限 = (max_u - |PD+FF|)/ki
```

详见 [LATENCY_BUDGET](LATENCY_BUDGET.md#5-延迟与增益的稳定边界) 和 [ADR-0001](adr/0001-predictive-control.md)。

### 5.5 transport

| 模块 | 关键函数/类 | 职责 | 不变量 |
|---|---|---|---|
| `transport/protocol.py` | `crc8`、`encode`、`encode_move`、`decode_status`、`StreamParser.feed`、`legacy_*` | v2 帧、CRC8/ATM、seq、设备遥测、v1 兼容 | v2 magic `0xA5`，version/type 字节 `0x21`，payload ≤255，CRC 覆盖 header+payload；`StreamParser` 遇到坏帧丢 1 字节重同步 |
| `transport/planner.py` | `MotionPlanner.add`、`take`、`flush` | 累加位移、按报告率拆分、余数携带、±127 限幅 | `add` 乘 `output.gain`；累加器上限 `4*max_per_report`；`take` 每次最多 8 个报告；余数留到下一次，不丢 |
| `transport/link.py` | `LinkBase`、`MockLink`、`build_link` | 链路抽象、dry-run | 无队列；`MockLink` 只记录不写口 |
| `transport/serial_link.py` | `SerialLink._run`、`_open`、`_emit_motion`、`_read_available`、`status` | 串口线程、重连、心跳、遥测 | 按键仅在变化时发；运动每报告周期发；连续 5 次写错误关闭重开；`ok()` 要求端口打开且错误 <5 |

协议帧（v2）：

```
0      magic 0xA5
1      version(高 4 位)=2 | type(低 4 位)
2      seq
3      payload length
4..    payload
last   CRC-8/ATM (poly 0x07) over bytes[0 : 4+len]
```

- `T_MOVE=1`：payload `<hh>` dx,dy（int16；planner 保证单报告 ±127）。
- `T_BUTTON=2`：1 字节 mask。
- `T_PING=3`：4 字节 counter。
- `T_STATUS=4`（设备→主机）：`STATUS_LEN=13`，`struct "<IHHHHB"` → uptime_ms(u32), rx_frames(u16), crc_errors(u16), queue_drops(u16), report_hz(u16), flags(u8)。固件 `firmware/esp32_hid/README.md` §5 与主机逐字节对齐。
- v1 兼容：`output.protocol="v1"` 时发 `0xAA` 的 10 字节旧帧，固件需匹配。

### 5.6 engine

| 函数 | 职责 | 不变量 |
|---|---|---|
| `Engine.__init__` | 装配 detector/source/grabber/link/tracker/policy/controller/fire/latency/cam_line | 所有依赖可注入（测试/仿真替换） |
| `Engine.start/stop/run` | 生命周期与固定节拍 | 启动顺序：采集→链路→推理；停止顺序相反 |
| `Engine.step` | 唯一把各阶段耦合起来的地方 | 见 §3.3；`tracker` 只在 `frame_id` 变化时更新；`cam` 只在 `can_aim and target` 时累加 |
| `Engine._evaluate` | 健康判定 | 四种 DEGRADED 原因，见 §5.7 |
| `Engine._publish` | 组装 `Snapshot` | UI 只读；指标缓存按 `metrics_interval_s` 刷新 |
| `Engine.apply` | 热加载不需要重开硬件的段 | `vision/capture/output.port/baud` 变更只提示重启；任何 diff 都会重建控制器（见 §7 已知问题） |
| `Engine.describe` | 运维摘要 | health/detail/model/capture/link/latency/shots/frame_size |

### 5.7 健康状态

| 状态 | 产生位置 | 含义 | 瞄准/开火 |
|---|---|---|---|
| `BOOT` | `Engine.__init__` | 未启动 | 否 |
| `OK` | `Engine._evaluate` 全部通过 | 采集/推理/链路都健康 | 允许 |
| `DEGRADED` | `_evaluate` 四个分支 | 某一阶段不健康，细节在 `health_detail` | 抑制（`can_aim=False`） |
| `FAILED` | **当前代码没有赋值路径** | 预留给不可恢复错误 | 否 |
| `STOPPED` | `Engine.stop` | 已停止 | 否 |

`DEGRADED` 的四个 detail 字符串是运维的判据：

| detail | 触发 | 阈值 | 处置 |
|---|---|---|---|
| `采集流中断` | `grabber.stale_s()*1000 > runtime.watchdog_ms` | 400 ms | 检查 HDMI/采集卡/摄像头权限/是否被 OBS 占用 |
| `等待首帧推理` | `det is None` | — | 等预热；持续则查模型/推理线程异常 |
| `推理结果陈旧` | `now - det.capture_ts > runtime.max_infer_age_s` | 250 ms | 查推理耗时 p95、采集是否掉到 0、线程是否被阻塞 |
| `输出链路断开` | `not link.ok()` | 端口未开或连续 5 次写错误 | 查串口/固件/线缆；`status()["detail"]` 有异常类型 |

### 5.8 sim / tuning / telemetry / cli

| 模块 | 关键函数 | 职责 | 不变量 |
|---|---|---|---|
| `sim/world.py` | `Scenario.pose/velocity`、`build`、`SCENARIOS` | 确定性目标运动 | 纯时间函数，无随机（除 query seed 的 rng 未用于 pose）；场景固定 6 个 |
| `sim/fake_detector.py` | `SyntheticDetector.detect/describe/bench` | 阈值检测器（HSV 颜色分割），无 onnxruntime | 输出与 ONNX 路径相同的 `DetectionFrame`；只认合成画面的蓝/绿矩形，`cls_name="body"`；`aim2 run --sim` 与桌面端 `--sim` 用它 |
| `sim/harness.py` | `simulate`、`compare`、`_recovery` | 闭环仿真、台架指标、v1/v2 对比 | 采集/检测频率、半段延迟、量化余数、dropout 都可配置；`legacy` 对照组强制 `control_hz=detect_hz`、无 tracker |
| `tuning/autotune.py` | `evaluate`、`autotune`、`SearchSpace` | 离线坐标下降 | 目标函数 `track_rms + 0.35*jitter + 2*recovery + penalty`；只在严格更优时接受；`passes` 默认 2 |
| `tuning/calibration.py` | `model_report`、`head_ratio_from_session`、`latency_from_estimator`、`wait_for_latency` | 标定辅助 | 模型无 head 类只警告；队友/人机被列为 target 是 error；head_ratio 需要 ≥20 个样本 |
| `telemetry/recorder.py` | `SessionRecorder.frame/tick/fire`、`load_session`、`summarize` | 会话录制 | gzip JSONL 四类记录：header/frame/tick/fire；frame 存 source 框 + cam 样本；tick 存位移/目标/延迟 |
| `telemetry/replay.py` | `replay`、`replay_compare` | 用真实录制重放不同控制器 | 世界轨迹由「source 框 + cam 样本」重建，不依赖当时控制器 |
| `cli.py` | `cmd_doctor`、`cmd_models`、`cmd_sim`、`cmd_bench`、`cmd_autotune`、`cmd_replay`、`cmd_run`、`cmd_record`、`cmd_config`、`cmd_calibrate` | 统一入口 | 每个子命令都有 `--json`；无硬件路径（`--sim/--dry-run`）与有硬件路径共用同一 Engine |
| `desktop/app.py` | `MainWindow`、`EngineThread`、`ParamRow`、`PreviewWidget` | PySide6 控制台 | 参数控件由 `core.config.FIELDS` 生成；改动走 `set_fields` + `Engine.apply` 热加载；30 Hz 轮询 `Snapshot`；预览叠检测/跟踪/目标框 |

---

## 6. 为什么用世界坐标系

v1 在屏幕系里锁点（`main_cli.py` 的 `lock=(aim_x,aim_y)`），带来四个结构性问题：

1. **自身运动污染目标速度**。屏幕误差 `e = p_world - c` 的差分同时包含敌人速度和我们的速度；PID 的 D 项和任何速度前馈都会把自己的输出当成目标运动，形成正反馈。v1 只能靠低增益和高阻尼压制，代价是跟枪滞后。
2. **多目标身份不稳定**。屏幕系里目标一跳，锁点就跳；v1 用 `SWITCH_JUMP` 清积分、`COAST_FRAMES` 续命，都是补丁。v2 用 `Track.tid` 保持身份，速度/协方差随身份走。
3. **延迟补偿没有基准**。要预测「命令生效时敌人在哪」，必须知道「敌人相对世界怎么动」；屏幕系里这个量随我们自己的每一步变化，无法外推。
4. **录制无法复现**。只存屏幕框，回放时换一个控制器就会得到不同的自身运动，目标的表观轨迹也变了。v2 存 `cam` 样本，世界轨迹与控制器无关，才能做 replay A/B。

世界系带来的成本：

- 需要标定输出灵敏度（1 px ↔ 1 px）；
- 需要延迟线采样（否则自身运动还是漏进 tracker）；
- 需要处理增量世界系的原点/reset（瞄准开关、分辨率变化）；
- 玩家手动鼠标、后坐力、镜头抖动不在补偿范围内，属于已知残差。

---

## 7. 扩展点

### 7.1 换模型

1. 把 `.onnx` 放进 `models/`。
2. `aim2 models inspect 端到端 320×320 导出 --json` 看 `input/format/targets/heads/letterbox`。
3. `aim2 config set vision.model 端到端 320×320 导出；按需改 `vision.target_classes` / `ignore_classes` / `conf` / `iou` / `roi`。
4. 重启进程（`Engine.apply` 对 `vision.*` 只提示重启，不会重建 detector）。
5. 验证：`aim2 doctor --probe`、`aim2 models inspect`、实机 HUD 的检测框与类别。

模型侧硬约束：

- 类别名来自 ONNX metadata `names`；没有 metadata 时 `_parse_names` 会退回 `"0: body, 1: head"` 风格解析，仍失败则只有数字类名，`ClassMap` 无法把 `body/head` 映射到 id，`targets` 为空，`model_report` 报 error。
- 输出头必须是三种之一；新形状要在 `decoders.infer_format` 加判定，并在 `decoders.py` 加 `decode_*`，再在 `OnnxDetector._decode` 分派。
- `yolov8` 必须走 letterbox（`vision.letterbox=auto` 已保证；手动设 `stretch` 会被 `model_report` 报 error）。

### 7.2 换输出/执行器

- 新链路实现 `transport/link.py::LinkBase` 的 `submit_motion` / `submit_button` / `ok` / `status` / `close`，在 `build_link` 里分派。
- 新协议在 `transport/protocol.py` 加 `T_*` 与 `encode_*` / `decode_*`；`StreamParser` 只认 `MAGIC` 和 CRC，新增 type 不需要改解析器。
- 固件必须与 `output.protocol` 匹配：v2 是 `0xA5` + CRC8；v1 是 `0xAA` + XOR。

### 7.3 换游戏

| 变更 | 位置 | 注意 |
|---|---|---|
| 分辨率/采集卡 | `capture.width/height/fps/fourcc` | 必须与游戏原生分辨率对齐（docs/MEASUREMENTS.md：不对齐会多 1–2 帧缩放延迟）；宽高必须偶数 |
| 准星/视野 | `policy.fov_radius`、`top_mask`、`bottom_mask` | 不同游戏 HUD 位置不同；遮罩按画面比例 |
| 头部 | `vision.target_classes` 含 `head` 则无需 `head_ratio`；否则 `policy.head_ratio` | 无 head 类必须现场标定 |
| 灵敏度 | `output.gain` + 游戏内灵敏度 | 必须让 1 px 命令 = 1 px 画面平移，否则世界系失真 |
| 开火判定 | `fire.*` | 不同游戏命中框/延迟不同；先用 `fire.mode=off` 验证瞄准 |
| 输入 tick | `output.report_hz`、`runtime.control_hz` | 游戏锁 60 Hz 时提高控制率不会提高可见更新率，但能减小量化台阶 |

### 7.4 换控制器/加 UI

- 控制器：实现 `step(ControlInput) -> ControlOutput`，加 `kind` 属性，在 `control/controller.py::build_controller` 分派，并加 `aim.controller` 枚举。`Engine.apply` 用 `kind` 判断，只有换了控制律才重建控制器。
- UI：只读 `Engine.snapshot()` 返回的 `Snapshot`；需要新字段时在 `_publish` 里加，不要在 UI 线程直接读 tracker/controller 内部状态。现有实现见 `desktop/app.py`：`MainWindow._tick` 每 33 ms 取一次快照，`EngineThread` 在独立线程跑 `Engine.run()`，参数控件由 `FIELDS` 自动生成，改动通过 `set_fields` + `Engine.apply` 生效。
- 桌面入口：`PYTHONPATH=src .venv/bin/python -m dma.desktop.app`（`--sim` 无硬件，`--dry-run` 不写串口）。
- 测试：解码器/几何是纯 numpy，适合单测；跟踪/控制用 `sim/harness.py`；端到端用 `aim2 run --sim --dry-run`。

---

## 8. 当前实现边界（代码与描述不一致处）

以下是写文档时在代码树中确认过的事实，运维/调参时必须知道：

| 项 | 现状 | 影响 |
|---|---|---|
| `desktop/app.py` | 已实现：`FIELDS` 生成参数、30 Hz `Snapshot`、`EngineThread` 跑引擎、`set_fields` + `Engine.apply` 热加载、预览叠加 | 桌面端入口 `python -m dma.desktop.app [--sim]`；`qt_gui.py` 仍是 v1 扁平参数 GUI（import `dma.config/vision/aim`），不驱动 v2 Engine |
| `tests/` | 已实现 8 个测试文件；本机 `pytest -q` 全部通过（当前 79 个用例） | 覆盖配置/几何/三解码/NMS/跟踪/延迟/速率归一化/选靶/开火/协议重同步/规划器/引擎/仿真/录制回放 |
| `firmware/esp32_hid/` | 已实现：`esp32_hid.ino`、`platformio.ini`、`README.md`；STATUS 13 字节，与主机协议逐字节对齐 | 烧录前按 README §2 设 `USB-OTG (TinyUSB)` + `USB CDC On Boot: Enabled`；HID 1000 Hz |
| `Engine.apply` 控制器判断 | 已修正为 `getattr(self.controller, "kind", None)` | 只有真的切换控制律才重建控制器；普通参数改动不会清零积分/EMA |
| `fire.radius` | 判定为「内缩框命中 **且** 残差 ≤ radius」；默认 `radius=9`、`inset=0.06` | 调大更宽松（更容易开火），调小更严格；与 `inset` 共同决定开火窗口 |
| `policy.lock_radius` | `TargetPolicy.select` 的外推漂移上限：`|v|·Δt` 超过它就放弃 coast | 防止追着「瞬移前的幻影」跑；过小会让遮挡跟踪变差 |
| `aim.settle_px` | 只在 `sim/harness.py` 用于首次捕获指标 | 实时控制不使用 |
| `capture.warmup_s` | `FrameGrabber`/`CvSource` 不使用 | 启动预热由采集卡自身和首帧等待决定 |
| `runtime.record`/`record_dir` | `Engine` 不读；`cli._run_engine` 读并创建 `SessionRecorder` | 直接用 `Engine` 不会录制 |
| `runtime.log_level`/`log_dir` | `Engine` 不读；`cli.main` 固定 `setup_logging(level="INFO", log_dir=None)`；桌面端 `console=False` | CLI 只写控制台、不落盘；桌面端不写控制台；需要 JSONL 时显式 `setup(..., "out/logs")` |
| `FAILED` | `Health.FAILED` 没有赋值路径 | 不要依赖该状态做告警 |
| `run_mac.sh` | `run / run-dry / gui / sim / doctor / selftest / bench` 七个入口 | 全部走 `python -m dma.cli`，路径相对脚本自身 |
| `aim2 bench` | `legacy` 对照组在 `sim/harness.py::compare` 中强制 60 Hz、无 tracker | 这是有意的「v1 等价」基线；当前默认参数下门禁通过 |
| `telemetry/replay.py` | 回放把每条检测的 `cls_name` 一律设为 `"body"` | 回放无法复现 head 类融合/头部优先；head_ratio 仍可用 |
| 世界系 reset | `set_aim_enabled(False)` 重置 `cam`/`cam_line`，帧尺寸变化清 `cam_line`/`latency`，但都不清 `tracker` | 旧 world track 与新原点不一致，重新开启/改分辨率后可能跳一下；见 [ADR-0002](adr/0002-world-frame-tracking.md#53-当前代码的已知缺口) |
| `Engine.set_aim_enabled(False)` | 已改为 `planner.reset()` 丢弃残留位移 | 关闭瞄准后准星不会再有尾巴动作 |

这些不是「文档免责声明」，而是当前代码的真实边界；改动其中任何一项都应同步更新本文与对应文档。

---

本文档对应 schema_version 2 与 src/dma 布局，改动代码请同步更新。
