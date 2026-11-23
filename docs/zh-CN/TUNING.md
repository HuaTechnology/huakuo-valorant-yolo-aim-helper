# 调参手册（TUNING）

<div align="right">

**简体中文** | [English](../TUNING.md)

</div>

> ⚠️ 仅供学习与研究，请勿用于任何竞技/排位场景。见 [DISCLAIMER](../../DISCLAIMER.md)。

> 代码基线：`src/dma`，schema 2。默认值取自 `src/dma/core/config.py`；仿真对比数值引用 `docs/MEASUREMENTS.md` §4（闭环仿真，33 ms 链路延迟、2.5 px 检测噪声、60 fps 采集、240 Hz 控制），不是台架实测。
> 调参顺序：**先标定（[CALIBRATION](CALIBRATION.md)）→ 再仿真 → 再回放 → 最后实机**。标定没做，调参只是在补偿错误的世界系/延迟。

---

## 1. 三条硬规则

1. **增益是按「命令生效窗口」归一化的**，不是「每 tick」。`u = scale*(kp*e + ki*I + ff) + kd*Δe`，`scale = dt/apply_s`。所以 60/120/240 Hz 下同一组 `kp` 行为一致；不要因为换了控制频率就重调 `kp`。
2. **`kp` 的物理上限是 1**（死拍）。默认 `kp_near=0.50`、`kp_far=0.85` 已经接近；`kp_far` 不要再往上顶，除非延迟估计 `confidence ≥ 0.35` 且仿真/回放都通过。
3. **一次只改一个参数**，改完留下 `config diff` 与 `bench`/`replay` 结果。v1 的 33 个滑块靠手感，v2 用数字说话。

---

## 2. 参数语义

### 2.1 `aim`（控制律）

| 参数 | 默认 | 范围 | 语义 | 调大 | 调小 |
|---|---|---|---|---|---|
| `controller` | `predictive` | `predictive｜legacy` | 控制律选择 | `legacy` = v1 PD+I 适配器（仅 A/B/回滚） | — |
| `kp_near` | 0.50 | 0–1.5 | 近距比例增益（误差 < `near_px`） | 贴脸更快，过大在目标附近抖/过冲 | 更稳，但近距收敛慢 |
| `kp_far` | 0.85 | 0–1.5 | 远距比例增益（误差 > `far_px`） | 甩枪更快，>1 易极限环 | 甩枪变慢，抗延迟更强 |
| `near_px` | 26 | 2–120 px | 近距带宽起点 | 更早进入低增益，稳但整体变慢 | 更晚进入低增益，贴脸更激进 |
| `far_px` | 260 | 40–600 px | 远距带宽终点 | 高增益只作用于更大的误差，稳 | 高增益覆盖更多误差，快但抖 |
| `ki` | 0.05 | 0–0.8 | 积分系数（消稳态误差） | 跟枪滞后减小，过大 windup/回弹 | 更保守，稳态误差靠 kp |
| `kd` | 0.10 | 0–1.0 | 微分阻尼（逐 tick 差分） | 抑制过冲/抖动，过大会放大检测噪声 | 更跟手，但容易过冲 |
| `integral_limit` | 60 | 0–300 px | 积分项上限 | 允许更大的稳态补偿，风险变大 | 更保守 |
| `integral_gate` | 70 | 0–200 px | 误差小于此值才积分 | 更大范围积分，可能在大误差时 windup | 只在很近时积分，防甩枪回弹 |
| `lead_gain` | 0.9 | 0–1.5 × | 速度前馈强度；1.0 = 完全补偿 `apply_s` 内的目标移动 | 跟枪更紧，过大在速度估计噪声上抖 | 更保守，滞后增加 |
| `lead_max` | 70 | 0–250 px | 单次前馈限幅 | 允许更大预判，坏速度估计会放大 | 更稳，快速目标补偿不足 |
| `latency_prior_s` | 0.033 | 0–0.20 s | 在线辨识收敛前的延迟先验 | 收敛前更保守（更多前视） | 收敛前更激进；0 等于不预判 |
| `latency_split` | 0.5 | 0.2–0.8 | 总延迟中「画面陈旧」占比 | 更多外推，可能过冲 | 更多算作应用延迟，`apply_s` 变大、`scale` 变小 |
| `latency_window` | 45 | 10–200 tick | 延迟辨识窗口 | 更平滑、更慢 | 更快、更抖 |
| `latency_min_conf` | 0.2 | 0–1 | 低于此置信度不更新 `loop_s` | 更保守，收敛慢 | 更容易更新，可能被噪声带偏 |
| `max_step` | 140 | 10–400 px | 单 tick 最大位移（再乘 `scale`） | 甩枪更快，饱和减少；过大过冲 | 更稳，大误差走不动 |
| `max_accel` | 0（关） | 0–200 px/tick | 单 tick 输出变化上限（乘 `scale`） | 更平滑，甩枪变慢 | 0 = 关闭 |
| `deadzone` | 1.5 | 0–20 px | 死区内半径 | 静止更稳，微调变钝 | 更跟手，可能抖 |
| `deadzone_exit` | 5.0 | 0–40 px | 退出死区的半径（滞回） | 更不容易被小误差拉出死区，稳 | 更容易退出，可能抖进抖出 |
| `output_ema` | 0.9 | 0–1 | 输出平滑系数；1=不平滑 | 更跟手，噪声更多 | 更平滑，群延迟增加（`(1-α)/α·dt`） |
| `settle_px` | 7.0 | 1–40 px | **仅仿真**首次捕获判据 | 无实时效果 | 无实时效果 |

### 2.2 `policy`（选靶）

| 参数 | 默认 | 范围 | 语义 | 调大 | 调小 |
|---|---|---|---|---|---|
| `fov_radius` | 550 | 80–1200 px | 候选目标的最大屏幕距离 | 能锁更远的目标，误锁/乱飘增加 | 只锁近处，稳定但视野窄 |
| `lock_radius` | 160 | 20–500 px | 丢失后外推的漂移上限（`\|v\|·Δt` 超过即放弃 coast） | 更能容忍遮挡/快速移动，但可能追着幻影 | 更早放弃，稳但遮挡时容易丢 |
| `switch_margin` | 55 | 0–300 px | 非当前目标的距离罚分（换靶迟滞） | 更粘当前目标，换人慢 | 更容易换靶，多目标摇摆 |
| `coast_s` | 0.18 | 0–0.6 s | 目标丢失后的外推时长 | 短暂遮挡下更连续，长时间消失会多推 | 更快放弃，可能频繁重置 |
| `head_bonus` | 28 | 0–120 px | 头部目标的距离奖励 | 更偏好头，可能舍近求远 | 更看重距离，头优先级下降 |
| `top_mask` | 0.08 | 0–0.40 | 忽略画面顶部比例（HUD） | 过滤更多 HUD，可能漏高处目标 | 更少过滤 |
| `bottom_mask` | 0.12 | 0–0.45 | 忽略画面底部比例（手/武器） | 过滤更多底部干扰 | 更少过滤 |
| `head_ratio` | 0.11 | 0–0.40 | 无 head 类时，头部瞄准点在框内的比例 | 瞄点下移（脖子/胸），过高会打身体 | 瞄点上移（头顶/空气） |
| `prefer_head` | true | bool | 是否给头部加分 | 关掉后只看距离 | — |
| `min_hits` | 1 | 1–6 | 选靶所需最少命中帧 | 更保守，新目标出现慢 | 更快锁定，误检风险 |

### 2.3 `fire`（开火）

| 参数 | 默认 | 范围 | 语义 | 调大 | 调小 |
|---|---|---|---|---|---|
| `mode` | `single` | `off｜single｜auto` | 开火模式 | `auto` 连发；`single` 每次锁定一枪 | `off` 完全不开火 |
| `radius` | 12 | 0–60 px | 命中点距瞄准点的回退半径（与内缩框是**或**关系） | 更容易开火（可能早开） | 更严格 |
| `inset` | 0.06 | 0–0.40 | 目标框内缩比例 | 更保守（要求更靠中心） | 更宽松（靠边缘也打） |
| `dwell_s` | 0.04 | 0–0.30 s | 连续满足条件多久才开火 | 更稳，反应慢 | 更快，可能半路开火 |
| `cooldown_s` | 0.12 | 0.02–0.60 s | 两次开火最小间隔 | 连发更慢 | 连发更快 |
| `tap_s` | 0.03 | 0.005–0.15 s | 按键按住时长 | 更长按压（武器可能连发） | 更短点按 |
| `max_sigma` | 80 | 1–300 px | Kalman 位置不确定度上限 | 更敢在不确定时开火 | 更保守，新目标出现时不开 |

`fire.reason` 的排查见 [OPERATIONS §6.4](OPERATIONS.md#64-开火)。

### 2.4 `tracker`（Kalman 跟踪）

| 参数 | 默认 | 范围 | 语义 | 调大 | 调小 |
|---|---|---|---|---|---|
| `enabled` | true | bool | 是否启用跟踪；关闭则每帧检测独立成 track | 关掉后无速度/协方差，控制器退化 | — |
| `iou_thr` | 0.25 | 0.05–0.80 | 关联所需 IoU | 关联更严，身份容易断 | 关联更松，可能把两个人串成一个 |
| `max_age` | 12 | 1–40 帧 | 丢失多少帧后删除 track | 更耐遮挡，可能保留幽灵 | 更快清理，重新捕获变慢 |
| `accel_noise` | 3000 | 100–12000 px/s² | 机动能力 σa（过程噪声） | 跟得更快、速度估计更灵敏，也更抖 | 更平滑，快速变向滞后 |
| `meas_noise` | 3.5 | 0.5–20 px | 检测位置噪声 σ | 更信模型、更平滑 | 更信检测、更跟手但抖 |
| `size_noise` | 6.0 | 0.5–30 px | 框尺寸噪声 σ | 尺寸变化更平滑 | 尺寸更跟手 |

### 2.5 `output` / `runtime` / `capture` / `vision`（运维相关）

| 参数 | 默认 | 语义 | 注意 |
|---|---|---|---|
| `output.protocol` | `v2` | `v1｜v2` | 必须与固件匹配；`v1` 无遥测 |
| `output.port` | `auto` | 串口路径 | 多设备时显式指定；改动需重启 |
| `output.baud` | 921600 | 波特率 | 改动需重启 |
| `output.report_hz` | 500 | HID 报告率 | 子帧平滑度；固件/设备实际值看 `device.report_hz` |
| `output.gain` | 1.0 | 每像素命令对应的 HID counts 倍率 | **世界系尺度**；必须按 [CALIBRATION §3](CALIBRATION.md#3-灵敏度与增益标定) 标定 |
| `output.max_per_report` | 127 | 单报告 int8 上限 | 受 HID 限制；planner 会拆分 |
| `output.heartbeat_ms` | 100 | 心跳周期 | 太小增加串口负载 |
| `output.write_timeout_s` | 0.02 | 写超时 | 太小易误判，太大卡链路 |
| `output.dry_run` | false | 不写串口 | 联调用 |
| `runtime.control_hz` | 240 | 控制线程频率 | 改动需重启 `run` 循环（`FixedRate` 在启动时创建） |
| `runtime.watchdog_ms` | 400 | 采集陈旧阈值 | 太大故障发现慢，太小误报 |
| `runtime.max_infer_age_s` | 0.12 s | 推理结果最大陈旧度 | 与推理 p95、采集 fps 匹配 |
| `runtime.metrics_interval_s` | 1.0 | 指标缓存刷新 | 只影响 `Snapshot.metrics` 刷新 |
| `runtime.log_level` / `log_dir` | INFO / `out/logs` | 日志 | CLI 当前 `log_dir=None`，见 [OPERATIONS §4](OPERATIONS.md#4-日志) |
| `runtime.record` / `record_dir` | false / `out/telemetry` | 会话录制 | 仅 CLI `run --record`/`record` 读取 |
| `capture.width/height` | 1280×720 | 采集分辨率 | 必须与游戏原生对齐；宽高偶数；改动需重启 |
| `capture.fps` | 120 | 请求帧率 | 游戏锁 60 时实际 60；以 `capture.fps` 为准 |
| `capture.fourcc` | MJPG | 采集格式 | MJPG 比 YUYV 多编解码延迟（docs/MEASUREMENTS.md） |
| `capture.buffer_size` | 1 | 驱动缓冲 | >1 增加延迟 |
| `capture.warmup_s` | 2.0 | **当前未被 grabber 使用** | 启动预热由首帧等待决定 |
| `vision.model` | `models/model.onnx` | 模型路径 | 改动需重启；输出头（e2e/v5/v8）自动识别 |
| `vision.conf` | 0.35 | 置信度阈值 | 改动需重启；漏检降、误检升 |
| `vision.iou` | 0.45 | NMS IoU | 改动需重启；重复框降 |
| `vision.max_det` | 32 | 最大检测数 | 多目标场景 |
| `vision.letterbox` | auto | 预处理 | `yolov8` 必须 letterbox |
| `vision.roi` / `roi_size` | off / 512 | ROI 裁剪 | 目标总在中心时用；ROI 会改变有效分辨率 |
| `vision.target_classes` / `ignore_classes` | body/head/st/... / teammate/ai/crosshair | 类别白/黑名单 | 改动需重启；`teammate` 永远不要放 target |

---

## 3. 自动调参

### 3.1 目标函数

`tuning/autotune.py::evaluate`：

```
cost_scenario = tracking_rms_px
              + 0.35 * jitter_px
              + 2.0  * min(recovery_s, 0.6)
              + penalty
cost = mean(cost_scenario over scenarios)
```

惩罚项（`_penalty`）：

| 条件 | 惩罚 |
|---|---|
| `tracking_rms` 非有限 | 1e6（直接淘汰） |
| `tracking_rms > 120 px` | +500 |
| `jitter_px > 6.0` | +(jitter-6)*25 |
| `on_target < 0.5` | +(0.5-on_target)*200 |

`recovery_s` 非有限（没有大偏差事件）按 0 计；`min(...,0.6)` 封顶。

### 3.2 搜索空间

`SearchSpace.values`（默认）：

| 字段 | 候选值 |
|---|---|
| `kp_near` | 0.28, 0.36, 0.42, 0.50, 0.58 |
| `kp_far` | 0.50, 0.62, 0.75, 0.88, 1.00 |
| `ki` | 0.0, 0.05, 0.10, 0.18, 0.30 |
| `kd` | 0.0, 0.03, 0.06, 0.10, 0.16 |
| `lead_gain` | 0.70, 0.85, 1.00, 1.15 |
| `deadzone` | 0.0, 1.0, 2.0, 3.0 |

坐标下降，`passes=2`，只在 `cost < best_cost - 1e-6` 时接受。默认 6 个场景 × 约 30–50 次评估，本机数秒级。

### 3.3 用法

```bash
cd ~/dma
export PYTHONPATH=src

# 看结果，不写盘
.venv/bin/python -m dma.cli autotune --passes 2 --json

# 写回 aim 段（写到 --settings 或默认 configs/settings.json）
.venv/bin/python -m dma.cli autotune --passes 2 --write
```

输出：

| 字段 | 含义 |
|---|---|
| `cost` / `baseline_cost` / `improvement` | 调参后/调参前/相对改善 |
| `aim` | 完整的 `AimCfg`（不只是改动的字段） |
| `history` | 每次候选的 `(field,value,cost)` |
| `rows` | 每个场景的 `track_rms/jitter/on_target/recovery_s` |

注意事项：

- 搜索空间上界 `kp_far=1.0` 就是稳定边界。自动调参**可能**推到 1.0；接受前必须跑 `bench` + `replay`，并在实机 `confidence≥0.35` 下复核。
- 目标函数里有 `jitter` 和 `recovery`，所以它不会为了 RMS 把输出调成神经质；但仿真没有真实检测抖动谱，实机仍可能更抖。
- `autotune` 只优化 `aim` 段；`policy/fire/tracker` 不在搜索空间内，改这些要手工 + 仿真。
- `--write` 会保存完整配置（包括迁移后的 `output.gain` 等），不会只写 6 个字段；写前先备份。
- `autotune` 的仿真用 `SimConfig()` 默认：240 Hz 控制、60 Hz 检测、33 ms 延迟、2.5 px 噪声、4% dropout、seed=11。改这些要在 Python 里传 `sim=`。

### 3.4 参数不要一起动的组合

| 组合 | 风险 |
|---|---|
| `kp_far` ↑ + `max_step` ↑ | 饱和减少但过冲/极限环放大 |
| `ki` ↑ + `integral_gate` ↑ | 大误差也积分，甩枪后回弹 |
| `kd` ↑ + `meas_noise` ↓ | 微分放大检测噪声，静止抖 |
| `lead_gain` ↑ + `tracker.accel_noise` ↑ | 速度估计更灵敏，前馈更容易过冲 |
| `deadzone` ↑ + `output_ema` ↓ | 双重平滑，微调变钝、跟枪滞后 |
| `switch_margin` ↑ + `fov_radius` ↑ | 锁死一个人，旁边的人打不到 |
| `fire.inset` ↓ + `fire.radius` ↑ | 开火窗口过大，半路开枪 |

---

## 4. 不要直接手改的字段

### 4.1 规则

- 用 `aim2 config set <path> <value>`，不要手改 JSON：`set_fields` 会做类型转换、范围/枚举校验、原子保存；手改会绕过校验，下一次 `load` 才报错。
- 改完用 `aim2 config diff --other 备份.json` 留下 diff；`Engine.apply` 会把每个字段的 old/new 写进日志。
- 改 `vision.*`/`capture.*`/`output.port|baud`/`runtime.control_hz` 后**重启**；热加载只覆盖 tracker/policy/aim/fire/output 的运行时字段。

### 4.2 当前无效或仅仿真字段

这些字段存在但当前代码没有消费点，不要用它们调手感：

| 字段 | 现状 |
|---|---|
| `aim.settle_px` | 仅 `sim/harness.py` 的首次捕获指标使用 |
| `capture.warmup_s` | `FrameGrabber`/`CvSource` 不使用 |

### 4.3 需要标定/重启才能改的字段

| 字段 | 原因 |
|---|---|
| `output.gain` | 世界系尺度；改了必须重标灵敏度，否则 tracker/前馈全错 |
| `policy.head_ratio` | 依赖模型框定义/视角；换模型/分辨率要重标 |
| `vision.model` / `vision.conf` / `vision.iou` / `vision.target_classes` | detector 在 `Engine.__init__` 时加载；`apply` 不重建 |
| `capture.*` | 采集卡已按旧参数打开；`apply` 只提示重启 |
| `output.port` / `output.baud` | 串口已打开；`apply` 只提示重启 |
| `runtime.control_hz` | `FixedRate` 在 `Engine.run` 启动时创建；热加载不会换节拍 |
| `runtime.log_level` / `log_dir` | `setup_logging` 只在进程启动时调用一次 |
| `runtime.record` / `record_dir` | 只在 CLI 启动时决定是否创建 `SessionRecorder` |

### 4.4 热加载的副作用

`Engine.apply` 用 `getattr(self.controller, "_kind", None)` 判断控制器是否变化，而控制器实际暴露的是 `kind`。结果是**任何**配置 diff 都会被判为「控制器变化」并重建控制器：

- `PredictiveAim.reset()` 清空积分、EMA、死区状态；
- 正在跟枪时改一个无关参数，输出会短暂归零/重建；
- 调参时尽量在目标消失/停火时改，或改完重启。

---

## 5. 仿真回归验证

### 5.1 场景

`sim/world.py::SCENARIOS`（确定性，纯时间函数）：

| 场景 | 时长 | 运动 | 主要考察 |
|---|---|---|---|
| `strafe_mid` | 6 s | 0.8 Hz 正弦，±120 px | 中距稳态跟踪 |
| `strafe_far` | 7 s | 0.55 Hz 正弦，±190 px，偏移 360 px | 远距/预判误差 |
| `jitter` | 5 s | 多频小幅抖动 | 噪声抑制 |
| `dash` | 5 s | 3 次 60 ms/约 200–500 px 瞬移 | 过冲/恢复 |
| `peek` | 6 s | 1.5 s 周期，0.85 s 可见 | coast/重捕获 |
| `micro_drift` | 6 s | 6 px/s + 3 px/s 慢漂 | 稳态误差/积分 |

### 5.2 指标

`sim/harness.py::SimMetrics`：

| 指标 | 定义 |
|---|---|
| `rms_px` | 全可见 tick 的误差 RMS |
| `p95_px` | 误差 95 分位 |
| `track_rms` | 去掉前 1/3 后的 RMS（稳态） |
| `first_acq_s` | 首次误差 ≤ `aim.settle_px` 的时间 |
| `on_target` | 准星落在真实框内的可见 tick 比例 |
| `recovery_s` | 从 >100 px 峰值回到 <10 px 的平均时间（`_recovery`） |
| `jitter_px` | 后半段输出幅值的标准差 |
| `hit_rate` | 仿真开火命中/开火次数 |

### 5.3 回归门禁

```bash
PYTHONPATH=src .venv/bin/python -m dma.cli bench
```

`cli.py::cmd_bench`：

- 对 6 个场景分别跑 `legacy`（v1 等价：60 Hz、无 tracker）与 `predictive`；
- 门禁按端到端质量，不按稳态 dither：
  - `new.rms_px > old.rms_px * 1.02`，或
  - `new.on_target < old.on_target - 0.01`，或
  - `new.jitter_px > old.jitter_px + 1.0`；
- 有失败 → 打印 `failures` 并返回 2；全过 → 0。

这也是为什么报告里 `peek`/`jitter` 的 `track_rms` 退化却仍能通过门禁：门禁看的是含捕获瞬态的 RMS、命中窗口和抖动，而不是只看稳态 dither。`track_rms` 仍在 `sim`/`autotune` 里用于诊断。

注意 `cmd_bench` 调 `compare()` 时**不读 `--settings`，也不传 `aim_cfg`**，用的是 `AimCfg()` 默认值。它验证的是「控制律默认行为」而不是你的 profile；要验证 profile 用 `aim2 sim`（它读 `settings.aim`），或用 Python 调 `simulate(..., aim_cfg=load().aim)`。

`docs/MEASUREMENTS.md` §4 的报告值（同一套仿真；当前代码 `bench` 输出会因配置/代码微调略有差异，以命令为准）：

| 场景 | 控制器 | rms_px | track_rms | on_target | recovery_s | jitter_px |
|---|---|---|---|---|---|---|
| `strafe_mid` | legacy | 22.48 | 21.58 | 0.53 | nan | 3.75 |
| `strafe_mid` | predictive | 10.64 | 9.60 | 0.99 | nan | 1.88 |
| `strafe_far` | legacy | 60.31 | 23.55 | 0.34 | 0.27 | 3.93 |
| `strafe_far` | predictive | 45.14 | 8.70 | 0.96 | 0.12 | 1.96 |
| `jitter` | legacy | 17.19 | 3.07 | 0.97 | 0.15 | 1.44 |
| `jitter` | predictive | 14.14 | 4.68 | 0.98 | 0.06 | 0.82 |
| `dash` | legacy | 56.04 | 64.22 | 0.90 | 0.16 | 10.51 |
| `dash` | predictive | 55.23 | 62.67 | 0.93 | 0.09 | 3.12 |
| `peek` | legacy | 45.87 | 2.05 | 0.93 | 0.25 | 0.99 |
| `peek` | predictive | 38.04 | 8.50 | 0.94 | 0.13 | 1.56 |
| `micro_drift` | legacy | 10.29 | 2.48 | 0.99 | nan | 1.22 |
| `micro_drift` | predictive | 8.72 | 3.59 | 0.99 | nan | 0.74 |

读法：

- 中距/远距/甩枪的 `on_target` 和 `recovery` 明显改善；
- `peek` 的 `track_rms` 反而变大（2.05 → 8.50），来源是 `policy.coast_s=0.18 s` 的外推：真实遮挡有用，长时间消失会多推。要更保守可把 `coast_s` 降到 0 或 0.08，再看 `first_acq_s`。
- `jitter` 的 `track_rms` 略大（3.07 → 4.68），但 `jitter_px` 更小；如果实机静止抖，优先调 `deadzone`/`output_ema`，不要只看这一项。

### 5.4 回放

```bash
# 录制（真机/视频）
PYTHONPATH=src .venv/bin/python -m dma.cli record --duration 60 --dry-run --no-hotkeys

# 摘要
PYTHONPATH=src .venv/bin/python -m dma.cli replay out/telemetry/session_x.jsonl.gz --summary

# 两控制器对比
PYTHONPATH=src .venv/bin/python -m dma.cli replay out/telemetry/session_x.jsonl.gz --compare
```

限制（必须知道）：

- `cmd_replay` 调用 `replay(session, kind)` 时**不传 `aim_cfg`**，所以用的是 `AimCfg()` 默认值，不是当前 `configs/settings.json`。要用当前配置，得用 Python 直接调 `telemetry.replay.replay(..., aim_cfg=load().aim)`。
- `telemetry/replay.py` 把每条检测的 `cls_name` 硬编码为 `"body"`，无法复现 head 类融合；它适合比较控制律，不适合验证锁头。
- 回放中心固定 `(640,360)`，只适用于 1280×720 会话；其他分辨率会偏。

### 5.5 推荐验证流程

1. `aim2 config migrate` 生成基线，`cp configs/settings.json` 备份。
2. `aim2 bench` 确认基线通过。
3. 改一个参数（`config set`）。
4. `aim2 bench`：不许比基线差；失败就回滚。
5. `aim2 sim --compare` 看单场景细节。
6. `aim2 replay SESSION --compare` 用真实轨迹复核（注意 §5.4 的默认 AimCfg 限制）。
7. `aim2 run --dry-run` 实机干跑 20–30 秒，看 `aim.err_px`、`aim.saturated`、`fire.reason`。
8. 实机开火前先关火，确认瞄准稳定；再开 `fire.mode=single`。
9. 记下 `config diff` 与结果，写进变更记录。

---

## 6. 现象 → 参数速查

| 现象 | 首选 | 次选 | 不要 |
|---|---|---|---|
| 甩枪慢 | `max_step` ↑、`kp_far` ↑ | `far_px` ↑ | 先加 `ki` |
| 目标附近抖 | `kd` ↑、`deadzone` ↑ | `output_ema` ↓、`kp_near` ↓ | 加 `lead_gain` |
| 跟枪滞后 | `lead_gain` ↑、`ki` ↑ | `integral_gate` ↑ | 盲目加 `kp_far` |
| 甩枪回弹 | `max_step` ↓、`kp_far` ↓ | `lead_max` ↓、`integral_gate` ↓ | 加 `kd` 掩盖 |
| 多目标摇摆 | `switch_margin` ↑、`fov_radius` ↓ | `coast_s` ↓、`tracker.iou_thr` ↑ | 同时降 `min_hits` |
| 往队友拉 | 换 sjz 模型、确认 `ignore_classes` | `policy.min_hits` ↑ | 用 `head_ratio` 修 |
| 打不到头 | 有 head 类就查融合/`head_bonus`；无 head 类重标 `head_ratio` | `fire.inset` ↓ | 把 `fire.radius` 拉大掩盖 |
| 不开火 | 看 `fire.reason` | `fire.radius`/`inset`/`dwell` | 直接 `mode=auto` |
| 静止时抖 | `deadzone`/`deadzone_exit` ↑、`output_ema` ↓ | `tracker.meas_noise` ↑ | 降 `max_step` 太多 |
| 快速变向跟不上 | `tracker.accel_noise` ↑ | `kp_far` ↑（谨慎） | 只加 `lead_gain` |
| 目标丢失后乱推 | `policy.coast_s` ↓ | `aim.lead_gain` ↓（coast 时已 ×0.4） | 关 tracker |

---

## 7. 变更记录模板

每次调参留一段，放在 `out/` 或 PR 描述里：

```markdown
### YYYY-MM-DD 变更标题
- 配置：configs/settings.json（profile=...）
- diff：
  - aim.kp_far: 0.85 -> 0.75
  - policy.coast_s: 0.18 -> 0.10
- 原因/现象：
- 验证：
  - `aim2 bench`：通过（failures=[]）
  - `aim2 replay SESSION --compare`：predictive track_rms ...（注意默认 AimCfg 限制）
  - 实机 60s：latency loop_ms=..., confidence=..., aim.err_px p50/p95=..., fire.reason 分布=...
- 回滚：cp configs/settings.bak.json configs/settings.json
```

---

本文档对应 schema_version 2 与 src/dma 布局，改动代码请同步更新。
