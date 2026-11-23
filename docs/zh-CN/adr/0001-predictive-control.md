# ADR-0001：用预测控制替代纯 PID

<div align="right">

**简体中文** | [English](../adr/0001-predictive-control.md)

</div>

- 状态：已接受（Accepted）
- 日期：2026-11-21
- 决策范围：`src/dma/control/controller.py`、`src/dma/control/latency.py`、`src/dma/engine/runtime.py`
- 相关：ADR-0002（世界坐标跟踪）、[LATENCY_BUDGET](../LATENCY_BUDGET.md)、[TUNING](../TUNING.md)

---

## 1. 背景

v1 的控制律是「PD + 抗饱和积分 + 距离自适应 EMA + 分段 kp」：

- `aim.py::AimController.update` 对**过期的屏幕误差**做 PID；
- 增益是在 60 Hz 主循环里手调的，没有「控制频率」概念；
- 延迟靠 `predict.py` 的启发式估计，默认还经常关掉；
- 串口按 127 分块发送，**余数丢弃**，所以必须保留 4 px 死区才能不抖。

docs/MEASUREMENTS.md记录的现象：

| 现象 | v1 解释 |
|---|---|
| 高增益 → 「弹簧」震荡 | 延迟 + 高增益 → 过冲极限环（docs/MEASUREMENTS.md） |
| 低增益 → 跟枪滞后 | 对过期误差做比例控制，稳态滞后 ∝ 速度/增益 |
| 改采集/控制帧率就重调 | 增益是「每帧」量纲，不是「每秒/每生效窗口」 |
| 甩枪后回弹 | 积分 windup + 无前馈/延迟补偿 |
| 实测 D=1 帧，但不敢用 | 延迟估计不可信，预判默认关闭（`config.py` 的 `PREDICT_FRAMES=0`） |

v1 代码注释里还有两组仿真结论（`aim.py` docstring）：

```
D=2: 初始过冲≈13px，跟枪滞后≈8.7px
D=3: 初始过冲≈44px，跟枪滞后≈8.4px
```

这说明：**同样的增益，延迟从 2 帧变 3 帧，过冲从 13 px 变成 44 px**。纯 PID 在延迟变化时没有稳定的工作点，只能不断手调。

---

## 2. 决策

把控制对象显式建模为**带延迟的积分器**，控制律围绕这个模型设计，而不是继续给 PID 打补丁。

### 2.1 被控对象

```
e(t) = p(t - L_measure) - c(t - L_apply)
```

- `p`：目标位置（屏幕系）。
- `c`：我们累计发出的位移（屏幕系）。
- `L_measure`：画面产生 → 检测可用。
- `L_apply`：命令发出 → 画面/准星生效。
- `L = L_measure + L_apply`：闭环总延迟，由 `LatencyEstimator` 在线辨识。

`Engine.step` 把 `L` 拆成：

```python
age_s   = det_age + split * loop_s          # 目标外推到「现在」
apply_s = max(dt, (1 - split) * loop_s)     # 命令生效窗口
lead_s  = loop_s + det_age                  # 开火预览
```

### 2.2 控制律

`PredictiveAim.step`：

```
scale = clamp(dt / apply_s, 0.02, 1.0)
kp    = kp_near + (kp_far - kp_near) * smoothstep(near_px, far_px, dist)
ff    = clamp(v * apply_s * lead_gain, ±lead_max)     # coasting 时 ×0.4
u     = scale * (kp*e + ki*I + ff) + kd*(e - e_prev)
u     = slew_limit(u)
u     = clamp_magnitude(u, max_step * scale)
I    += e*dt（仅 dist < integral_gate），clamp 到 integral_limit
饱和时 I 上限 = (max_u - |PD+FF|) / ki
```

五个设计点：

1. **速度前馈放在 kp 之外**。前馈补偿的是「目标在 `apply_s` 内自己会走多远」，不应该被比例增益缩放；否则纯比例环对匀速目标的稳态滞后是 `v*apply/kp`，这正是 v1「跟枪滞后」的来源。
2. **前馈限幅** `lead_max=70 px`。速度估计来自 Kalman，坏帧/瞬时误检可能给出很大的 `v`；限幅保证一次误判不会变成 600 px 过冲。coasting 时前馈再乘 0.4，因为外推速度不如实测可信。
3. **速率归一化** `scale = dt/apply_s`。控制律的语义是「在命令生效窗口内走多少像素」，不是「每 tick 走多少」。没有 `scale`，240 Hz 环每单位时间的位移是 60 Hz 的 4 倍，延迟下必然震荡——这正是 v1 增益「只在 60 fps 有效」的根因。
4. **增益调度**：远距 `kp_far`（默认 0.85）接近死拍，近距 `kp_near`（默认 0.50）更保守，因为近距误差更多来自检测噪声/延迟估计误差，而不是目标真实运动。用 `smoothstep(26,260)` 连续过渡，避免 v1 分段 kp 的硬切换。
5. **抗饱和用余量投影**，不是简单 clamp：饱和时积分器只能保留「输出预算减去 P/D/FF 已占用」的部分。v1 的 `integral_limit` 固定 100 px，甩枪时积分继续累积，甩完就回弹。

### 2.3 延迟辨识

`LatencyEstimator` 在**检测事件**上做一维系统辨识：

```
cost(m) = median_k |Δe_k + u_{k-m}|      （二维取 L1 和）
m*      = argmin cost(m)
conf    = 1 - cost(m*) / median(cost)
```

- 候选 `m ∈ [0,10]` 帧，窗口 45 个检测样本。
- 只有 `conf >= latency_min_conf`（默认 0.2）才用 `loop_s` 平滑更新（`smoothing=0.25`）。
- 目标静止/我们不动时，误差曲线平坦，`conf` 低，保持先验——这是有意的，不做「无激励辨识」。

为什么不用分段计时器：`perception.infer_ms`/`transport.write_ms` 只能看到 5–15 ms 的段；真正主导的采集卡/显示/游戏输入 tick 在主机侧不可见，只能在闭环里估。

### 2.4 保留 legacy

`aim.controller="legacy"` 仍然可切到 `LegacyAim`（包装 v1 `AimController`）：

- 用于 A/B、回放对照、以及「v2 引擎 + v1 控制律」的降级运行；
- 它不是完整 v1 回滚（世界系/线程/协议仍是 v2），见 [OPERATIONS §7](../OPERATIONS.md#7-回滚到-v1)。

---

## 3. 放弃的方案

| 方案 | 为什么放弃 |
|---|---|
| **纯 PID，提高 kp 硬扛** | 延迟变化时没有稳定工作点；v1 的 D=2→3 过冲 13→44 px 就是证据。高 kp 在延迟下形成极限环，低 kp 稳态滞后，二者不可兼得 |
| **完整 Smith 预测器** | 需要精确的模型/队列状态；把「已发未生效位移」反馈进预测，检测抖动会被放大。`predict.py` docstring 记录 v1 实测：完整 Smith 抖动大，限幅前馈把滞后从 9 px 降到 3–4 px 且抖动保持基线。v2 进一步去掉了输出队列（`MotionPlanner` 只保留累加器），Smith 的「pending 队列」前提也不成立 |
| **位置死区补偿** | 用死区掩盖量化误差，代价是近距永远有 4 px 死区（v1 的 `deadzone=4.0`）。v2 用 `MotionPlanner` 携带余数，亚像素误差可以累积到 1 count，死区降到 1.5 px 只用于「保持」而不是「掩盖」 |
| **固定帧数预判**（v1 `PREDICT_FRAMES`） | 帧数不是物理量；60→120/240 Hz 后同一「2 帧」含义全变。v2 用秒 + 在线辨识，`lead_max` 限幅 |
| **MPC / LQR** | 对象是一维延迟积分器，模型误差主要来自延迟估计和检测噪声，不是高阶动态；4.17 ms 控制预算里做 QP 没有收益，且验证成本高。当前用 6 个参数的坐标下降 + 闭环仿真就能覆盖 |
| **学习型控制（RL/模仿）** | 不可解释、不可回放、安全边界难保证；没有足够真实数据前不纳入实时回路 |

---

## 4. 后果

### 4.1 正面

- 延迟变化时只需保证 `loop_s` 估计可信，增益不需要随帧率重调。
- 中距/远距/甩枪的仿真指标显著改善（`docs/MEASUREMENTS.md` §4）：`strafe_mid` 命中窗口 0.53→0.99、`strafe_far` 0.34→0.96、`dash` 恢复 0.16→0.09 s。
- `peek`/`jitter` 的稳态 `track_rms` 有退化（2.05→8.50、3.07→4.68），来源是 `coast_s` 外推和较弱输出平滑；这是已知代价，调参见 [TUNING §5.3](../TUNING.md#53-回归门禁)。

### 4.2 成本/风险

| 风险 | 缓解 |
|---|---|
| 延迟估计错误 → 前馈/外推方向错 | `confidence` 门限 + 先验；`lead_max` 限幅；低置信度不调高 `kp_far` |
| 世界系尺度错 → 速度估计错 | 强制 `output.gain` 标定（[CALIBRATION §3](../CALIBRATION.md#3-灵敏度与增益标定)） |
| 近距噪声 → 抖 | 死区滞回 + `output_ema` + 低 `kp_near`；不要用提高 `kd` 硬压 |
| 积分 windup | 条件积分 + 余量投影 + 换靶 `controller.reset()` |
| 换靶状态残留 | `Engine.step` 在 `target.tid` 变化时重置控制器（v2 新增） |
| 参数语义变化 | `legacy` 保留；`aim.*` 字段有 `FIELDS` 帮助文本；`config diff` 审计 |

### 4.3 对测试/运维的影响

- 需要 `LatencyEstimator` + 世界系 + 标定流程；不是「只改一个 kp」.
- `aim2 bench` 把「预测律必须不差于 v1」做成门禁；`aim2 sim/replay` 提供可复现对照。
- 实时调参时任何 `Engine.apply` 都会重建控制器（`kind` 判断问题），积分/EMA 清零；见 [OPERATIONS §6.3](../OPERATIONS.md#63-瞄准控制)。

---

## 5. 证据与代码位置

| 结论 | 位置 |
|---|---|
| 延迟积分器模型 | `control/controller.py` 模块 docstring |
| 速率归一化 | `PredictiveAim.step` 的 `scale = clamp(dt/apply_s, ...)` |
| 前馈在 kp 外 + 限幅 | `PredictiveAim.step` 的 `ff_x/ff_y` |
| 条件积分 + 余量投影 | `PredictiveAim.step` 的 `integral_gate` / `headroom` |
| 增益调度 | `PredictiveAim.step` 的 `smoothstep(near_px, far_px, dist)` |
| 延迟辨识 | `LatencyEstimator._estimate` |
| 换靶重置 | `Engine.step` 的 `_ctrl_tid` 分支 |
| v1 仿真结论 | `src/dma/aim.py` docstring（D=2/3 的过冲/滞后） |
| v1 Smith 前馈结论 | `src/dma/predict.py` docstring |
| 闭环对比 | `docs/MEASUREMENTS.md` §4、`sim/harness.py` |

---

## 6. 何时重新评估

- 采集从 60 fps 提升到 120/240 fps：重跑 `aim2 bench`，确认 `kp` 调度仍合适（速率归一化保证量纲不变，但检测噪声谱会变）。
- 出现可变延迟/抖动（WiFi 串口、USB hub、游戏输入 tick 不稳）：考虑增益随 `confidence` 自适应，或对 `loop_s` 加方差惩罚。
- 引入带 head/teammate 的模型后，选靶/融合误差成为主误差源：控制律不是瓶颈，不要再加增益。
- 若未来能拿到游戏输入 tick 的精确时间（驱动/API），可把 `L_apply` 从估计值改为测量值，`latency_split` 可以固定为物理比例。

---

本文档对应 schema_version 2 与 src/dma 布局，改动代码请同步更新。
