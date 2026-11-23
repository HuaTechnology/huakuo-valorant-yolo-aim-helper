<div align="right">

**简体中文** | [English](CHANGELOG.md)

</div>

# 变更记录

遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/) 与 [SemVer](https://semver.org/lang/zh-CN/)。

## [2.0.0] - 2026-11-23

首个公开版本：把「单文件脚本 + 手调 PID」重写为可测量、可回放、可回归的流水线。

### 新增
- 分层架构：`core / capture / perception / control / transport / engine / sim / tuning / telemetry / desktop`
- 三种检测输出头解码（端到端 / YOLOv5 / YOLOv8）+ 分类别 NMS + 类别白名单
- Kalman 多目标跟踪（ID / 世界系速度 / 位置协方差 / 丢帧外推）
- 世界坐标系与延迟采样线；在线延迟辨识（含物理下限与三重拒绝条件）
- 速度前馈 + 速率归一化的控制律（60/240 Hz 同增益同行为）
- 协议 v2：CRC8、序号、设备 STATUS 遥测；子帧运动规划器（余数保留）
- 引擎：四线程 + 快照 + 健康看门狗 + 热加载
- 闭环仿真台与回归门禁 `aim2 bench`；离线自动调参 `aim2 autotune`
- 会话录制与闭环回放；`aim2 selftest` / `link-test` / `calibrate`
- PySide6 控制台（UI 由配置注册表生成）
- ESP32-S3 固件：1000 Hz HID、CRC、deadman、STATUS 遥测

### 修复（相对内部开发版）
- 延迟估计可能塌到物理不可能的 2.5 ms，导致延迟型积分对象进入极限环
- 界面刷新会把用户刚打开的开关按回去（信号回环）
- 迁移旧配置时误把控制增益当同类参数搬运
- 固件报告率统计的是「调用次数」而非「送达次数」，USB 未连接时也显示正常

### 附件
- `7-wa-v26s-320x.onnx`（约 36 MB）：作者自用的 320×320 端到端示例模型，放进 `models/` 即可。
  仅有身体类（`st`），锁头依赖 `policy.head_ratio`；仅供个人学习研究，不得再分发，禁用于竞技场景。

### 已知限制
- 无头/队友类的模型仍需比例标定与人工避让
- 视觉链路约 2 帧固有延迟
- 延迟辨识在目标静止时无观测信号，保持先验
