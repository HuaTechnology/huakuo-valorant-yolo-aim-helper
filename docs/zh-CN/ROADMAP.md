<div align="right">

**简体中文** | [English](../ROADMAP.md)

</div>

# 路线图

标注 `help wanted` 的项欢迎 PR，见 [CONTRIBUTING](../../CONTRIBUTING.md)。

> ⚠️ 仅供学习与研究，请勿用于任何竞技/排位场景。见 [DISCLAIMER](../../DISCLAIMER.md)。

## 近期（可独立完成）

- [x] **文档英文化**：`docs/` 为英文标准版，`docs/zh-CN/` 为中文镜像
- [ ] **演示素材**：合成画面（`aim2 run --sim`）录 5 秒 GIF 放 README 顶部
- [ ] **更多输出头**：RT-DETR / YOLOv10 的解码与形状判定（各配纯 numpy 单测）
- [ ] **Issue 模板补充**：崩溃日志模板（附 JSONL 会话尾部）

## 中期（需要硬件或跨平台）

- [ ] Windows 原生采集（Desktop Duplication / DXGI），去掉采集卡这一跳
      —— 这是延迟的质变路径，见 `docs/MEASUREMENTS.md` §1
- [ ] 更高帧率支持：120 fps 采集下的控制律验证（速率归一化已就绪，缺实测）
- [ ] Linux 下的 UVC + V4L2 采集路径与 CI 冒烟测试
- [ ] ESP32 固件：USB 描述符自检、双口拓扑下的自动化烧录脚本

## 长期（研究性）

- [ ] 多模型融合（身体类 + 头类 + 队友类）与置信度加权
- [ ] 自适应延迟估计：用卡尔曼新息而不是残差中位数做延迟辨识
- [ ] 参数自整定：用回放器在**真实轨迹**上做贝叶斯优化，替代 `autotune` 的坐标下降

## 不做

- 内存读取 / 驱动注入 / 反作弊对抗：本项目只做「读画面、写 HID」。
- 任何形式的模型权重分发：git 树里永远不含 `.onnx`。
