# models/

本目录**不参与版本控制**（`.gitignore` 只放行本说明文件）。请把你自己的 `.onnx` 放在这里，
并让 `vision.model` 指向它。

检测器会自动识别三种常见输出头，不需要改代码：

| 输出张量 | 家族 | 解码方式 |
|---|---|---|
| `[1, N, 6]` | 端到端导出（已内置 NMS） | `x1,y1,x2,y2,conf,cls` |
| `[1, 25200, 5+nc]` | YOLOv5 | objectness × class + 分类别 NMS |
| `[1, 4+nc, 2100]` 或转置 | YOLOv8 | anchor-free，无 objectness |

## 示例模型

作者自用的 320×320 端到端导出以 **Release 附件**形式提供：

- 下载：<https://github.com/HuaTechnology/huakuo-valorant-yolo-aim-helper/releases/latest>
- 文件名：`7-wa-v26s-320x.onnx`（约 36 MB）
- 放置：本目录，然后 `python -m dma.cli config set vision.model models/7-wa-v26s-320x.onnx`

该权重只有身体类（`st`），锁头依赖 `policy.head_ratio`；仅可用于**个人学习与研究**，
请勿再分发，也不要用于任何竞技场景（见仓库根目录 `DISCLAIMER.md`）。

验证：

```bash
python -m dma.cli models inspect models/7-wa-v26s-320x.onnx
python -m dma.cli models bench models/7-wa-v26s-320x.onnx --runs 20
```
