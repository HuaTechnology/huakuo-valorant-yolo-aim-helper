<div align="right">

**简体中文** | [English](NOTICE)

</div>

huakuo-valorant-yolo-aim-helper
Copyright (C) 2026 huakuo-valorant-yolo-aim-helper contributors

This program is free software: you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this
program.  If not, see <https://www.gnu.org/licenses/>.

--------------------------------------------------------------------------------
第三方组件
--------------------------------------------------------------------------------

运行时依赖（均为宽松许可证，与本项目的 GPL-3.0 兼容）：

  numpy            BSD-3-Clause
  opencv-python    Apache-2.0（OpenCV 本体为 Apache-2.0）
  onnxruntime      MIT
  pyserial         BSD-3-Clause

可选依赖：

  PySide6          LGPL-3.0（动态链接使用；分发二进制时需保留其许可证与替换方式）
  pynput           LGPL-3.0（同上）

固件：

  Arduino-ESP32 core 3.x   LGPL-2.1（以库形式链接；固件源码随本仓库以 GPL-3.0 发布）

--------------------------------------------------------------------------------
不包含的内容
--------------------------------------------------------------------------------

* 模型权重不以 git 内容分发。作者自用的示例模型（`7-wa-v26s-320x.onnx`）以
  **Release 附件**形式提供，仅供个人学习与研究，**不得再分发**，也不得用于任何
  竞技/排位场景；其权利归原始提供者，本项目不对其准确性或适用性作任何担保。
* 使用自备模型时，请自行确认你拥有相应的使用许可。
* 游戏画面、录像、截图 —— 版权归各自权利人。
* 任何厂商名称、商标仅用于说明兼容的输出格式，不表示隶属或背书关系。
