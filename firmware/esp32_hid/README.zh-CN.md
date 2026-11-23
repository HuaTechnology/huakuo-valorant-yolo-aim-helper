<div align="right">

**简体中文** | [English](README.md)

</div>

# ESP32 HID 桥固件（esp32_hid）

> ⚠️ **仅供学习与研究，请勿用于任何竞技/排位场景。** 详见仓库根目录 [DISCLAIMER](../../DISCLAIMER.md)。

串口二进制帧 → **原生 USB HID 鼠标**的桥固件。它只做一件事：把主机 `aim2` 送来的
位移/按键帧，按 1000 Hz 拆成 HID 报告发给被控机器。所有瞄准逻辑、坐标换算、
平滑/预测都在主机侧（`src/dma/`），本固件**不做任何决策**，只保证时序、无损、
及故障时立刻松手。

```
macOS: aim2 ──USB CDC 921600──► ESP32-S3 ──USB HID──► 目标机器/游戏
                (v2 二进制帧)               (鼠标报告 1000 Hz)
```

一条 USB 线同时承载 **CDC 串口 + HID 鼠标**（TinyUSB 复合设备），所以主机看到的是
`/dev/cu.usbmodem*`，被控端看到的是一个普通 USB 鼠标。

---

## 1. 接线拓扑（先说清楚哪根线插哪台机器）

### 1.0 材料清单（整套系统就这四样）

| # | 物料 | 数量 | 说明 |
| --- | --- | --- | --- |
| 1 | HDMI 采集卡（UVC / MS2109 之类） | 1 | Windows 的 HDMI 输出接它，它再插 Mac 的 USB |
| 2 | HDMI 线 | 1 | Windows 显卡/主板 HDMI → 采集卡 HDMI IN |
| 3 | ESP32-S3 开发板（双 USB 口：`USB` + `UART`） | 1 | 原生 USB 出 HID，板载 UART 桥收协议 |
| 4 | USB Type-C 数据线（不是充电线） | **2** | 一根 ESP32 `UART` 口 → Mac；一根 ESP32 `USB` 口 → Windows |

整套连线只有这四条，任何一条接错都表现为「不动」：

```
Windows ──HDMI──▶ 采集卡 ──USB──▶ Mac        （画面方向）
Mac ──Type-C──▶ ESP32 `UART` 口              （控制方向：协议帧 921600）
ESP32 `USB` 口 ──Type-C──▶ Windows            （输出方向：HID 鼠标）
```

本项目是**双机拓扑**，ESP32 同时连两台电脑：

```
                 ┌──────────── 原生 USB 口（丝印 USB）────────▶ Windows（游戏机）
                 │                                            ↑ HID 鼠标注入
   ESP32-S3 ─────┤
                 └──────────── UART 口（丝印 UART，板载 CP2102/CH343）──▶ Mac（推理机）
                                                              ↑ 协议帧 921600
```

| 口 | 丝印 | 对端 | 用途 | 主机侧设备名 |
| --- | --- | --- | --- | --- |
| 原生 USB | `USB` | **Windows** | 只做 HID 鼠标 | 设备管理器里 `USB 输入设备 / HID-compliant mouse` |
| UART | `UART` | **Mac** | 只跑协议帧（v2，CRC8） | `/dev/cu.usbserial-*`、`/dev/cu.wchusbserial-*` |

因此固件必须用 **`USB_MODE=0` + `CDC_ON_BOOT=0`**：`Serial` 落在 UART0(GPIO43/44)，
也就是 UART 口；原生 USB 只承载 HID。默认 env `esp32-s3-uart-link` 就是这个配置。

> 如果你的推理机与游戏机是同一台（单机拓扑），改用 env `esp32-s3-single-usb`
> （`CDC_ON_BOOT=1`），此时串口和 HID 共用一个 USB 口，主机侧串口是 `/dev/cu.usbmodem*`。
> 两种拓扑下 `SerialSink` 的自动发现都能命中。

| 项 | 要求 |
| --- | --- |
| 主控 | ESP32-S3-DevKitC-1（推荐）；ESP32-S2 亦可（单核，报告任务与解析任务共核） |
| 供电 | 原生 USB 供电即可；HID 类设备 100 mA 足够 |
| 线材 | 必须带数据线；HID 那根不要过 Hub/KVM/延长线（会拉低轮询率、偶发掉线） |
| 指示灯 | 无；状态全部走 STATUS 帧遥测 |

---

## 2. 板级设置（**错了 HID 一定不工作**）

Arduino IDE 2.x + `esp32` core **3.x**（Boards Manager 选 esp32 by Espressif 3.x）。

| 选项 | 双机拓扑（本项目） | 单机拓扑 |
| --- | --- | --- |
| Board | `ESP32S3 Dev Module` | 同 |
| **USB Mode** | **`USB-OTG (TinyUSB)`** | 同（唯一能让 `USBHIDMouse` 枚举的模式） |
| **USB CDC On Boot** | **`Disabled`**（`Serial`→UART0） | `Enabled`（`Serial`→USB CDC） |
| Flash Size / PSRAM / Partition | 按板子实际 / 不用 / 默认 | 同 |
| Upload Speed | `921600`（不稳降到 `115200`） | 同 |
| CPU Frequency | `240 MHz` | 同 |

PlatformIO 对应 `env:esp32-s3-uart-link`（默认）/ `env:esp32-s3-single-usb`。

### 为什么 `USB_MODE` 不能是 1

`ARDUINO_USB_MODE=1` 是「硬件 USB-Serial/JTAG 外设」，此时 TinyUSB 不初始化，
`USBHIDMouse` 类根本不会被编译进固件，HID 永远不枚举。板卡清单默认就会注入
`-DARDUINO_USB_MODE=1`，所以 `platformio.ini` 里先用 `build_unflags` 摘掉，再显式给 `0`。
`[env:esp32-s3-hwcdc-jtag]` 保留了这个字面组合，**仅作对照，不可用于运行**。

---

## 3. 构建与烧录

### 3.0 关键结论：烧录不需要把 UART 线从 Mac 上拔下来

ESP32-S3 的 ROM 下载模式同时支持**两个物理口**：UART0 和原生 USB（USB-Serial/JTAG）。
你现在是「UART→Mac、USB→Windows」，两条路都能烧，**都不用动线**：

| 烧录方式 | 用哪个口 | 操作 | 适用 |
| --- | --- | --- | --- |
| **A. Windows 上烧（推荐，回答你的问题）** | 原生 USB（已在 Windows） | 板子进下载模式：按住 **BOOT** → 点一下 **RST** → 松开 BOOT；Windows 设备管理器出现 `USB JTAG/serial debug unit (COMx)`；用它烧 | Windows 上有 Arduino IDE / PlatformIO / esptool |
| **B. Mac 上烧** | UART（已在 Mac） | `pio run -t upload`（默认 env）或 esptool 指向 `/dev/cu.usbserial-*`；S3 DevKit 的 UART 口带 DTR/RTS 自动复位，一般无需按 BOOT | Mac 上有工具链，或想一条命令烧完 |

- 方式 A 烧完按 **RST**（或 esptool `--after hard_reset`）→ 固件启动 → Windows 重新枚举 HID。
  Mac 侧的 `/dev/cu.usbserial-*` 全程不受影响，因为 ROM 下载模式走的是原生 USB，不占 UART0。
- 方式 A 的前提是**能进下载模式**：若 BOOT 键不好按，可短接 GPIO0 到 GND 再复位。
- 若你坚持「COM 必须在 Windows」：那只是把 UART 线换到 Windows 上，用于 **B 方案**；
  但烧完要把它插回 Mac，否则 Mac 侧的 `SerialSink` 找不到串口（`aim2 doctor` 会报
  「未找到串口设备」）。**先试 A，通常不用换线。**

### 3.1 PlatformIO

```bash
cd firmware/esp32_hid
pio run                       # 双机拓扑（默认 env，协议走 UART0）
pio run -t upload             # 烧录（UART 口在 Mac 上时走这里；换到 Windows 则用 A 方案）
pio run -e esp32-s3-single-usb -t upload   # 单机拓扑
pio device monitor -b 921600  # 双机拓扑下 monitor 看的是 UART 口
```

### 3.2 Arduino IDE

1. 打开 `firmware/esp32_hid/esp32_hid.ino`（同目录缺其它文件提示可忽略）；
2. Tools 按第 2 节设置（双机拓扑：`USB-OTG (TinyUSB)` + `CDC On Boot: Disabled`）；
3. 端口选 **UART 口**（`/dev/cu.usbserial-*`，Mac 上烧）或 **原生 USB 的下载模式 COM**
   （Windows 上烧，先 BOOT+RST）；Upload；
4. 失败时先确认是否真的进了下载模式（Windows 看 COM 号是否变化，Mac 看 `ls /dev/cu.*`）。

烧录完成后的自检：

- Windows：设备管理器应出现 `HID-compliant mouse`（拔插一次确认）；
- Mac：`ls /dev/cu.usbserial-*` 有设备，且 `python -m dma.cli doctor` 的 serial 一行为 OK；
- 联调：`python -m dma.cli run --dry-run` 看健康状态是否 `ok`，再 `run` 观察鼠标是否响应。

## 4. 上电行为

| 时间 | 行为 |
| --- | --- |
| 0 ms | `Serial`(CDC) + `USB.begin()` + `Mouse.begin()`，创建 3 个任务；`g_last_rx_us` 从此刻起算 |
| 0–300 ms | **deadman 状态**：不输出任何 HID，STATUS 的 `flags` bit0 = 1（正常现象） |
| 收到第一个合法帧 | deadman 解除，`flags` bit0 = 0，开始按 1000 Hz 输出 |
| 每 100 ms | 主动上报 STATUS（uptime / 计数 / 报告率 / flags） |
| 连续 300 ms 无合法帧 | 立刻 `Mouse.release(MOUSE_LEFT)`、清空累加器、停止输出（`flags` bit0 = 1） |

上电横幅（纯 ASCII，无 `0xA5`，主机解析器会当垃圾丢弃且不计 CRC 错）：

```
[esp32-hid] v2 proto, 1000 Hz, deadman 300 ms
```

---

## 5. 线协议（与 `src/dma/transport/protocol.py` 逐字节一致）

```
off 0 : 0xA5                                  magic
off 1 : (2 << 4) | type                       version(高nibble) | type(低nibble)
off 2 : seq                                   序号，回绕
off 3 : len                                   payload 长度
off 4 : payload[len]
last  : CRC-8/ATM (poly 0x07, init 0, MSB-first, 不反射)，覆盖 bytes[0 : 4+len]
```

| type | 方向 | payload | 固件行为 |
| --- | --- | --- | --- |
| 1 `MOVE` | 主机→设备 | `int16 LE dx`, `int16 LE dy` | 只累加到 `int32` 累加器；**绝不在解析上下文发 HID** |
| 2 `BUTTON` | 主机→设备 | `uint8 mask`（bit0 = 左键） | 更新目标态 + 立即唤醒报告任务（µs 级响应，主机 tap 30 ms 不会丢） |
| 3 `PING` | 主机→设备 | `uint32 LE counter` | 喂看门狗（不回包） |
| 4 `STATUS` | **设备→主机** | 13 字节，见下 | 每 100 ms 主动上报 |
| 5 `CONFIG` | 主机→设备 | 任意 | 保留：校验 CRC 后忽略，计入 `unknown`，同时视为链路存活 |

**重同步**（与主机 `StreamParser` 同语义）：找不到 `0xA5` → 整段丢弃且不计错；
`len > 128`、版本不是 2、CRC 不符 → **只丢 1 字节**继续找；半帧则等后续字节。
这样热插拔、固件重启、USB 抖动都不会把链路焊死。

### STATUS payload（13 字节，`struct "<IHHHHB"`）

与主机 `protocol.STATUS_LEN == 13` / `decode_status()` 的 `struct.unpack("<IHHHHB", payload[:13])` 严格一致：

| 偏移 | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0..3 | `uptime_ms` | `uint32` LE | 上电毫秒数（`esp_timer`，约 49 天回绕） |
| 4..5 | `rx_frames` | `uint16` LE | 收到的**合法**帧数（MOVE/BUTTON/PING/CONFIG 全计），饱和到 65535 |
| 6..7 | `crc_errors` | `uint16` LE | CRC / 长度 / 版本错次数，口径与主机 `StreamParser.crc_errors` 相同 |
| 8..9 | `queue_drops` | `uint16` LE | 被丢弃的位移量（累加器饱和 / CRC 风暴 / deadman 清理），单位「报告数」 |
| 10..11 | `report_hz` | `uint16` LE | 最近 100 ms 实际发出的 HID 报告数折算成 Hz（无位移时为 0，1000 可如实上报） |
| 12 | `flags` | `uint8` | bit0 = deadman 生效中；bit1 = USB 已就绪/已枚举；其余位保留为 0 |

整帧 18 字节（magic + ver/type + seq + len + 13B payload + CRC）。字段全部手写小端写入，
不依赖编译器的 struct 打包（协议是字节序契约，不是编译器契约）。

> 历史兼容：若对接早期 12 字节布局（`"<IHHHBB"`，`report_hz` 为 `uint8`，1000 Hz 会饱和成 255），
> 把源码顶部 `STATUS_HZ_AS_U8` 置 `true` 即可回退；正常构建保持 `false`。

---

## 6. 时序设计（为什么这样写）

| 关注点 | 做法 |
| --- | --- |
| 谁碰 HID | 只有 `report_task`。`Mouse.move/press/release` **全文件仅这三处调用点**，且同任务串行——`Mouse.move()` 会阻塞到下一个 HID 报告周期，多任务交叉调用会导致报告乱序、光标抖动 |
| 报告节拍 | `report_task` 固定在 **core 1**，优先级 20；用「截止时刻 + 可被通知打断的等待」实现 `vTaskDelayUntil` 的定速语义，最后 ≤250 µs 用自旋对齐，避免 tick 量化把 1000 Hz 变成 1~2 ms 抖动 |
| 按键延迟 | `BUTTON` 帧在解析任务里 `xTaskNotifyGive()` 唤醒报告任务 → 立刻下发 HID，实测 µs 级，远小于主机 30 ms tap |
| 位移无损 | 单次 `Mouse.move` 每轴 ≤ ±127，超出部分留在累加器（上限 ±508，与主机 `planner` 的 `4×max_per_report` 同值），下一次报告继续发，**余数永不丢**。近目标时控制器才能收敛到亚像素 |
| 抖动隔离 | 解析任务在 **core 0**（与 USB/TinyUSB 同核），报告任务独享 **core 1** |
| CRC 风暴 | 连续 32 次 CRC 错 → 清空累加器（噪声不允许变成位移），`queue_drops` 记账 |
| deadman | 300 ms 无合法帧 → 松键 + 停输出 + 清累加器；不松手是最危险的故障模式 |
| 遥测 | 独立 `status_task`（core 0，低优先级）每 100 ms 发 STATUS；只在 CDC 真正挂载时写，避免写入阻塞拖慢其它任务 |

任务布局：

| 任务 | core | 优先级 | 栈 | 职责 |
| --- | --- | --- | --- | --- |
| `hid_rx` | 0 | 8 | 4096 | 轮询 CDC → 字节流解析 → 累加器/按键/看门狗 |
| `hid_report` | 1 | 20 | 3072 | 1000 Hz：deadman 判定 → 取子帧 → `Mouse.move()` |
| `hid_status` | 0 | 3 | 3072 | 100 ms STATUS 遥测 + 报告率折算 |

可调常量全部集中在源码顶部 `namespace fw`：`REPORT_HZ`、`DEADMAN_MS`、
`MAX_PER_REPORT`、`AXIS_CAP`、`CRC_STORM_LIMIT`、`STATUS_PERIOD_MS`、任务核心/优先级/栈、
`SERIAL_RX_BUFSZ`（CDC 环缓 2048 B，921600 bps 下 ≈22 ms 余量）。

---

## 7. 验证

### 7.1 `aim2 doctor`

```bash
aim2 doctor            # 或：aim2 doctor --port /dev/cu.usbmodem1301
```

`SerialLink.status()` 提供的字段应满足：

| 字段 | 期望 |
| --- | --- |
| `port` | `/dev/cu.usbmodem*`（自动探测顺序 `usbmodem` → `wchusbserial` → `ttyUSB/ttyACM`） |
| `frames` / `errors` | `frames` 持续增长；`errors` 应恒为 0（连续 5 次写失败会触发重连） |
| `crc_errors` | 恒定或极慢增长；持续增长 = 线材/供电问题 |
| `device.uptime_s` | 上电后单调递增（设备没在重启） |
| `device.rx_frames` | 与主机 `frames` 同量级增长（差值是重连窗口内的丢帧） |
| `device.crc_errors` | 设备侧 CRC 错，应长期为 0 |
| `device.queue_drops` | 长期为 0；持续增长 = 累加器饱和（主机一次给太多）或 CRC 风暴 |
| `device.report_hz` | 有位移时接近固件 `REPORT_HZ`（1000），且不超过 USB 端点的实际轮询上限；无位移时为 0 |

> `flags`（deadman / USB 就绪）目前没有进 `SerialLink.status()['device']`，需要主机侧
> 在 `status()` 与 metrics 里补一行输出才能看到。

### 7.2 不依赖 `aim2` 的裸机自检（只用项目自带协议模块）

```python
import sys, time, serial
sys.path.insert(0, "src")
from dma.transport import protocol as P

ser = serial.Serial("/dev/cu.usbmodem1301", 921600, timeout=0.5)
# 1) 读 STATUS：应每 100 ms 一帧，magic/CRC/字段自洽
p = P.StreamParser()
deadline = time.time() + 1.0
while time.time() < deadline:
    for kind, payload, seq in p.feed(ser.read(64)):
        if kind == P.T_STATUS:
            st = P.decode_status(payload)          # 13 字节 "<IHHHHB"
            print(f"uptime={st.uptime_ms}ms rx={st.rx_frames} crc={st.crc_errors} "
                  f"drops={st.queue_drops} hz={st.report_hz} flags={st.flags:#04x}")
# 2) 让光标走 100 px 再回来（先确认没有游戏在跑）
for dx in (50, 50, -50, -50):
    ser.write(P.encode_move(dx, 0, 0)); time.sleep(0.01)
# 3) 一次 30 ms 的点击
ser.write(P.encode_button(1, 0)); time.sleep(0.03); ser.write(P.encode_button(0, 0))
# 4) 静默 0.5 s → 设备应进入 deadman（flags bit0 = 1），且不会有按键残留
```

### 7.3 断线安全（必做）

1. 按住左键的 tap 进行中（`BUTTON 1` 已发、`BUTTON 0` 未发）；
2. `Ctrl-C` 掉 `aim2`（或直接拔掉 Mac 侧 USB）；
3. **300 ms 内左键必须自己弹起**，之后没有任何位移输出。

### 7.4 仓库内可做的静态回归（已做）

`firmware/esp32_hid/esp32_hid.ino` 的协议核心（CRC、重同步、子帧拆分、deadman、
STATUS 字节）已在宿主机上用桩头文件编译并跑过 35 项断言，且与
`src/dma/transport/protocol.py` 产生的真实帧逐字节对拍通过（含 13 字节 STATUS 交给
`decode_status()` 反解校验）；**USB 枚举、HID 报告
周期、真机报告率、deadman 的真实松键效果只能上板验证**（见 §9）。

---

## 8. 常见问题

### HID 掉线 / 设备不枚举

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| Mac 只看到串口、没有鼠标 | USB Mode 选成了 `Hardware CDC and JTAG` | 改 `USB-OTG (TinyUSB)` + `USB CDC On Boot: Enabled`，重新烧录 |
| 完全没有设备 | 插了 `UART` 口（CP2102），不是原生 USB 口 | 换到丝印 `USB` 的口 |
| 完全没有设备 | 充电线 / 坏线 / Hub / KVM | 换线、直连 |
| 枚举后几十秒又消失 | 供电跌落（长线 + 克隆板）；或主机侧 `usbmodem` 被其它程序占用 | 换短线、去掉 Hub；`lsof /dev/cu.usbmodem*` 查占用 |
| macOS 睡眠唤醒后失效 | USB 重枚举，`aim2` 会走 `reconnects` 重连 | 唤醒后确认 `aim2 doctor` 里 `reconnects` 增长且 `device.uptime_s` 归零（设备重启属正常） |

### 报告率不到 1000 Hz

1. **先确认量的是哪一段**：`device.report_hz` 是「最近 100 ms 实际发出的 HID 报告数折算」，
   没有位移时报 0 是正常的；主机侧 `report_hz` 字段是 uint16，1000 不会截断。
2. 主机侧 `output.report_hz`（默认 500）会限制发送帧率；固件最多按 1000 Hz 出报告，
   但没有新位移就不发空报告。
3. USB 中断端点 `bInterval` 决定上限：本固件用 core 3.x 的 `USBHIDMouse`（1 ms 周期）。
   接了 Hub/KVM 后主机可能降级。
4. 串口侧不是瓶颈：921600 bps ≈ 92 kB/s，500~1000 帧/s × 9~17 B ≈ 15 kB/s，余量 6 倍。
5. CPU 频率必须 240 MHz；如果同时开了 WiFi/蓝牙，优先级 20 的报告任务仍可能被
   射频任务的长时间临界区拖慢——**本桥不要开 WiFi/BT**。

### 按键粘滞（最危险的故障）

本固件有三道防线，按顺序排查：

1. **deadman**：300 ms 无合法帧即 `release`。若仍粘住，说明设备仍在收帧（主机没停），
   或 USB 已断（此时 OS 会在设备移除时释放按键，但仍建议 `REPORT_HZ` 保持、`DEADMAN_MS` 不要调大）。
2. **CRC 风暴**：连续 32 次 CRC 错会清累加器，但**不会**自动松键（松键由 deadman 负责）。
   若线材很差、帧时断时续，deadman 可能一直不触发（偶发好帧续命）——这属于线材问题，
   换线，看 `device.crc_errors`。
3. **主机侧 `FireCfg.tap_s = 30 ms`**：若 `aim2` 在发 `BUTTON 1` 后崩溃，设备靠 deadman 兜底。
   若要更保险，把 `DEADMAN_MS` 降到 150 ms（代价：主机卡顿时更容易误判为掉线）。

其他可能被误判为「粘滞」的情况：目标游戏自己吞了 mouse-up（前台失焦）、
KVM 切换时 HID 状态被缓存、以及把 `BUTTON` 帧发成了「只发按下不发抬起」。

### 光标抖动 / 位移发飘

- **不要在别处调用 `Mouse.*`**：全文件只有 `report_task` 调，改代码时务必保持。
- 累加器上限 `AXIS_CAP=508` 若被改小，快速甩枪会被截断（`queue_drops` 会涨）。
- 解析任务与 USB 同核（core 0）是刻意设计；把 `report_task` 也挪到 core 0 会明显变差。
- 主机侧 `planner` 的 `max_per_report` 与固件 `MAX_PER_REPORT` 都是 127，不要只改一边。

### 串口找不到 / `aim2` 报「未找到串口设备」

- `Serial` 落到了 UART0 → 说明 `USB CDC On Boot` 没开（见 §2 矩阵）。
- 端口名不是 `usbmodem`（例如用了 CP2102 口）→ `SerialLink.resolve_port()` 会先匹配
  `/dev/cu.usbmodem*`，插错口时会连到错误的设备。
- macOS 上被别的程序占用（Arduino 串口监视器、`screen`）→ 关掉再试。

### 烧录失败

- `A fatal error occurred: Failed to connect`：按住 BOOT → 点 RST → 松开 BOOT 再上传。
- 端口选成了 UART 口 → 改选原生 USB 口（下载也能走原生 USB）。
- 降 `upload_speed` 到 `115200`。

---

## 9. 只能上板验证的项（宿主机回归覆盖不到）

1. **USB 枚举**：复合设备描述符、HID 报告描述符是否被 macOS/Windows/Linux 接受，
   `bInterval` 实际值。
2. **真实报告率与抖动**：`Mouse.move()` 阻塞语义 + TinyUSB FIFO + 主机轮询共同决定，
   只有用示波器/逻辑分析仪抓 D+/D-，或在被控端用 `raw input` 采样时间戳才能确证。
3. **按键 µs 级响应**：代码路径是「解析任务 notify → 报告任务 apply」，理论 µs 级，
   但实际延迟含 USB 帧调度，需在目标机上看 HID 报告时间戳。
4. **deadman 的真实松键**：需要 OS 侧观察「按住 → 断线 → 300 ms 内自动弹起」。
5. **921600 CDC 吞吐与 `SERIAL_RX_BUFSZ` 边界**：极端突发（1000 帧/s 连续数秒）下是否溢出。
6. **长时间稳定性**：≥ 8 h 连续运行、内存不涨、`crc_errors` 不漂。
7. **与目标游戏/系统的兼容性**：部分游戏对「非物理鼠标」的 raw input 有额外过滤与加速曲线，
   属于主机侧标定问题，但只能在真机上确认。

---

## 10. 文件

| 文件 | 内容 |
| --- | --- |
| `esp32_hid.ino` | 固件全部实现（单文件，无第三方依赖，注释逐段说明） |
| `platformio.ini` | PlatformIO 工程（主 env 可用；`esp32-s3-hwcdc-jtag` 仅规格对照） |
| `README.md` | 本文档 |
