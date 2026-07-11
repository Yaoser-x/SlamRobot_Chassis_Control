# Lab 13: ESP12F WiFi 帧协议通信与远程控制

## 实验目标

1. **知识**：理解 STM32↔ESP8266 的 UART 帧协议结构、upper_protocol 二进制帧格式、烧录桥原理
2. **技能**：能通过 `espflash` 命令建立 STM32→ESP8266 烧录通道，用 `esptool.py` 烧录 ESP8266 固件
3. **素养**：理解嵌入式系统中"协议通信"与"透明桥接"两种模式的设计——正常通信用帧协议，固件更新用透传桥

## 实验原理

### 1. ESP12F 通信架构

```
正常模式:
  手机网页 ──WiFi──→ ESP12F ──upper_protocol帧──→ STM32F407
                     (USART2)                      │
                                                   ↓
                                              CONTROL_SOURCE_ESP12F

烧录模式 (espflash on):
  PC (esptool.py) ──bin──→ USART1 ──透明桥──→ USART2 ──→ ESP12F BootROM
```

**正常模式**：ESP12F 作为 WiFi 转 UART 模块，将手机/网页的控制指令通过 `upper_protocol` 帧协议发送给 STM32（`App/protocol/esp12f_comm.c` 解析），提交至 `CONTROL_SOURCE_ESP12F`。这不是桥接，而是标准的帧协议通信。

**烧录模式** (`espflash on`)：USART1 和 USART2 之间建立透明桥——STM32 将 PC 发来的二进制流直传给 ESP12F，同时将 ESP12F 的回复直传给 PC。此模式下调试台暂停命令解析（避免二进制流被误当作文本命令）。**桥接仅用于烧录固件，正常通信不使用桥接。**

### 2. 透明烧录桥

`BSP/esp12f/esp12f_flash_bridge.c` 实现双向 1:1 透传：

```
PC (esptool.py) ──bin──→ USART1 ──→ USART2 ──→ ESP12F
PC (esptool.py) ←──bin── USART1 ←── USART2 ←── ESP12F
```

关键设计点：
- 烧录桥激活后，`debugTask` 跳过命令解析（`Esp12fFlashBridge_IsActive()` 检查）
- 双向各维护环形缓冲，处理 USART1/USART2 速率不匹配
- 溢出计数器（`pc_to_esp_overflow`、`esp_to_pc_overflow`）可诊断缓冲是否过小
- 空闲超时自动退出（无数据传输超过一定时间后自动关闭桥接）

### 3. ESP12F 控制协议

正常模式下 ESP12F 发送的 `SET_VELOCITY` 帧复用 `upper_protocol` 格式（`App/protocol/upper_protocol.c`），与上位机 USART3 协议帧完全兼容：

| 帧类型 | 方向 | 作用 |
|--------|------|------|
| SET_VELOCITY | ESP→STM | 设置目标速度 |
| UPPER_CMD_ESTOP | ESP→STM | 紧急停止 |
| UPPER_CMD_LINE_CTRL | ESP→STM | 远程启停巡线 |
| STATUS | STM→ESP | 状态回传 |

Web 控制安全状态机：

- 无有效 EEPROM magic/version/length/CRC 记录时，只启动开放 `F407_Chassis_Setup`，接受 8–63 字节密码；配置前不启动 WebSocket。
- 首个 `claim` 的客户端成为 owner；500ms 租约由 200ms `heartbeat` 续租，其他客户端只读遥测。
- `vel`/`stop`/`line` 仅 owner 可用。owner 断开或超时立即发零速并释放控制权。
- 任意客户端可设置 ESTOP，但 ESP 网页/固件和 STM32 远程协议都不接受 ESTOP=0。

### 4. ESP8266 下载模式

ESP8266 进入下载模式需 GPIO0 拉低 + 复位脉冲：

```
espboot 1           → 拉低 ESP_IO0 (下载模式)
espreset            → 脉冲 ESP_RST (触发复位)
→ ESP8266 进入 UART BootROM，等待 esptool.py 连接
espflash on         → 启用透明桥
→ PC 端 esptool.py 可以烧录
```

烧录完成后：
```
espflash off        → 关闭透明桥
espboot 0           → ESP_IO0 拉高 (正常启动)
espreset            → 复位 → ESP8266 正常启动
```

相关 GPIO：

| 功能 | STM32 引脚 | 说明 |
|------|-----------|------|
| ESP_RST | PB11 | ESP8266 复位引脚 |
| ESP_IO0 | PD7 | 下载模式选择（0=下载，1=正常） |
| USART2 TX/RX | PD5/PD6 | STM↔ESP 通信 |

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘、12V 电池、USB-TTL 串口模块、ESP12F 模块（已焊接） |
| 软件 | 串口终端 + Python 3 + esptool.py；Arduino IDE 验收使用 ESP8266 Core 3.1.2 + arduinoWebSockets 2.7.2 |

## 实验步骤

### 步骤 1：ESP12F 状态确认

```bash
s
# SYS 行: esp=0/0 表示 rx_frames=0, tx_frames=0
# 若 ESP12F 已烧录网页固件并发送数据: esp=1523/89
```

若 ESP12F 未烧录固件或未启动，`esp` 计数器为 0（正常）。

### 步骤 2：ESP12F 复位与模式切换

```bash
espboot 0           # 确保正常启动模式（IO0 高电平）
espreset            # 复位 ESP8266 → "esp12f reset"
# 等待 2 秒让 ESP8266 启动
s                   # 检查 esp 计数器变化
```

### 步骤 3：进入下载模式

```bash
espboot 1           # 设置 IO0 低电平（下载模式）
espreset            # 复位进入 BootROM
espflash status     # 查看桥接状态（此时应 inactive）
```

`espflash status` 输出示例：
```
ESPFLASH active=0 download=1 idle=0ms pc_rx=0 pc_tx=0 esp_rx=0 esp_tx=0 ovf=0/0 uart_err=0 auto_exit=0
              │           │
              │           └─ download_mode=1 (IO0 低电平)
              └─ active=0 (桥接尚未启用)
```

### 步骤 4：启用透明桥并烧录 ESP8266

```bash
espflash on         # 启用透明桥
# 固件输出: "esp12f flash bridge on: close this terminal and use esptool/Arduino at 115200"
```

**立即关闭串口终端**（不断开的话终端输入的字符也会被送到 ESP），然后在 PC 端执行：

```bash
# 确认 ESP8266 BootROM 可通信
esptool.py --chip esp8266 --port COMx --baud 115200 chip_id

# 烧录固件（以 NodeMCU 或自定义固件为例）
esptool.py --chip esp8266 --port COMx --baud 115200 write_flash 0x00000 firmware.bin
```

**注意**：`COMx` 是 STM32 USART1 在 PC 上映射的串口号（不是 ESP8266 直连）。STM32 在此充当 USB-TTL 桥。

烧录完成后重新打开串口终端：

```bash
espflash off        # 关闭透明桥
espboot 0           # 恢复正常启动模式
espreset            # 复位 → ESP8266 从 Flash 启动
```

### 步骤 5：验证 ESP12F 数据收发

如果 ESP12F 已烧录网页控制固件：

```bash
s
# 观察 SYS 行 esp 计数器
# esp=1523/89 表示收到 1523 帧，发送 89 帧
```

通过 WiFi 连接到 ESP12F 的 AP，打开网页控制界面，发送运动命令：

```bash
s
# 若网页发送 SET_VELOCITY: source=3 (ESP12F)
```

**验证优先级**：ESP12F (优先级 3) > DEBUG (4)，所以网页命令会覆盖串口 `vel` 命令。

追加安全验收：

1. 擦除 EEPROM，确认只有 `F407_Chassis_Setup`；分别提交 7/8/63/64 字节密码，只有 8–63 字节成功。
2. 同时连接两个浏览器：owner 可运动，readonly 继续收遥测但运动/巡线被拒绝。
3. 关闭 owner 页面或停 heartbeat，500ms 内 `source` 应回 NONE 且 PWM 归零。
4. readonly 可触发 ESTOP；急停后页面只显示“请本地解除”，必须在 USART1 执行 `estop 0`。

### 步骤 6：桥接溢流诊断

长时间烧录时检查溢流：

```bash
espflash status
# 若 ovf 计数器 > 0，表示缓冲区不足
# pc_to_esp_overflow > 0: PC→ESP 速率过高
# esp_to_pc_overflow > 0: ESP→PC 数据来不及发送
```

## 数据分析

### 通信帧率分析

```bash
log 1 esp           # CSV 仅 esp 字段
# 保持 30 秒记录正常通信帧率
log 0
```

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('lab13_esp.csv')
t = df['t_ms'] / 1000.0

fig, ax = plt.subplots(figsize=(10, 4))
ax.plot(t, df['esp_rx'].diff().fillna(0) / 0.5, 'b-', alpha=0.7, label='RX frame rate (Hz)')
ax.plot(t, df['esp_tx'].diff().fillna(0) / 0.5, 'r-', alpha=0.7, label='TX frame rate (Hz)')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Frame Rate (Hz)')
ax.legend()
ax.grid(alpha=0.3)
plt.title('ESP12F Communication Frame Rate')
plt.savefig('lab13_esp_frame_rate.png', dpi=150)
```

## 思考题

1. 透明烧录桥为何要在激活时停止调试台命令解析？如果不停止，串口终端输入的 `help` 命令的二进制数据被发送给 ESP8266，会发生什么？

2. `espflash status` 的 `auto_exit` 计数器记录了空闲自动退出次数。这个机制的目的是什么？（提示：考虑用户忘记执行 `espflash off` 的场景）

3. ESP12F 作为 `CONTROL_SOURCE_ESP12F (3)`，优先级高于 LINE(5) 和 DEBUG(4)，但低于 PS2(2) 和 UPPER(1)。WiFi 控制可能因网络延迟导致命令抖动。500ms 的超时设计对 WiFi 控制是否足够？

4. 如果没有 STM32 的透明桥，要烧录 ESP8266 需要什么硬件？（提示：USB-TTL 模块 + 手动拉低 GPIO0）。STM32 烧录桥相比直连方案的优势是什么？

5. ESP12F 和上位机 (USART3) 复用相同的 `upper_protocol` 帧格式。这个复用的优缺点各是什么？如果两者需要同时发送不同格式的帧，架构需要如何调整？

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `esptool.py chip_id` 超时 | STM32 桥未启用或 ESP 不在下载模式 | `espflash status` 确认 active=1, download=1 |
| `espflash off` 后 ESP 无响应 | ESP 未正常复位 | `espboot 0` + `espreset` |
| `ovf` 计数器持续增长 | 缓冲区不足或 PC 发送速率过高 | 降低 esptool.py 波特率至 9600 |
| `uart_err` > 0 | USART 帧错误（噪声或波特率不匹配） | 检查接线和接地；确认 115200 8N1 |
| 网页控制延迟大 | ESP12F WiFi 延迟或 STM32 处理队列积压 | 检查 WiFi 信号强度；减少不必要的数据帧 |
