# ESP12F 烧录指南

## 1. 架构概述

板载 ESP12F (ESP8266) 的 UART0 通过 STM32 USART2 与主控通信。烧录时 STM32 充当 USB-to-UART 透传桥：

```
PC (USB) ←→ STM32 USART1 ←→ 透传 ←→ STM32 USART2 ←→ ESP12F UART0
```

STM32 同时控制 ESP12F 的 `EN`、`RST`、`IO0` 引脚，管理启动模式和下载时序。

**约束**：第一版固定波特率 `115200 8N1`，不使用高速上传。确认硬件串口稳定后再考虑动态高速烧录。

**固件源码**：`firmware/esp12f/F407_ESP12F.ino`（Arduino 工程），编译和功能说明见 [`firmware/esp12f/README.md`](../firmware/esp12f/README.md)。

---

## 2. esptool.py 烧录流程

### 2.1 步骤

1. 打开 USART1 串口终端（Putty/TeraTerm/串口助手），输入：

   ```
   espflash on
   ```

   提示 `"esp12f flash bridge on: close this terminal..."`

2. **立即关闭串口终端**，释放 STM32 USART1 对应的 COM 端口。

3. 使用同一 COM 端口执行 esptool.py：

   ```bash
   # 验证连接
   esptool.py --chip esp8266 --port COMx --baud 115200 chip_id

   # 烧录固件
   esptool.py --chip esp8266 --port COMx --baud 115200 write_flash 0x00000 firmware.bin
   ```

4. 烧录完成后等待 **30 秒闲置自动退出** bridge，或复位整板。

5. 重新打开串口终端，验证：

   ```
   espflash status
   status
   ```

   确认 bridge 已 inactive、ESP12F 正常协议恢复。

### 2.2 esptool.py 常用命令

```bash
# 擦除全部 Flash
esptool.py --chip esp8266 --port COMx --baud 115200 erase_flash

# 读取 Flash 内容（前 1MB）
esptool.py --chip esp8266 --port COMx --baud 115200 read_flash 0 0x100000 dump.bin

# 查看 Flash 信息
esptool.py --chip esp8266 --port COMx --baud 115200 flash_id
```

---

## 3. Arduino IDE 烧录流程

适用于使用 Arduino 框架开发的 ESP8266 固件。

### 3.1 配置

1. Arduino IDE 安装 **ESP8266 boards**（`https://arduino.esp8266.com/stable/package_esp8266com_index.json`）
2. 开发板选择 **`Generic ESP8266 Module`**
3. 关键参数设置：

   | 参数 | 推荐值 |
   | --- | --- |
   | Board | Generic ESP8266 Module |
   | Upload Speed | **115200** |
   | Flash Mode | **DOUT**（若模组 Flash 确认支持 DIO/QIO 再切换） |
   | Flash Size | 按模组实际 Flash 容量选择 |
   | Port | STM32 USART1 对应的 COM 口 |
   | Reset Method | **None**（不依赖 DTR/RTS 自动复位） |

### 3.2 步骤

1. 完成上述配置
2. 用串口终端向 USART1 输入 `espflash on`
3. **关闭串口终端**释放 COM 口
4. 回到 Arduino IDE，点击 **Upload**
5. 观察编译和上传进度。上传完成后等待 30 秒自动退出 bridge，或复位整板

---

## 4. 烧录桥行为详解

### 4.1 进入 bridge 时的动作

- 清空当前运动命令（`stop`）
- 停止 cycle CSV 日志（`stream_mode = 0`）
- 关闭 `vel` 调试指令
- **下载模式时序**（`Esp12fFlashBridge_StartDownloadBoot`）：
  1. 同时拉低 `EN` + `IO0` + `RST`，等待 50ms（全芯片掉电复位）
  2. 释放 `EN`，等待 10ms（ESP8266 在 `EN` 上升沿采样 `IO0`）
  3. 释放 `RST`，等待 BootROM 就绪
- 启动 USART1 ↔ USART2 双向透传

### 4.2 bridge active 期间

- USART1 收到的**每个字节**直接转发到 USART2（ESP12F），不做任何解析
- USART2 收到的**每个字节**直接转发到 USART1（PC），不做任何修改
- ESP12F 正常网页控制协议暂停，status 帧不会混入烧录数据
- 调试台命令台暂停（`Esp12fFlashBridge_IsActive()` 检查），避免二进制烧录流被误解析
- 30 秒无任何串口活动 → **自动退出** bridge

### 4.3 自动退出

退出时执行：
- `ESP_IO0 = 1`（正常启动模式）
- ESP12F 复位（`ESP_RST` 脉冲）
- USART2 恢复正常协议收发
- 调试台命令台恢复

---

### 4.4 手动退出

如果需要提前退出 bridge（例如烧录完成后立即恢复调试）：

```
espflash off    # 关闭烧录桥
espat off       # 关闭 AT 透传桥
```

但注意 bridge active 期间调试台不解析文本命令，因此 `espflash off` / `espat off` **只能在 bridge 未 active 时使用**。如需强制退出，**复位整板**。30 秒无串口活动自动退出是最常用的退出方式。

---

## 5. AT 透传模式（连通性测试）

`espat on` 提供与 `espflash on` 相同的 USART1↔USART2 透明桥接，但**不进入下载模式**：IO0 保持高电平，并复位 ESP12F 进入正常运行模式。适用于：

- 焊接后验证 ESP12F 物理连通性（发送 AT 指令看回复）
- 调试 ESP12F 固件的串口输出
- 与已烧录固件的 ESP12F 直接交互

### 5.1 步骤

1. 打开 USART1 串口终端，输入：

   ```
   espat on
   ```

   提示 `"AT passthrough active: IO0=high, USART1<->USART2 bridge open."`

2. 直接在终端中输入 AT 指令（需 ESP12F 已烧录 AT 固件或支持 AT 指令的自定义固件）：

   ```
   AT
   AT+GMR
   AT+CWMODE?
   ```

3. 30 秒无操作自动退出，或复位整板强制退出。

### 5.2 与 `espflash on` 的对比

| | `espflash on` | `espat on` |
|---|---|---|
| IO0 电平 | 低（下载模式） | 高（正常运行） |
| RST 操作 | 脉冲复位 → BootROM | 脉冲复位 → 正常固件 |
| 用途 | esptool.py / Arduino 烧录 | AT 指令测试连通性 |
| ESP12F 需有固件 | 否（BootROM 烧录） | 是（需运行 AT 固件或自定义固件） |
| 退出后 | 正常启动（IO0=1, RST 脉冲） | 恢复正常（IO0=1, RST 脉冲） |

### 5.3 空片测试策略

如果 ESP12F 是空片（未烧录任何固件），建议先使用 `espflash on` 烧录 AT 固件或自定义固件，再用 `espat on` 测试连通性。空片本身不会响应 AT 指令。

---

## 6. 故障排查

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `esptool.py` 报 `Could not open port` | 串口终端或 Arduino Serial Monitor 仍占用 COM 口 | 关闭所有占用该 COM 口的程序后重试 |
| `Timed out waiting for packet header` | ESP12F 未进入下载模式 | 确认已执行 `espflash on`；检查 `ESP_IO0` 电平（下载模式应为低） |
| `Invalid head of packet` / 上传中断 | 波特率设置错误或串口质量差 | 确认 esptool.py 和 Arduino IDE 均使用 **115200**；不要使用 460800 等高速率 |
| `chip_id` 返回 `0x000000` 或超时 | ESP12F 未响应 | 检查 `ESP_EN=1`，`ESP_RST=1`；检查 USART2 接线方向；用万用表确认 ESP12F 供电正常 |
| 烧录成功后 ESP12F 程序不启动 | ESP12F 仍处于下载模式或未复位 | 等待 30 秒自动退出后 ESP12F 自动复位；或复位整板 |
| 查看桥接统计 | 想知道收发字节数 | bridge 退出后输入 `espflash status`：查看 `active/download/idle`、`pc_rx/pc_tx/esp_rx/esp_tx`、`ovf`(pc→esp/esp→pc)、`uart_err`、`rx_start_err`、`auto_exit`、`exit_idle` |
| bridge 中途断开（esptool 报错） | 烧录工具异常退出 | 等待 30 秒自动退出 bridge，或复位整板恢复 |
