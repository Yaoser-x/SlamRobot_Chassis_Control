# USART1 调试命令台

## 1. 概述

USART1（PB6 TX / PB7 RX），`115200 8N1`。由 `debugTask`（osPriorityBelowNormal, 10ms 周期, 1024W 栈）驱动，通过中断式环形缓冲区接收命令行，`\r` 或 `\n` 为行结束符。

**安全约束**：ESP12F flash bridge active 期间调试台暂停命令解析（避免二进制烧录流被误当作文本命令）。

---

## 2. 全部命令

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `help` / `h` | — | 打印命令列表 |
| `status` / `s` | — | 打印所有子系统单次快照（编码器/底盘/ADC/IMU/系统/巡线） |
| `rtos` | — | FreeRTOS heap、各任务栈余量、missed-period、通信统计 |
| `header` | — | 打印全字段 CSV 日志标题行 |
| **`log 0`** | — | 停止 CSV 日志输出 |
| **`log 1`** | `[field...]` | 启动 2 Hz CSV 日志，可选字段过滤（见第 3 节） |
| **`line`** | — | 打印巡线传感器原始数据与控制状态 |
| **`line on`** | — | 启用巡线控制 |
| **`line off`** | — | 禁用巡线控制 |
| `motor` | `<L> <R>` | 左右侧开环 permille（范围 -900…900） |
| `left` / `right` | `<P>` | 单侧开环快捷命令 |
| `m1`…`m4` | `<F> <R>` | 单路电机 raw IN1/IN2 permille |
| `raw` | `<LF> <LR> <RF> <RR>` | 四路 raw 输入 |
| `vel` | `<mm/s> [mrad/s]` | 闭环速度控制（提交至 `CONTROL_SOURCE_DEBUG`） |
| `stop` | — | 清除所有测试命令、清空 open-loop 和 `vel` 指令 |
| `estop` | `<0\|1>` | 清除/设置紧急停止 |
| `clearfault` | — | 清除锁存过流/DRV 故障标志 |
| `imutest` | — | 探测 BMI270（读取 CHIP_ID 期望 `0x24`） |
| `imudiag` | — | SPI 硬件诊断（寄存器回读 + bit-bang + MISO 上拉/下拉/悬空） |
| `imuinit` | — | 加载配置表并初始化 BMI270 |
| `imucal` | `[n]` | 陀螺零偏校准（默认自动采样数，可选指定） |
| `imucalclear` | — | 清除陀螺校准数据 |
| `imu` | `<0\|1>` | 启用/禁用 IMU 周期采样 |
| `espreset` | — | 复位 ESP12F（`ESP_RST` 脉冲） |
| `espboot` | `<0\|1>` | 设置 ESP12F 正常启动 / 下载模式（`ESP_IO0`） |
| `espflash on` | — | 启用 USART1↔USART2 透明烧录桥（下载模式：IO0=0 + 复位） |
| `espflash off` | — | 关闭烧录桥，ESP12F 正常启动 |
| `espflash status` | — | 查看桥接统计（收发字节/溢出/错误/自动退出次数） |
| **`espat on`** | — | 启用 USART1↔USART2 AT 透传桥（正常模式：IO0=1 + 复位） |
| **`espat off`** | — | 关闭 AT 透传桥，ESP12F 正常启动 |
| **`i2cscan`** | — | 扫描 I2C1 总线（地址 1–127），列出所有 ACK 响应的器件 |

---

## 3. 周期 CSV 日志 (`log`)

### 3.1 基本用法

日志以 500ms 间隔（2 Hz）向 USART1 输出 CSV 行。**全字段模式**（`log 1` 无参数）输出全部 36 列，兼容历史数据解析脚本。**过滤模式**仅输出指定字段，减少串口带宽占用：

```
log 0                          停止日志
log 1                          全字段（36 列），兼容旧行为
log 1 imu                      仅时间戳 + IMU 14 列
log 1 motor imu                时间戳 + motor(8 列) + imu(14 列)，按输入顺序
log 1 motor adc line           时间戳 + motor + adc + line，按输入顺序
```

### 3.2 可用字段

| 字段名 | 输出列 | 列数 | 数据来源 |
| --- | --- | --- | --- |
| `motor` | `m1_mms, m2_mms, m3_mms, m4_mms, m1_pwm, m2_pwm, m3_pwm, m4_pwm` | 8 | `EncoderDriver_GetState` + `ChassisControl_GetState` |
| `adc` | `vbat_mv, m1_ma, m2_ma, m3_ma, m4_ma` | 5 | `AdcMonitor_GetState` |
| `imu` | `imu_online, imu_chip, imu_acc_x/y/z_mg, imu_gyro_corr_x/y/z_mdps, imu_gyro_filt_x/y/z_mdps, imu_roll/pitch/yaw_mdeg` | 14 | `ImuBmi270_GetState` |
| `errors` | `errors` | 1 | `SystemMonitor_GetState` |
| `source` | `source` | 1 | `SystemMonitor_GetState` |
| `ps2` | `ps2_ok, ps2_fail` | 2 | `Ps2Control_GetState` |
| `line` | `line_bytes, line_frames` | 2 | `LineUart_GetState` |
| `esp` | `esp_rx, esp_tx` | 2 | `Esp12fComm_GetState` |

**输出格式**：第一列为 `t_ms`（`osKernelGetTickCount()`），后续按用户输入顺序排列字段列。所有浮点值缩放为毫/微单位整数（×1000），避免 `printf` 浮点开销。

**性能**：过滤模式下惰性获取状态快照——仅获取选中字段对应的子系统数据，其余不调用 `GetState`。日志行通过 `HAL_UART_Transmit` 同步输出（50ms 超时），USART TX 繁忙时阻塞等待。

---

## 4. 巡线调试命令 (`line`)

### 4.1 输出格式

```
> line
LINE enabled=0 active=0 pos=3.50 err=0.00 lx=0.000 az=0.000 det=3
LINE st=00110000 an=234,198,45,67,812,723,1023,998
LINE rx_bytes=14280 frames=680 proto_err=2 ovf=0
```

### 4.2 字段说明

**第一行 — 控制状态**：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `enabled` | `0`/`1` | 全局巡线开关（`g_line_enabled`），通过 `line on`/`line off` 或 PS2 三角键或 ESP12F 控制 |
| `active` | `0`/`1` | 当前是否正在跟踪黑线（有控制指令输出） |
| `pos` | float | 加权平均线位置。0.0 = CH1（最左），7.0 = CH8（最右），中心 3.5 |
| `err` | float | 距中心偏差 = pos − 3.5。正值=线偏右→应右转，负值=线偏左→应左转 |
| `lx` | float | 当前输出的 `linear_x` (m/s)，正常为 `LINE_SPEED_MPS` |
| `az` | float | 当前输出的 `angular_z` (rad/s)，由 P 控制器计算 |
| `det` | uint | 当前检测到黑线的通道数（0–8）。0 表示丢线 |

**第二行 — 传感器原始数据**：

| 字段 | 格式 | 说明 |
| --- | --- | --- |
| `st` | 8 位 `0`/`1` | 8 通道状态位（`state[0]`–`state[7]`）。`1` = 检测到黑线（模拟量 < `LINE_ANALOG_THRESHOLD`） |
| `an` | 8 个 uint | 8 通道原始模拟量 ADC 值（0–4095） |

**第三行 — UART 收帧统计**：

| 字段 | 说明 |
| --- | --- |
| `rx_bytes` | UART4 DMA 累计接收字节数（含所有原始字节） |
| `frames` | 成功解析的有效协议帧数（`0x55 0xAA` 帧头 + 校验和通过） |
| `proto_err` | 协议错误数（校验和失败或非法帧长） |
| `ovf` | DMA 循环缓冲溢出次数 |

### 4.3 子命令

| 命令 | 说明 |
| --- | --- |
| `line` | 打印当前传感器数据与控制状态（单次快照） |
| `line on` | 启用巡线控制（设置 `g_line_enabled = 1`） |
| `line off` | 禁用巡线控制（设置 `g_line_enabled = 0`，并 `ClearSource(LINE)`） |

---

## 5. 其他命令速览

### 5.1 状态查询

- **`status` / `s`**：单次输出所有子系统快照（5 行），含编码器速度与计数、底盘目标/实际速度与 PWM、ADC 电压电流、IMU 状态与欧拉角、系统错误标志与控制源、通信统计
- **`rtos`**：FreeRTOS 运行时状态，除 heap/栈外还包括 USART3 TX drop 和 ESP12F TX drop 计数
- **`header`**：打印全字段 CSV 标题行（调试用，正常由 `log 1` 自动输出）

### 5.2 电机调试

- 所有 motor/raw 命令受 ESTOP 和 fault-stop 保护，激活时拒绝执行并提示 `"rejected: estop/fault active"`
- permille 参数自动钳位至 `±CHASSIS_PWM_MAX_PERMILLE`（默认 900‰）
- `vel` 通过 `ControlManager` 提交至 `CONTROL_SOURCE_DEBUG`（最低优先级），每 10ms 自动刷新时间戳避免超时

### 5.3 IMU 操作

- `imutest` 执行单次 SPI 读取 chip-id，不修改 IMU 运行状态
- `imucal` 要求保持静止，校准成功后自动写入零偏估计值，`imucalclear` 清除
- `imu 0` 不会关闭 SPI 外设，仅停止 `imuTask` 周期采样

### 5.4 ESP12F 管理

- `espreset` 通过拉低 `ESP_RST` 复位 ESP12F
- `espboot 0` 设置下载模式（`ESP_IO0=0` + 复位），`espboot 1` 恢复正常
- `espflash on` 进入烧录桥（下载模式：IO0=0, RST 脉冲），用于 esptool.py 或 Arduino 烧录
- `espat on` 进入 AT 透传桥（正常模式：IO0=1, RST 脉冲），用于手动发 AT 指令测试连通性
- `espat on` / `espflash on` 操作均不可逆——需要等待 30s 自动退出或复位整板才能恢复调试台
- `espflash off` / `espat off` 用于桥未激活时手动恢复（或复位整板强制退出）

### 5.5 I2C 扫描 (`i2cscan`)

扫描 I2C1 总线地址 1–127，通过 `HAL_I2C_IsDeviceReady` 逐一探测，列出所有 ACK 响应的器件 7-bit 地址。

**输出示例**：

```
I2C1 scan:
  0x3C (7-bit)  ACK
```

- 7-bit 地址 `0x3C` 对应 SSD1306 OLED（HAL 左移 1 位后 `0x78`）
- 无器件响应时输出 `"no device found"`
- 用于验证 I2C 总线连接和器件地址，排查 OLED 不显示问题
