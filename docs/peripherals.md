# 外设资源

## 1. 电机 PWM 与 DRV8874 H-Bridge

四路电机采用 DRV8874 H-bridge 驱动，每路两个 PWM 通道（IN1/IN2）控制方向和转速。TIM1/TIM8 各提供 4 个 PWM 通道。

### 1.1 电机引脚分配

| 电机 | 侧别 | IN1 (前进 PWM) | IN2 (后退 PWM) | nFAULT |
| --- | --- | --- | --- | --- |
| M1 | 左侧 | TIM1 CH1 / PE9 | TIM8 CH1 / PC6 | PA2 |
| M2 | 左侧 | TIM1 CH2 / PE11 | TIM8 CH2 / PC7 | PA3 |
| M3 | 右侧 | TIM1 CH3 / PE13 | TIM8 CH3 / PC8 | PD14 |
| M4 | 右侧 | TIM1 CH4 / PE14 | TIM8 CH4 / PC9 | PD15 |

### 1.2 PWM 与保护信号

| 资源 | 引脚 | 关键配置 | 用途 |
| --- | --- | --- | --- |
| TIM1 | PE9/PE11/PE13/PE14 | Period `8399`, Mode `PWM1`, Initial Pulse `0` | 四路 IN1 前进 PWM |
| TIM8 | PC6/PC7/PC8/PC9 | Period `8399`, Mode `PWM1`, Initial Pulse `0` | 四路 IN2 后退 PWM |
| TIM1_BKIN | PE15 | Break 使能，低有效 | TIM1 硬件刹车 |
| TIM8_BKIN | PA6 | Break 使能，低有效 | TIM8 硬件刹车 |
| DRV_SLEEP_ALL | PE7 | 高电平唤醒 | DRV8874 全局 nSLEEP 控制 |

### 1.3 H-Bridge 控制逻辑

| 状态 | IN1 PWM | IN2 PWM | 说明 |
| --- | --- | --- | --- |
| 前进 | `+duty` | `0` | IN1 占空比控制转速，IN2 低电平 |
| 后退 | `0` | `+duty` | IN2 占空比控制转速，IN1 低电平 |
| 滑行 | `0` | `0` | 两路均拉低，电机惯性转动 |
| 刹车 | `100%` | `100%` | 两路均拉高，H 桥短路制动 |

**故障保护**：任一路 `nFAULT` 拉低（低有效）→ 锁存 fault-stop → 清空全部运动命令 → 拉低 DRV_SLEEP_ALL → 停止 PWM 输出。`clearfault` 命令清除软件锁存，但若硬件 `nFAULT` 仍为低则下一轮 safetyTask 重新锁存。

---

## 2. 编码器

四路增量式编码器通过定时器 Encoder 模式计数，支持回绕差分和速度计算。

| 电机 | 定时器 | 通道引脚 | 模式 | 滤波 | Period |
| --- | --- | --- | --- | --- | --- |
| M1 | TIM2 | PA15 CH1, PB3 CH2 | Encoder TI12 | IC filter `8` | `0xFFFFFFFF` |
| M2 | TIM3 | PB4 CH1, PB5 CH2 | Encoder TI12 | IC filter `8` | `65535` |
| M3 | TIM4 | PD12 CH1, PD13 CH2 | Encoder TI12 | IC filter `8` | `65535` |
| M4 | TIM5 | PA0 CH1, PA1 CH2 | Encoder TI12 | IC filter `8` | `0xFFFFFFFF` |

**配置参数**（`App/chassis/chassis_config.h`）：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `CHASSIS_ENCODER_BASE_PPR` | `11.0f` | 编码器基础线数 |
| `CHASSIS_ENCODER_QUADRATURE_MULT` | `4.0f` | 四倍频系数 |
| `CHASSIS_MOTOR_GEAR_RATIO` | `56.0f` | 电机减速比 |
| `CHASSIS_MIN_ENCODER_DT_MS` | `1U` | 速度计算最小间隔 |
| `CHASSIS_MAX_ENCODER_DT_MS` | `100U` | 速度计算最大间隔（超时判零） |

方向符号通过 `CHASSIS_Mx_ENCODER_DIR`（`+1`/`-1`）修正：车辆前进时 `status` 中速度显示应为正值。默认 M1/M2 为 `+1`，M3/M4 为 `-1`。

---

## 3. ADC 采样

ADC1 配置为 12-bit 分辨率、连续扫描模式、软件触发、DMA2 Stream0 循环传输。

### 3.1 通道分配

| Rank | ADC 通道 | 引脚 | 信号 | 采样时间 |
| --- | --- | --- | --- | --- |
| 1 | ADC1_IN10 | PC0 | M1 电流 | 84 cycles |
| 2 | ADC1_IN11 | PC1 | M2 电流 | 84 cycles |
| 3 | ADC1_IN12 | PC2 | M3 电流 | 84 cycles |
| 4 | ADC1_IN13 | PC3 | M4 电流 | 84 cycles |
| 5 | ADC1_IN14 | PC4 | VBAT 电池电压 | 84 cycles |

### 3.2 换算参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `ADC_MONITOR_VREF_V` | `3.3f` | ADC 参考电压 |
| `ADC_MONITOR_RESOLUTION_COUNTS` | `4095.0f` | 12-bit 满量程 |
| `MOTOR_CURRENT_SHUNT_OHM` | `0.1f` | 电流采样电阻 |
| `MOTOR_CURRENT_VOLTS_PER_AMP` | `0.1f` | 电流传感器增益 (100mV/A) |
| `MOTOR_CURRENT_FILTER_ALPHA` | `0.25f` | 电流 EMA 滤波系数 |
| `MOTOR_CURRENT_LIMIT_A` | `0.8f` | 单路过流阈值 |
| `MOTOR_OVERCURRENT_DEBOUNCE_COUNT` | `5U` | 过流去抖计数 |

**电池分压**：`VBAT × R_lower / (R_upper + R_lower)`，默认 `R_upper = 47kΩ`，`R_lower = 10kΩ`，分压比 `5.7`。

---

## 4. 通信接口

四条 UART/USART，用途各异，均配置为 `115200 8N1`。

| 接口 | 引脚 | 功能标签 | RX 模式 | DMA | 用途 |
| --- | --- | --- | --- | --- | --- |
| USART1 | PB6 TX, PB7 RX | DEBUG | 中断逐字节 | 无 | 调试命令台 |
| USART2 | PD5 TX, PD6 RX | ESP12F | 中断逐字节 | 无 | ESP12F 协议 + 烧录透传 |
| USART3 | PD8 TX, PD9 RX | RPI | DMA1 Stream1 Circular | 是 | 上位机 (Raspberry Pi) 帧协议 |
| UART4 | PC10 TX, PC11 RX | LINE | DMA1 Stream2 Circular | 是 | HiWonder 八路巡线传感器 |

USART3 / ESP12F 共用同一上位机帧协议（`App/protocol/upper_protocol.h`）。ESP12F 额外支持 `UPPER_CMD_LINE_CTRL (0x03)` 指令远程启停巡线。

---

## 5. BMI270 IMU

Bosch BMI270 六轴惯性测量单元，SPI2 接口通信。

| 信号 | 引脚 | 配置 |
| --- | --- | --- |
| SPI2 SCK | PB13 | Master, 2-line, 8-bit, CPOL=0/CPHA=0 |
| SPI2 MISO | PB14 | Master 接收 |
| SPI2 MOSI | PB15 | Master 发送 |
| IMU_CS | PB12 | GPIO Output，默认高（片选低有效） |
| IMU_INT1 | PE0 | EXTI0 输入（中断功能预留） |

**驱动特性**：
- 集成 Bosch SensorAPI v2.86.1，加载 8192 字节配置表至 feature engine 内部 RAM
- 100 Hz ODR（加速度计 ±2g，陀螺仪 ±500dps）
- 陀螺零偏校准（`imucal [n]`，含静止验证）
- 互补滤波姿态估计（roll/pitch 融合加速度计，yaw 纯积分）
- EMA 低通滤波（加速度计/陀螺仪 α=0.20）
- SPI 硬件诊断（`imudiag`：寄存器回读 + bit-bang 回退 + MISO 上下拉检测）
- 初始化失败或运行中离线自动重连（1s 间隔重试）

### 5.1 I2C1 预留

| 信号 | 引脚 | 配置 |
| --- | --- | --- |
| I2C1 SCL | PB8 | 100 kHz, 7-bit |
| I2C1 SDA | PB9 | 100 kHz, 7-bit |

当前未挂载 I2C 外设，预留扩展。

---

## 6. ESP12F 控制 GPIO

板载 ESP8266 (ESP12F) 模组，STM32 通过三个 GPIO 控制其启动和下载模式。

| 信号 | 引脚 | 方向 | 默认电平 | 用途 |
| --- | --- | --- | --- | --- |
| ESP_EN | PB10 | Output | 高 | ESP12F 芯片使能 (CH_PD/EN) |
| ESP_RST | PB11 | Output | 高 | ESP12F 复位 (EXT_RSTB) |
| ESP_IO0 | PD7 | Output | 高 | 启动模式：高=正常启动，低=下载模式 |

**启动时序**：
- 正常启动：`ESP_EN=1` → `ESP_RST=1` → `ESP_IO0=1`
- 下载模式：`ESP_EN=1` → `ESP_IO0=0` → `ESP_RST` 脉冲复位

透明烧录桥由 `espflash on` 命令激活，详见 [ESP12F 烧录指南](esp12f-flashing.md)。

---

## 7. PS2 手柄

通过 GPIO bit-bang 方式与 PS2 手柄通信（SPI-like 协议）。

| 信号 | 引脚 | 方向 | 默认电平 | 用途 |
| --- | --- | --- | --- | --- |
| PS2_DO (CMD) | PE3 | Input, Pull-up | — | 命令线（STM32 → 手柄） |
| PS2_DI (DAT) | PE2 | Output | 高 | 数据线（手柄 → STM32） |
| PS2_CS | PE4 | Output | 高 | 片选（低有效） |
| PS2_CLK | PE5 | Output | 高 | 时钟（~250 kHz） |
| TEST_LED | PE6 | Output | — | 状态指示灯 |

**控制语义**：
- 手柄在线时提交 `CONTROL_SOURCE_PS2` 运动命令
- 离线连续 3 次读取失败后 `ClearSource(PS2)`
- 零速命令仍有效（不用旧速度"滑过去"）
- **三角键 (btn2 mask 0x10)** 上升沿触发巡线模式切换

---

## 8. 巡线传感器

HiWonder 八路红外巡线传感器，通过 UART4 与主控通信。

### 8.1 硬件连接

| STM32 | 传感器 |
| --- | --- |
| PC10 (UART4 TX) | 传感器 RX |
| PC11 (UART4 RX) | 传感器 TX |
| GND | GND |

> 注意：传感器由外部供电（通常 3.3V/5V），`LINE_EN` 和 `LINE_SET` 在当前工程中不再使用。

### 8.2 通信协议

**物理层**：UART `115200 8N1`，DMA1 Stream2 循环接收（128 字节缓冲区）。

**协议模式**：手动查询模式。初始化时发送 `0x00` 进入手动模式，后续每周期（5ms / 200 Hz）发送 `0x02` 请求模拟量，传感器返回 21 字节帧：

```
Byte  0:    0x55       帧头 1
Byte  1:    0xAA       帧头 2
Byte  2:    0x02       命令（模拟量）
Byte  3:    0x10       数据长度（16 字节）
Byte  4-19: CH1..CH8   8 通道 × 2 字节（低字节在前）
Byte 20:    CHECKSUM   校验和 = ~(sum of bytes[2..19]) & 0xFF
```

**状态判定**：`analog < LINE_ANALOG_THRESHOLD` 判定为检测到黑线（`state[ch] = 1`），阈值默认 500（12-bit ADC 范围 0–4095）。

**控制命令**（手动模式）：

| 命令 | 字节 | 响应 |
| --- | --- | --- |
| 设置手动模式 | `0x00` | 无 |
| 读取状态位 | `0x01` | 1 字节 bitmask |
| 读取模拟量 | `0x02` | 21 字节帧 |
| 读取阈值 | `0x03` | 21 字节帧 |

### 8.3 驱动架构

| 文件 | 层级 | 职责 |
| --- | --- | --- |
| `BSP/line/line_uart.c` | BSP 驱动 | DMA 循环缓冲管理、二进制帧状态机解析（帧头检测、校验和验证、8 通道模拟量与状态位提取）、传感器初始化与周期查询 |
| `BSP/line/line_control.c` | BSP 控制 | 加权平均线位置计算 → P 控制器 (`angular_z = LINE_KP × error`) → 通过 `ControlManager_SetCommand` 提交至 `CONTROL_SOURCE_LINE` |

### 8.4 控制参数

所有参数集中在 `App/chassis/chassis_config.h`：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `LINE_DEFAULT_ENABLED` | `0U` | 上电后巡线默认关闭 |
| `LINE_SPEED_MPS` | `0.15f` | 巡线前进速度 (m/s) |
| `LINE_KP` | `2.5f` | 角速度比例增益 |
| `LINE_ANGULAR_MAX_RPS` | `2.0f` | 角速度输出钳位 (rad/s) |
| `LINE_SENSOR_TIMEOUT_MS` | `50U` | 传感器数据超时（超时清源） |
| `LINE_DETECT_THRESHOLD_COUNT` | `1U` | 最少检测通道数（低于此值视为丢线） |
| `LINE_ANALOG_THRESHOLD` | `500U` | 黑线判定模拟量阈值（在 `line_uart.h`） |
| `CHASSIS_LINE_PERIOD_MS` | `5U` | 巡线任务周期 |

### 8.5 控制行为

- **默认关闭**：`g_line_enabled = 0U`，上电后巡线不参与仲裁
- **启用方式**：PS2 三角键 / `line on` 命令 / ESP12F `UPPER_CMD_LINE_CTRL (0x03)`
- **跟踪中**：加权平均线位置（CH1=0 … CH8=7）→ 偏差 = 位置 − 3.5（中心）→ `angular_z = LINE_KP × 偏差`
- **丢线处理**：8 路全白或传感器超时 → `ClearSource(CONTROL_SOURCE_LINE)` → 控制权自动回退到下一优先级源
- **手动覆盖**：PS2 摇杆操作时优先级高于巡线，自动盖过

### 8.6 调试命令

参见 [调试命令台 — line 命令](debug-console.md#line-命令)。
