
# 外设资源

## 1. 电机 PWM 与 DRV8874 H-Bridge

四路电机采用 DRV8874 H-bridge 驱动，PMODE 配置为 PH/EN。TIM1 的 IN1 作为 EN/PWM 控速，PC6-PC9 作为 GPIO 输出 PH/IN2 方向电平。

### 1.1 电机引脚分配

| 电机 | 侧别 | EN/IN1 (PWM) | PH/IN2 (DIR) | nFAULT |
| --- | --- | --- | --- | --- |
| M1 | 左侧 | TIM1 CH1 / PE9 | GPIO / PC6 | PA2 |
| M2 | 左侧 | TIM1 CH2 / PE11 | GPIO / PC7 | PD14 |
| M3 | 右侧 | TIM1 CH3 / PE13 | GPIO / PC8 | PA3 |
| M4 | 右侧 | TIM1 CH4 / PE14 | GPIO / PC9 | PD15 |

> 注意：CubeMX 生成文件中的 M2/M3 GPIO label 保留旧命名；运行时在 BSP 映射层修正 M2/M3 的 nFAULT、编码器和电流归属，不改变引脚复用。

### 1.2 PWM 与保护信号

| 资源 | 引脚 | 关键配置 | 用途 |
| --- | --- | --- | --- |
| TIM1 | PE9/PE11/PE13/PE14 | Period `8399`, Mode `PWM1`, Initial Pulse `0` | 四路 EN/IN1 速度 PWM |
| GPIOC | PC6/PC7/PC8/PC9 | Push-pull output, default low | 四路 PH/IN2 方向电平 |
| TIM1_BKIN | PE15 | Break 使能，低有效 | TIM1 硬件刹车 |
| TIM8_BKIN | PA6 | Break 使能，低有效 | TIM8 Break 诊断输入；不再控制 PH GPIO |
| DRV_SLEEP_ALL | PE7 | 高电平唤醒 | DRV8874 全局 nSLEEP 控制 |

TIM1/TIM8 均启用 `AutomaticOutput`。TIM1_BKIN 拉低时硬件立即清除 TIM1 MOE，切断 EN/PWM；TIM8_BKIN 仍可在 `status` 的 `BREAK` 行诊断 BIF/MOE，但 PH/IN2 已由 GPIO 软件写入，不再受 TIM8 输出通道控制。

### 1.3 H-Bridge 控制逻辑

| 状态 | EN/IN1 PWM | PH/IN2 电平 | 说明 |
| --- | --- | --- | --- |
| 前进 | `+duty` | GPIO 高 | EN 占空比控制转速，PH 高电平 |
| 后退 | `+duty` | GPIO 低 | EN 占空比控制转速，PH 低电平 |
| 停止 | `0` | GPIO 低 | DRV8874 PH/EN 的低侧慢衰减制动；普通停机不表示高阻滑行 |

**故障保护**：任一路 `nFAULT` 拉低（低有效）→ 锁存 fault-stop → 清空全部运动命令 → EN=0 停止输出并进入低侧慢衰减制动。`clearfault` 命令清除软件锁存，但若硬件 `nFAULT` 仍为低则下一轮 safetyTask 重新锁存。

---

## 2. 编码器

四路增量式编码器通过定时器 Encoder 模式计数，支持回绕差分和速度计算。

| 电机 | 定时器 | 通道引脚 | 模式 | 滤波 | Period |
| --- | --- | --- | --- | --- | --- |
| M1 | TIM2 | PA15 CH1, PB3 CH2 | Encoder TI12 | IC filter `15` | `0xFFFFFFFF` |
| M2 | TIM4 | PD12 CH1, PD13 CH2 | Encoder TI12 | IC filter `15` | `65535` |
| M3 | TIM3 | PB4 CH1, PB5 CH2 | Encoder TI12 | IC filter `15` | `65535` |
| M4 | TIM5 | PA0 CH1, PA1 CH2 | Encoder TI12 | IC filter `15` | `0xFFFFFFFF` |

**配置参数**（`Domain/config/control_config.h` 与 `BSP/bsp_config.h`）：

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

ADC1 配置为 12-bit 分辨率、5 通道扫描、TIM8 TRGO 上升沿触发、DMA2 Stream0 循环传输。TIM8 作为 ADC 触发定时器运行，计数周期为 20 kHz 等效节拍（168MHz / 8400），通过重复计数器（RepetitionCounter=9）每 10 个更新周期产生一次 TRGO，使 ADC 采样频率固定为 2 kHz，避免连续转换产生中断风暴。

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
| `ADC_MONITOR_BATTERY_FILTER_ALPHA` | `0.10f` | 电池电压 EMA 滤波系数 |
| `ADC_MONITOR_CURRENT_ZERO_SAMPLES` | `256U` | 上电电流零点采样次数 |
| `MOTOR_CURRENT_VOLTS_PER_AMP` | `1.0f` | 电流传感器实测初始标定 (1.0V/A) |
| `MOTOR_CURRENT_VOLTS_PER_AMP_M1..M4` | 继承全局值 | 每路电流比例标定，`adccal plan` 给出建议值 |
| `MOTOR_CURRENT_FILTER_ALPHA` | `0.25f` | 电流窗口 trimmed 值的 EMA 滤波系数 |
| `MOTOR_CURRENT_LIMIT_A` | `0.0f` | 实时 PWM 电流节流阈值；0 表示关闭 |
| `MOTOR_CURRENT_GUARD_OBSERVE_ONLY` | `1U` | 电流保护观察模式；默认只统计不改 PWM |
| `MOTOR_CURRENT_SOFT_LIMIT_ENABLED` | `0U` | 电流软限流开关；默认关闭 |
| `MOTOR_ADC_OVERCURRENT_FAULT_ENABLED` | `0U` | ADC 电流软件过流锁停开关；默认关闭 |
| `MOTOR_OVERCURRENT_DEBOUNCE_COUNT` | `5U` | 过流去抖计数 |
| `ADC_MONITOR_CURRENT_ZERO_MAX_SPAN_RAW` | `20U` | 零点学习窗口允许的最大 raw 跨度 |

**电池分压**：`VBAT × R_lower / (R_upper + R_lower)`，默认 `R_upper = 47kΩ`，`R_lower = 10kΩ`，分压比 `5.7`。

**电流零点**：上电后 ADC monitor 在静止阶段累计 `ADC_MONITOR_CURRENT_ZERO_SAMPLES` 次 ADC 样本，分别生成 M1-M4 的 raw 零点；`status` 中 `cal=n/256` 表示零点采样进度，完成后 `valid=1`。`adccal zero` 可在所有有效电机输出为 0 时重新学习零点，若零点窗口跨度超过 `ADC_MONITOR_CURRENT_ZERO_MAX_SPAN_RAW`，`current_control_valid` 会保持无效，避免错误零点进入保护/控制。

**电流窗口**：ADC1 由 TIM8 TRGO 约 2kHz 触发，`AdcMonitor_Update()` 每 20ms 汇总窗口统计。`status` 的 `ADCWIN` 行和 `log 1 adcraw` 可查看每路 `mean/rms/peak/n`；`ADCQ` 行补充 `signed_mean/noise/zero_span/quality_flags`，用于区分真实反向漂移、噪声和零点异常。默认 `adc` 字段继续输出慢速稳定电流，保持旧 CSV 兼容。

**电流介入**：电流链路分为 `current_valid`（可显示）和 `current_control_valid`（可用于保护/控制）。默认 `MOTOR_CURRENT_GUARD_OBSERVE_ONLY=1U`、`MOTOR_CURRENT_SOFT_LIMIT_ENABLED=0U`、`MOTOR_ADC_OVERCURRENT_FAULT_ENABLED=0U`，即只记录 over-limit 和 would-latch 计数，不改变 PWM，也不触发软件 fault-stop。只有 `current_control_valid=1` 且显式打开对应配置时，软限流或 ADC 软件过流锁停才会介入。

---

## 4. 通信接口

四条 UART/USART，用途各异，均配置为 `115200 8N1`。

| 接口 | 引脚 | 功能标签 | RX 模式 | DMA | 用途 |
| --- | --- | --- | --- | --- | --- |
| USART1 | PB6 TX, PB7 RX | DEBUG | 中断逐字节 | 无 | 调试命令台 |
| USART2 | PD5 TX, PD6 RX | ESP12F | 中断逐字节 | 无 | ESP12F 协议 + 烧录透传 |
| USART3 | PD8 TX, PD9 RX | RPI | DMA1 Stream1 Circular | 是 | 上位机 (Raspberry Pi) 帧协议 |
| UART4 | PC10 TX, PC11 RX | LINE | DMA1 Stream2 Circular | 是 | HiWonder 八路巡线传感器 |

USART3 / ESP12F 共用同一上位机帧协议（`Domain/protocol/upper_protocol.h`）。两者均支持 `UPPER_CMD_LINE_CTRL (0x03)` 指令远程启停巡线。

---

## 5. BMI270 IMU

Bosch BMI270 六轴惯性测量单元，SPI2 接口通信。

| 信号 | 引脚 | 配置 |
| --- | --- | --- |
| SPI2 SCK | PB13 | Master, 2-line, 8-bit, CPOL=0/CPHA=0 |
| SPI2 MISO | PB14 | Master 接收 |
| SPI2 MOSI | PB15 | Master 发送 |
| IMU_CS | PB12 | GPIO Output，默认高（片选低有效） |
| IMU_INT1 | PE0 | EXTI0 输入（BMI270 DRDY/FIFO watermark 唤醒 `imuTask`） |

**驱动特性**：
- 集成 Bosch SensorAPI v2.86.1，加载 8192 字节配置表至 feature engine 内部 RAM
- `normal / performance / debug` 三套 profile 统一管理 ODR、量程、FIFO、DRDY、APS，并在配置后读回校验
- 100 Hz ODR（加速度计 ±2g，陀螺仪 ±500dps），FIFO header + watermark + SensorTime 数据链路
- 陀螺零偏校准（`imucal [n]`，含静止验证）
- Mahony 四元数融合输出 quaternion + roll/pitch/yaw；异常加速度模长自动降低校正权重
- 传感器坐标、车体坐标、ROS REP-103 坐标分层处理；上位机 IMU 输出统一为车体坐标
- EMA 低通滤波（加速度计/陀螺仪 α=0.20）
- SPI 硬件诊断（`imudiag`：寄存器回读 + bit-bang 回退 + MISO 上下拉检测）
- 初始化状态机显式跟踪 probe/config/profile/sampling/retry；SPI、FIFO、时间戳、饱和、加速度和姿态质量状态均有 flags/counters

### 5.1 I2C1 — SSD1306 OLED

| 信号 | 引脚 | 配置 |
| --- | --- | --- |
| I2C1 SCL | PB8 | 100 kHz, 7-bit |
| I2C1 SDA | PB9 | 100 kHz, 7-bit |

I2C1 总线上挂载 SSD1306 128×64 单色 OLED 显示屏（7-bit 地址 `0x3C`，HAL 左移后 `0x78`）。详见 [第 9 节 OLED 显示屏](#9-oled-ssd1306-显示屏)。

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
| PS2_DO (CMD) | PE3 | Output | 高 | 命令线（STM32 → 手柄） |
| PS2_DI (DAT) | PE2 | Input, Pull-up | — | 数据线（手柄 → STM32） |
| PS2_CS | PE4 | Output | 高 | 片选（低有效） |
| PS2_CLK | PE5 | Output | 高 | 时钟（~33 kHz，半周期 10μs） |
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
| `Service/control/line_control_service.c` | Service 控制 | 加权平均线位置计算 → PD 控制器 → 通过 `ControlService_SetCommand` 提交至 `CONTROL_SOURCE_LINE` |

### 8.4 控制参数

算法默认值位于 `Domain/config/control_config.h`，运行时值由 `ParamService` 发布：

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

参见 [调试命令台 — line 命令](debug-console.md#4-巡线调试命令-line)。

---

## 9. OLED SSD1306 显示屏

SSD1306 128×64 单色 OLED，通过 I2C1 接口驱动，由 `oledTask`（osPriorityLow, 100ms 周期）刷新。提供三阶段 UI：欢迎屏 → 系统自检 → 运行状态。

### 9.1 硬件连接

| 信号 | 引脚 | 说明 |
| --- | --- | --- |
| I2C1 SCL | PB8 | 100 kHz 时钟 |
| I2C1 SDA | PB9 | 数据线 |
| VCC | 3.3V | 模组供电 |
| GND | GND | 共地 |

7-bit 器件地址 `0x3C`，HAL 库左移 1 位后为 `0x78`。

### 9.2 驱动架构

| 文件 | 层级 | 职责 |
| --- | --- | --- |
| `BSP/oled/ssd1306.c` | BSP 驱动 | I2C 命令/数据写入、framebuffer 管理（128×8 pages）、像素/字符/矩形/进度条绘制、整屏刷新（page-mode 批量写入，8 次 I2C 事务） |
| `App/display/oled_ui.c` | App 显示 | 三阶段 UI 状态机、自检执行（I2C/IMU/ADC/Motor/Encoder/UART/ESP12F）、模块在线状态聚合、错误码闪烁 |
| `BSP/oled/oled_font_data.c` | 字体数据 | 8×16 ASCII、12×12/16×16 中文字体位图数据 |

### 9.3 UI 三阶段

| 阶段 | 枚举 | 持续时间 | 显示内容 |
| --- | --- | --- | --- |
| **欢迎屏** | `OLED_PHASE_WELCOME` | 5s (`OLED_WELCOME_DURATION_MS`) | 项目名 "F407 V2.0"、中文副标题 "四轮差速底盘控制"、MCU 型号 "STM32F407VET6"、"系统启动中…" |
| **系统自检** | `OLED_PHASE_SELFCHECK` | 8 项 × 600ms (`OLED_SELFCHECK_ITEM_MS`) | 逐项检测 I2C/IMU/ADC/Motor/Encoder/UART3-RPI/UART4-Line/ESP12F，显示 PASS/FAIL 与进度条 |
| **正常运行** | `OLED_PHASE_NORMAL` | 持续 | 运行时间、电池电压、8 模块在线状态（●/○）、错误码 hex（有错误时 500ms 闪烁）、当前控制源 |

### 9.4 自检项目

| 序号 | 项目 | 检测方式 | 失败 bit |
| --- | --- | --- | --- |
| 1 | I2C | `HAL_I2C_IsDeviceReady` 探测 OLED 地址 | `OLED_SC_ERROR_I2C` (bit 9) |
| 2 | IMU | 读取 BMI270 `chip_id == 0x24` | `OLED_SC_ERROR_IMU` (bit 10) |
| 3 | ADC | 电池电压 > 6.0V | `OLED_SC_ERROR_ADC` (bit 11) |
| 4 | Motor | 通过 `MotorDriver_GetState()` 检查已启用逻辑电机无 nFAULT | `OLED_SC_ERROR_MOTOR` (bit 12) |
| 5 | Encoder | `speed_valid_all` 标志 | `OLED_SC_ERROR_ENCODER` (bit 13) |
| 6 | UART3 RPI | 暂不检测（直接通过） | `OLED_SC_ERROR_UART3_RPI` (bit 14) |
| 7 | UART4 Line | 暂不检测（直接通过） | `OLED_SC_ERROR_UART4_LINE` (bit 15) |
| 8 | ESP12F | 暂不检测（直接通过） | `OLED_SC_ERROR_ESP12F` (bit 16) |

> 自检错误 bit 位于 bit 9–16，与 `SafetyService` 的 motor/battery 错误 bit（bit 0–8）不冲突。正常运行阶段错误码 hex 为两者 OR 合并显示。

### 9.5 模块在线检测（正常运行阶段）

每 100ms 刷新周期更新各模块在线状态，用实心圆点 `●`（在线）和空心圆点 `○`（离线）显示：

| 标签 | 模块 | 检测方式 |
| --- | --- | --- |
| RPI | 上位机 | USART3 最近一帧 RX 时间戳 < `OLED_MODULE_TIMEOUT_RPI_MS`（500ms） |
| PS2 | PS2 手柄 | `Ps2Control_GetState().online` |
| IMU | BMI270 | `ImuBmi270_GetState().online` |
| Line | 巡线传感器 | 传感器数据时间戳 < `OLED_MODULE_TIMEOUT_LINE_MS`（50ms） |
| Enc | 编码器 | `EncoderDriver_GetState().speed_valid_all` |
| ESP | ESP12F | `Esp12fService_GetState().rx_frames > 0` |
| Motr | 电机驱动 | `SafetyService_GetState().error_flags` 无 `DRV_FAULT` |
| ADC | 模数采样 | `AdcMonitor_GetState().current_valid`；保护/控制另看 `current_control_valid` |

### 9.6 配置参数

所有参数集中在 `Domain/config/control_config.h / BSP/bsp_config.h`：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `OLED_I2C_ADDR` | `0x3CU` | SSD1306 7-bit I2C 地址 |
| `OLED_TASK_PERIOD_MS` | `100U` | OLED 刷新任务周期 |
| `OLED_WELCOME_DURATION_MS` | `5000U` | 欢迎屏持续时间 |
| `OLED_SELFCHECK_ITEM_MS` | `600U` | 每项自检耗时 |
| `OLED_SELFCHECK_TOTAL_ITEMS` | `8U` | 自检项目总数 |
| `OLED_ERROR_BLINK_PERIOD_MS` | `500U` | 错误码闪烁周期 |
| `OLED_MODULE_TIMEOUT_RPI_MS` | `500U` | RPI 模块超时判定 |
| `OLED_MODULE_TIMEOUT_LINE_MS` | `50U` | 巡线模块超时判定 |

### 9.7 I2C 扫描命令

调试台 `i2cscan` 命令可扫描 I2C1 总线（地址 1–127），列出所有 ACK 响应的器件地址。参见 [调试命令台 — i2cscan](debug-console.md#56-i2c-扫描-i2cscan)。

---

## 10. Reset Trace 崩溃追踪

跨复位保留的崩溃诊断系统，记录故障类型、堆栈帧、任务心跳、控制状态和关键外设寄存器。

### 10.1 存储机制

`reset_trace_record_t` v4 结构体（156 字节）存放在链接脚本定义的 `.noinit` RAM 段（`STM32F407XX_FLASH.ld`），该段在启动时不清零，可跨软件复位、看门狗复位和故障复位保留；断电后 RAM 内容不保证保留。记录通过 XOR-shift checksum 自校验。

| 文件 | 职责 |
| --- | --- |
| `Platform/platform_reset_trace.c` | 核心逻辑：心跳更新、控制状态跟踪、崩溃捕获、校验和计算 |
| `Platform/platform_reset_trace.h` | 类型定义和 API 声明 |
| `STM32F407XX_FLASH.ld` | `.noinit` 段定义 |

### 10.2 崩溃捕获点

| 捕获点 | 记录内容 | 电机禁用 |
| --- | --- | --- |
| `HardFault_Handler` | 裸函数提取 PC/LR/XPSR/SP、MSP/PSP、CONTROL、FPCCR、CFSR/HFSR/BFAR/MMFAR、DMA2 Stream0 和 ADC1 寄存器 | 是 |
| `NMI_Handler` | NMI 标志 | 是 |
| `MemManage_Handler` | CFSR MemManage 位 | 是 |
| `BusFault_Handler` | CFSR BusFault 位、BFAR | 是 |
| `UsageFault_Handler` | CFSR UsageFault 位 | 是 |
| `Error_Handler` | HAL 错误 | 是 |
| FreeRTOS hooks | 任务名→task ID 映射、溢出/分配失败原因 | 是 |
| DMA2_Stream0 guard | DMA 句柄完整性校验失败原因 | 是 |

### 10.3 任务心跳

`safetyTask`、`motorTask`、`ps2Task`、`espTask`、`debugTask` 每个周期调用 `PlatformResetTrace_TaskHeartbeat()` 记录当前 tick。崩溃时通过各任务最后心跳判断哪个任务首先停止。

### 10.4 调试台输出

上电时自动打印 `RESET` 和 `RESETTRACE` 行；`status` 命令也包含这些信息。详见 [调试命令台 — 复位诊断](debug-console.md#55-复位诊断reset-trace)。

## 11. P0 硬件参数与保护

- 实物轮径 70mm，默认轮半径 `0.035m`；有效轮距 `0.176m`（机械测量 181.5mm）；编码器 2464 count/轮圈。轮半径/轮距可运行时调整，电机/编码器方向支持编译期默认与运行时覆盖（`set motor_dir`/`set encoder_dir`）。
- 3S 电池：有效样本 <10.5V 置 `LOW_BATTERY`，>11.0V 解除；<9.0V 连续 500ms 锁存 `BATTERY_CRITICAL` bit18；>9.6V 连续 2s 仅自动清该位。
- 用户确认的板级连线：四路 nFAULT 经两组 BAT54A 汇聚、47k 上拉，同时连到 PE15/TIM1_BKIN 与 PA6/TIM8_BKIN。仓库不含原理图/KiCad，该连线是用户确认的板级事实，不是从仓库原理图复核得出。
- TIM1 是权威硬切断与软件锁存；TIM8 只是同网冗余诊断。`SYSTEM_ERROR_TIM_BREAK` 不把两路 BIF 计为两个独立故障。
