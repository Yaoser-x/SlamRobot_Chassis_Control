# SlamRobot_Chassis_Control

STM32F407 V2.0 底盘控制固件工程。当前仓库基于 STM32CubeMX 生成的 CMake 工程，目标芯片为 `STM32F407VET6`（LQFP100），使用 STM32 HAL/LL、FreeRTOS CMSIS-RTOS2、CMake Presets、Ninja 和 GNU Arm Embedded Toolchain 构建。

> 当前状态：V2.0 已在 CubeMX 外设骨架上接入 V1.0 已验证控制链，落地四轮左右差速底盘。左侧为 `M1+M2`，右侧为 `M3+M4`；电压/电流采样电路与换算参数沿用既有设计；IMU 改为 `BMI270`；无线模块为板载 `ESP12F`；巡线只保留 `UART4 LINE_TX/RX`。

## 工程概览

| 项目 | 当前配置 |
| --- | --- |
| MCU | STM32F407VET6 |
| 封装 | LQFP100 |
| Cube 固件包 | STM32Cube FW_F4 V1.28.3 |
| 构建系统 | CMake 3.22+ / Ninja |
| 工具链 | GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`) |
| RTOS | FreeRTOS Kernel V10.3.1 + CMSIS-RTOS2 |
| 构建预设 | `Debug` / `Release` |
| 链接脚本 | `STM32F407XX_FLASH.ld` |
| CubeMX 配置 | `F407_V2.0.ioc` |
| 控制优先级 | `RPI/USART3 > PS2 > ESP12F > DEBUG` |
| 底盘形式 | 四轮左右差速（可通过 `CHASSIS_Mx_ENABLED`/`CHASSIS_Mx_SIDE` 配置为 2WD 或自定义布局），默认 `M1+M2` 左侧，`M3+M4` 右侧 |
| 电机驱动 | DRV8874 H-bridge PWM，支持 `nSLEEP` 与低有效 `nFAULT` |
| IMU | BMI270，`SPI2 + IMU_CS + IMU_INT1` |
| 巡线 | `UART4 LINE_TX/RX` DMA/ring buffer |
| ESP12F 烧录 | PC/Arduino IDE -> STM32 USART1 -> STM32 USART2 -> ESP12F UART0 透明桥 |

## 目录结构

```text
App/
  chassis/              四轮差速、任务入口、底盘参数与底盘数学辅助
  control/              控制源仲裁、命令超时、ESTOP/fault-stop
  debug/                USART1 调试命令台
  monitor/              电压、电流、编码器、DRV fault 状态聚合
  protocol/             上位机帧协议与 USART3 通信
BSP/
  adc/                  M1-M4 电流 + VBAT DMA 采样换算
  encoder/              TIM2/TIM3/TIM4/TIM5 编码器与回绕差分
  esp12f/               ESP12F 协议、启动控制、USART1 辅助烧录桥
  imu/                  BMI270 SPI 驱动（配置表加载、陀螺校准、姿态估计、SPI 诊断）
  led/                  TEST_LED 状态灯
  line/                 UART4 巡线 ring buffer
  motor/                TIM1/TIM8 DRV8874 PWM/H-bridge 驱动
  pid/                  速度环 PID
  ps2/                  PS2 手柄语义
Core/
  Inc/                  用户头文件、FreeRTOSConfig、HAL 配置
  Src/                  main、外设初始化、FreeRTOS 线程入口、中断处理
Drivers/                CMSIS 与 STM32F4 HAL/LL 驱动
Middlewares/            FreeRTOS 内核与 CMSIS-RTOS2 适配层
cmake/
  gcc-arm-none-eabi.cmake
  stm32cubemx/          CubeMX 生成的 CMake 目标与源文件清单
.github/workflows/
  firmware-build.yml    GitHub Actions 固件构建流水线
CMakeLists.txt          顶层 CMake 工程
CMakePresets.json       Debug / Release 构建预设
F407_V2.0.ioc           STM32CubeMX 工程配置
STM32F407XX_FLASH.ld    Flash 链接脚本
startup_stm32f407xx.s   启动文件
```

## 时钟配置

当前时钟来自 `SystemClock_Config()` 与 `.ioc`：

| 项目 | 频率 / 配置 |
| --- | --- |
| HSI（临时，待 HSE 晶振修复后恢复） | 16 MHz |
| PLLM / PLLN / PLLP / PLLQ | 16 / 336 / 2 / 7 |
| SYSCLK | 168 MHz |
| HCLK / Cortex FCLK | 168 MHz |
| APB1 | 42 MHz |
| APB1 Timer Clock | 84 MHz |
| APB2 | 84 MHz |
| APB2 Timer Clock | 168 MHz |
| HAL timebase | TIM6 / `TIM6_DAC_IRQn` |

## 外设资源

### 电机 PWM、DRV8874 与 Break

| 电机 | 侧别 | IN1 PWM | IN2 PWM | Fault |
| --- | --- | --- | --- | --- |
| M1 | Left | TIM1 CH1 / PE9 | TIM8 CH1 / PC6 | PA2 |
| M2 | Left | TIM1 CH2 / PE11 | TIM8 CH2 / PC7 | PA3 |
| M3 | Right | TIM1 CH3 / PE13 | TIM8 CH3 / PC8 | PD14 |
| M4 | Right | TIM1 CH4 / PE14 | TIM8 CH4 / PC9 | PD15 |

| 资源 | 用途 | 引脚 / 通道 | 关键配置 |
| --- | --- | --- | --- |
| TIM1 | M1-M4 的 IN1 PWM | PE9 CH1, PE11 CH2, PE13 CH3, PE14 CH4 | Period `8399`, PWM1, Pulse `0` |
| TIM8 | M1-M4 的 IN2 PWM | PC6 CH1, PC7 CH2, PC8 CH3, PC9 CH4 | Period `8399`, PWM1, Pulse `0` |
| TIM1_BKIN | TIM1 Break 输入 | PE15 | Break 使能，低有效 |
| TIM8_BKIN | TIM8 Break 输入 | PA6 | Break 使能，低有效 |
| DRV_SLEEP_ALL | DRV8874 全局唤醒 | PE7 | 高电平唤醒 |

应用层已启动 TIM1/TIM8 PWM，并按 DRV8874 H-bridge 方式实现前进、后退、滑行、刹车。任一路 `nFAULT` 低有效触发后会锁存 fault-stop、清空当前运动命令并停止输出。

### 编码器

| 电机 | 定时器 | 引脚 | 配置 |
| --- | --- | --- | --- |
| M1 | TIM2 | PA15 CH1, PB3 CH2 | Encoder TI12, IC filter `8`, Period `0xFFFFFFFF` |
| M2 | TIM3 | PB4 CH1, PB5 CH2 | Encoder TI12, IC filter `8`, Period `65535` |
| M3 | TIM4 | PD12 CH1, PD13 CH2 | Encoder TI12, IC filter `8`, Period `65535` |
| M4 | TIM5 | PA0 CH1, PA1 CH2 | Encoder TI12, IC filter `8`, Period `0xFFFFFFFF` |

应用层已启动四路编码器并独立计算计数、回绕差分和速度。电机启用、左右侧归属、电机方向和编码器方向集中在 `App/chassis/chassis_config.h` 中配置；默认布局仍为 `M1+M2` 左侧、`M3+M4` 右侧，M1/M2 编码器方向为 `+1`，M3/M4 为 `-1`，上板后可按实际验收微调。

### ADC 与电源/电流采样

ADC1 配置为 12-bit、连续扫描、软件触发、DMA continuous requests，使用 DMA2 Stream0 circular 模式。

| Rank | 通道 | 引脚 | 信号 |
| --- | --- | --- | --- |
| 1 | ADC1_IN10 | PC0 | M1_CURRENT |
| 2 | ADC1_IN11 | PC1 | M2_CURRENT |
| 3 | ADC1_IN12 | PC2 | M3_CURRENT |
| 4 | ADC1_IN13 | PC3 | M4_CURRENT |
| 5 | ADC1_IN14 | PC4 | VBAT_SENSE |

采样时间均为 `ADC_SAMPLETIME_84CYCLES`。应用层保持 V2 现有通道顺序 `M1_CURRENT..M4_CURRENT, VBAT_SENSE`，输出四路电流、电池电压、左右侧电流平均值和原始 ADC 值。电池分压、电流采样零点和 volts-per-amp 常量集中在 `App/chassis/chassis_config.h`，沿用已确认不变的采样设计。

### 通信接口

| 接口 | 引脚 | 当前用途标签 | 参数 |
| --- | --- | --- | --- |
| USART1 | PB6 TX, PB7 RX | DEBUG_TX / DEBUG_RX | 115200 8N1 |
| USART2 | PD5 TX, PD6 RX | ESP_TX / ESP_RX | 115200 8N1 |
| USART3 | PD8 TX, PD9 RX | RPI_TX / RPI_RX | 115200 8N1，RX DMA |
| UART4 | PC10 TX, PC11 RX | LINE_TX / LINE_RX | 115200 8N1，RX DMA |

USART3 RX 使用 DMA1 Stream1，UART4 RX 使用 DMA1 Stream2。USART3 复用 V1 上位机帧协议；ESP12F 保留同一帧协议；UART4 巡线当前只接入 DMA/ring buffer、收帧计数和 USART1 调试输出。

### BMI270 与 I2C 预留

| 资源 | 引脚 | 配置 |
| --- | --- | --- |
| SPI2 SCK / MISO / MOSI | PB13 / PB14 / PB15 | Master, 2-line, 8-bit, Mode 0, prescaler 16 |
| IMU_CS | PB12 | GPIO output，默认高 |
| IMU_INT1 | PE0 | EXTI0 输入 |
| I2C1 SCL / SDA | PB8 / PB9 | 100 kHz, 7-bit addressing |

SPI2 在 `.ioc` 中计算速率约为 2.625 Mbit/s。BMI270 驱动已完整落地：集成 Bosch SensorAPI 配置表（8192 bytes，版本 v2.86.1），加载至 feature engine 内部 RAM；支持 100 Hz acc/gyro ODR（±2g / ±500dps）；内置陀螺零偏校准（`imucal`，含静止验证）；互补滤波姿态估计（roll/pitch 融合加速度计，yaw 纯积分）；EMA 低通滤波（acc/gyro α=0.20）；SPI 硬件诊断（`imudiag`，含 bit-bang 回退与 MISO 上拉/下拉/悬空检测）；初始化失败或运行中离线时自动重连（1s 间隔重试）。

### ESP12F 控制 GPIO

| 信号 | 引脚 | 方向 / 默认状态 | 用途 |
| --- | --- | --- | --- |
| ESP_EN | PB10 | Output, 默认高 | 芯片使能 |
| ESP_RST | PB11 | Output, 默认高 | 复位控制 |
| ESP_IO0 | PD7 | Output, 默认高 | 正常启动 / 下载模式选择 |

正常启动保持 `ESP_EN=1`、`ESP_RST=1`、`ESP_IO0=1`；下载模式使用 `ESP_IO0=0` 后复位。USART2 负责 ESP12F 与主控协议通信，USART1 辅助烧录桥会在烧录期间临时接管 USART1 和 USART2。

### PS2 与状态 GPIO

| 信号 | 引脚 | 方向 / 默认状态 | 用途 |
| --- | --- | --- | --- |
| PS2_DO | PE3 | Input, Pull-up | CMD（命令线，STM32 输出到 PS2 手柄） |
| PS2_DI | PE2 | Output, 默认高 | DAT（数据线，PS2 手柄输入到 STM32） |
| PS2_CS | PE4 | Output, 默认高 | |
| PS2_CLK | PE5 | Output, 默认高 | |
| TEST_LED | PE6 | Output | |

PS2 语义沿用 V1：手柄在线时可提交低优先级运动命令，离线达到阈值后清除 PS2 source；零速命令仍视为有效命令，不用旧速度“滑过去”。

### 巡线接口

巡线只保留 `UART4 LINE_TX/RX`。当前工程不再声明或使用 `LINE_EN`、`LINE_SET`，也不在 GPIO 初始化中配置这两个输出。

## 控制与安全语义

- 控制源优先级固定为 `RPI/USART3 > PS2 > ESP12F > DEBUG`。
- 命令超时为 `CHASSIS_CMD_TIMEOUT_MS`，超时后不继续沿用旧运动命令。
- 非法 source、NaN/Inf 速度、失能命令、ESTOP、fault-stop 都采用 reject-and-stop 语义。
- `clearfault` 会清除软件锁存，但如果 DRV8874 `nFAULT` 仍为低电平，下一轮 safety task 会重新锁存。
- 四轮 PID 独立运行，左右目标按差速模型分配；默认 M1/M2 接收左侧目标、M3/M4 接收右侧目标，可通过 `CHASSIS_Mx_ENABLED` 和 `CHASSIS_Mx_SIDE` 改成 2 驱或不同左右布局。

## 教育科研使用边界

- `m1` 到 `m4`、`raw`、`left`、`right`、`motor` 属于 bring-up 调试命令，建议只在架空轮、低占空比、有人看护时使用。
- raw/open-loop 调试保留是为了方便课堂演示、方向验收、示波器测波形和单轮排故；它们仍受 ESTOP、fault-stop、DRV fault 和过流锁存约束。
- `vel` 闭环实验应在电压、电流、编码器方向、DRV fault、四轮 raw 方向都确认后再进行。
- IMU、PS2、ESP12F、UART4 巡线离线不阻塞底盘实验，但会在 `status` 中显示；ADC、编码器、DRV fault 是底盘核心观察量。
- ESP12F flash bridge 是维护功能。bridge active 时 USART1 会被二进制透传接管，普通调试命令和 ESP12F 正常网页控制协议暂停。

## FreeRTOS 框架

工程启用 FreeRTOS Kernel V10.3.1，并通过 CMSIS-RTOS2 接口创建线程。调度器启动流程位于 `Core/Src/main.c`：

```c
osKernelInitialize();
MX_FREERTOS_Init();
osKernelStart();
```

### FreeRTOSConfig 关键参数

| 参数 | 当前值 |
| --- | --- |
| `configUSE_PREEMPTION` | 1 |
| `configTICK_RATE_HZ` | 1000 |
| `configMAX_PRIORITIES` | 56 |
| `configTOTAL_HEAP_SIZE` | 49152 |
| `USE_FreeRTOS_HEAP_4` | enabled |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 |
| `configUSE_MALLOC_FAILED_HOOK` | 1 |
| `configUSE_TIMERS` | 1 |
| `configUSE_MUTEXES` | 1 |
| `configUSE_COUNTING_SEMAPHORES` | 1 |
| `INCLUDE_uxTaskGetStackHighWaterMark` | 1 |
| `INCLUDE_uxTaskGetStackHighWaterMark2` | 1 |
| `configENABLE_FPU` | 1 |

`configASSERT()`、`vApplicationStackOverflowHook()`、`vApplicationMallocFailedHook()` 和任务创建失败检查已接入 fail-safe：一旦发生 RTOS 级异常，会立即拉低 `DRV_SLEEP_ALL` 并停在 fatal loop，避免电机保持旧 PWM。该策略不会自动软复位，便于教学和实验时保留故障现场。

### FreeRTOS 调度说明

控制、安全、通信和传感器任务使用 `osDelayUntil()` 绝对周期调度，减少“执行耗时 + delay”造成的周期漂移。`defaultTask` 只是空闲保活，保留 `osDelay(1)`；`debugTask` 保持低优先级普通轮询，避免调试打印影响安全和电机任务。

USART1 `rtos` 命令会打印：

- `heap_free` / `heap_min`：当前和历史最低 heap 余量。
- `stack_free`：各任务剩余栈空间。
- `missed`：周期任务本轮执行超过目标唤醒点的累计次数。
- `upper_drop` / `esp_drop`：状态帧发送忙导致跳过的次数。

空载时 `motorTask` 和 `safetyTask` 的 `missed` 不应持续增长；如果某个核心任务 `stack_free` 长期低于 128 到 256B，应优先增加该任务栈。

### 当前线程表

`Core/Src/freertos.c` 只在 CubeMX 保留的 `USER CODE` 区域接入业务任务，避免重新生成代码时丢失。

| 线程 | 优先级 | 周期 | 调度 | 栈大小 | 当前职责 |
| --- | --- | --- | --- | --- | --- |
| `defaultTask` | Low | 1 ms | `osDelay` | 128 words | 空闲保活 |
| `safetyTask` | High | 20 ms | `osDelayUntil` | 256 words | 状态聚合、命令超时、fault-stop |
| `motorTask` | AboveNormal | 10 ms | `osDelayUntil` | 512 words | 编码器更新、速度环、PWM 输出 |
| `rpiCommTask` | Normal | 5 ms | `osDelayUntil` | 512 words | USART3 上位机协议 |
| `imuTask` | Normal | 10 ms | `osDelayUntil` | 512 words | BMI270 周期采样（100 Hz） |
| `lineTask` | BelowNormal | 5 ms | `osDelayUntil` | 256 words | UART4 巡线收帧计数 |
| `espTask` | BelowNormal | 5 ms | `osDelayUntil` | 512 words | ESP12F 协议收发与 flash bridge 更新 |
| `debugTask` | BelowNormal | 10 ms | `osDelay` | 1024 words | USART1 命令台 |
| `ps2Task` | Normal | 20 ms | `osDelayUntil` | 512 words | PS2 手柄控制 |
| `ledTask` | Low | 50 ms | `osDelayUntil` | 128 words | TEST_LED 状态指示 |

## USART1 调试命令

```text
help
status
rtos
header
log <0|1>
motor <left_permille> <right_permille>
left <permille>
right <permille>
m1 <forward_permille> <reverse_permille>
m2 <forward_permille> <reverse_permille>
m3 <forward_permille> <reverse_permille>
m4 <forward_permille> <reverse_permille>
raw <left_forward> <left_reverse> <right_forward> <right_reverse>
vel <linear_mps> [angular_rps]
stop
estop
clearfault
imutest
imuinit
imudiag
imucal [n]
imucalclear
imu <0|1>
espreset
espboot <0|1>
espflash on
espflash off
espflash status
```

建议先用 `status`、`rtos`、`m1` 到 `m4`、`left`、`right` 逐步验收，不要一开始直接跑闭环高速。`rtos` 会打印 FreeRTOS heap、各任务剩余栈、missed-period 和通信发送 drop；`espflash on` 进入烧录桥接后，USART1 会变成透明二进制通道，普通调试命令暂时不可用。

## ESP12F 烧录

板载 ESP12F 的 UART0 已接到 STM32 USART2，STM32 又通过 USART1 暴露给 PC。因此可以让 STM32 做透明桥：PC 或 Arduino IDE 只连接 STM32 的 USART1 调试串口，STM32 将 USART1 和 USART2 双向透传，并控制 `ESP_IO0/RST/EN` 让 ESP12F 进入下载模式。

第一版固定 `115200 8N1`，不要使用高速上传。固定低速的目的是先保证可烧录、可恢复、不会因为串口质量或转发节奏导致中途失败。

### esptool.py 流程

1. 打开 USART1 串口终端，输入：

   ```text
   espflash on
   ```

2. 看到提示 `esp12f flash bridge on...` 后，立刻关闭串口终端，释放这个 COM 口。
3. 使用同一个 STM32 USART1 COM 口执行：

   ```bash
   esptool.py --chip esp8266 --port COMx --baud 115200 chip_id
   esptool.py --chip esp8266 --port COMx --baud 115200 write_flash 0x00000 firmware.bin
   ```

4. 烧录结束后等待 30 秒空闲自动退出 bridge，或复位整板。
5. 重新打开 USART1 串口终端，输入：

   ```text
   espflash status
   status
   ```

   确认 bridge 已 inactive，ESP12F 正常协议和网页控制恢复。

### Arduino IDE 流程

Arduino IDE 可以直接作为 ESP8266 上传工具使用，但要先手动让 STM32 进入 bridge：

1. Arduino IDE 安装 ESP8266 boards。
2. Board 选择 `Generic ESP8266 Module`。
3. Port 选择 STM32 USART1 对应的 COM 口。
4. Upload Speed 选择 `115200`。
5. Flash Mode 优先选 `DOUT`；若确认模组 flash 支持其他模式，再尝试 `DIO/QIO`。
6. 用串口终端向 USART1 输入 `espflash on`。
7. 关闭串口终端，释放 COM 口。
8. 回到 Arduino IDE 点击 Upload。
9. 上传完成后等待 30 秒自动退出 bridge，或复位整板让 ESP12F 正常启动。

注意：当前方案不依赖 Arduino IDE 的 DTR/RTS 自动复位。ESP12F 是否进入下载模式由 STM32 的 `espflash on` 显式控制。

### 烧录桥行为

- `espflash on` 会清空当前运动命令，停止 open-loop/raw 测试，避免烧录期间误运动。
- bridge active 后，USART1 收到的每个字节都会转发到 USART2，USART2 收到的每个字节都会转发到 USART1。
- bridge active 后，ESP12F 正常网页控制协议暂停，ESP status 帧不会混入烧录数据。
- bridge active 后，USART1 调试台不会解析普通命令，避免二进制烧录流被误当作文本命令。
- 30 秒无串口活动后自动退出 bridge，`ESP_IO0` 拉高并复位 ESP12F 正常启动。
- 如果烧录工具中途断开，等待自动退出或复位整板即可恢复。

### 烧录排查

- `esptool.py` 打不开 COM：串口终端或 Arduino Serial Monitor 仍占用端口，先关闭。
- `Timed out waiting for packet header`：通常是没有先执行 `espflash on`，或 ESP12F 没进入 download mode。
- `Invalid head of packet` / 上传中断：确认波特率是 `115200`，不要使用 Arduino IDE 的高速上传。
- `chip_id` 读不到：检查 `ESP_IO0=0` 后是否复位，检查 `ESP_TX/ESP_RX` 与 STM32 USART2 方向是否正确。
- 烧录后程序不启动：确认 bridge 已退出，`ESP_IO0=1`，然后复位 ESP12F 或整板。
- 想看桥接计数：退出 bridge 后输入 `espflash status`，查看 rx/tx、overflow、uart_err、auto_exit。

## Bring-up 顺序

1. USART1 输入 `rtos`，确认任务都已创建，heap、栈余量和 missed-period 正常。
2. 输入 `status`，确认 ADC、编码器、DRV fault、BMI270、PS2、ESP12F、LINE 状态字段会刷新。
3. 架空轮后执行 `m1` 到 `m4` 单轮 raw 测试，确认 DRV8874 `IN1/IN2` 方向、`nSLEEP` 和 `nFAULT`。
4. 执行 `left`、`right`、`motor`，确认 `CHASSIS_Mx_SIDE` 配置对应的左右侧聚合正确。
5. 空转和落地分别检查 TIM2/TIM3/TIM4/TIM5 编码器方向与速度符号。
6. 检查四路电流和 `VBAT_SENSE` 换算，必要时只调整 `chassis_config.h` 中校准常量。
7. 使用 `vel` 做低速闭环测试，观察 `status` 中目标速度、实际速度、fault-stop 和 missed-period。
8. 执行 `imutest`/`imuinit`/`imudiag`，确认 BMI270 chip-id 为 `0x24`；执行 `imucal` 静止校准陀螺零偏；随后观察 `status` 中 Euler 角和采样错误计数。
9. 执行 `espflash on`，用 `esptool.py --chip esp8266 --port COMx --baud 115200 chip_id` 验证 ESP12F 烧录通道。
10. 烧录 ESP12F 网页固件后，验证 ESP12F 网页控制和 USART2 帧协议。
11. 接入 UART4 巡线模块，查看 `status` 中 LINE 收帧计数。

## 构建环境

本地构建需要：

- CMake 3.22 或更高版本
- Ninja
- GNU Arm Embedded Toolchain，命令需可通过 `arm-none-eabi-gcc` 访问，并包含 newlib / nano specs 支持

可用以下命令检查环境：

```bash
cmake --version
ninja --version
arm-none-eabi-gcc --version
```

## 本地构建

Debug：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Release：

```bash
cmake --preset Release
cmake --build --preset Release
```

构建产物：

```text
build/Debug/F407_V2.0.elf
build/Debug/F407_V2.0.map
build/Release/F407_V2.0.elf
build/Release/F407_V2.0.map
```

本地最近一次验证结果：

| 命令 | 结果 |
| --- | --- |
| `cmake --build --preset Debug` | Passed, RAM 70768 B / 128 KB, FLASH 81012 B / 512 KB |

## GitHub Actions

CI 文件位于 `.github/workflows/firmware-build.yml`，工作流名称为 `Firmware Build`。

触发条件：

- push 到 `main`
- 向 `main` 发起 Pull Request

CI 行为：

- 使用 `ubuntu-24.04`
- 安装 `cmake`、`ninja-build`、`gcc-arm-none-eabi`、`libnewlib-arm-none-eabi`
- 矩阵构建 `Debug` 与 `Release`
- 上传 `firmware-Debug` 与 `firmware-Release` artifacts，包含 `.elf` 与 `.map`

## Git 与行尾策略

仓库使用 `.gitattributes` 固定文本文件为 LF，避免 Windows `core.autocrlf=true` 把 `Drivers/`、HAL、CMSIS 等第三方文件刷成大面积无关变更。本地已把 `core.excludesfile` 指向仓库可读的 `.git/info/exclude`，避免系统级 ignore 文件权限问题影响 `git status` 输出。

## 开发注意事项

- 不改硬件引脚定义；业务代码只消费 `Core/Inc/main.h` 与 `.ioc` 中已有标签。
- 不要把 `build/`、IDE 临时文件、系统噪声或下载产物提交入库。
- CubeMX 生成区外的业务代码优先放入 `App/` 与 `BSP/`；必须接入生成文件时只写 `USER CODE BEGIN/END` 区域。
- 修改 `.ioc` 后重新生成代码时，需要复查 `Core/Src/freertos.c`、`Core/Inc/main.h`、`Core/Src/gpio.c`、`cmake/stm32cubemx/CMakeLists.txt` 和顶层 `CMakeLists.txt` 的差异。
- 修改时钟、链接脚本、FreeRTOS heap 或 CMake 工具链后，应同时验证 Debug 和 Release。
- 控制安全相关改动必须保留 reject-and-stop 语义，不能静默夹紧非法命令后继续运行。
- BMI270 驱动已完整落地（Bosch 配置表、陀螺校准、姿态估计）；温度读取、中断驱动和 FIFO 模式待后续实现。
- ESP12F bridge 第一版固定 `115200`；确认硬件串口稳定后，再考虑动态高速烧录。

## 提交前检查

建议至少执行：

```bash
git status --short --branch
git diff --check -- .gitattributes README.md CMakeLists.txt Core/Src/freertos.c App BSP
cmake --build --preset Debug
```

涉及构建配置、链接脚本、启动文件或 FreeRTOS 配置时，再补充：

```bash
cmake --preset Release
cmake --build --preset Release
```

## 常见排查

- `arm-none-eabi-gcc` 找不到：确认 GNU Arm Embedded Toolchain 已安装，并加入 `PATH`。
- `nano.specs` 或 newlib 相关链接错误：确认工具链包含 newlib；Ubuntu CI 中需要安装 `libnewlib-arm-none-eabi`。
- GitHub Actions 没有产物：先检查对应 preset 是否构建成功，再确认 artifact 路径是否仍为 `build/<Preset>/F407_V2.0.elf` 和 `.map`。
- `git status` 出现大量 `Drivers/` 修改：先检查 `.gitattributes` 是否存在，再执行 `git restore --worktree -- Drivers` 清掉第三方库行尾噪声。
- 重新生成 CubeMX 后构建失败：优先检查顶层 `CMakeLists.txt` 是否仍包含 `App/`、`BSP/` 业务源文件与 include path。
