# SlamRobot_Chassis_Control

STM32F407 V2.0 底盘控制固件工程。当前仓库是基于 STM32CubeMX 生成的 CMake 工程骨架，目标芯片为 `STM32F407VET6`（LQFP100），使用 STM32 HAL/LL、FreeRTOS CMSIS-RTOS2、CMake Presets、Ninja 和 GNU Arm Embedded Toolchain 构建。

> 当前状态：外设初始化、FreeRTOS 线程框架、CMake 构建与 GitHub Actions 已就绪；各业务线程目前仍是 `osDelay(1)` 占位循环，尚未实现底盘闭环控制、通信协议解析或传感器业务逻辑。

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

## 目录结构

```text
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
| HSE | 8 MHz |
| PLLM / PLLN / PLLP / PLLQ | 4 / 168 / 2 / 4 |
| SYSCLK | 168 MHz |
| HCLK / Cortex FCLK | 168 MHz |
| APB1 | 42 MHz |
| APB1 Timer Clock | 84 MHz |
| APB2 | 84 MHz |
| APB2 Timer Clock | 168 MHz |
| HAL timebase | TIM6 / `TIM6_DAC_IRQn` |

## 外设资源

### 电机 PWM 与 Break

| 资源 | 用途 | 引脚 / 通道 | 关键配置 |
| --- | --- | --- | --- |
| TIM1 | M1-M4 的 IN1 PWM | PE9 CH1, PE11 CH2, PE13 CH3, PE14 CH4 | Period `8399`, PWM1, Pulse `0` |
| TIM8 | M1-M4 的 IN2 PWM | PC6 CH1, PC7 CH2, PC8 CH3, PC9 CH4 | Period `8399`, PWM1, Pulse `0` |
| TIM1_BKIN | TIM1 Break 输入 | PE15 | Break 使能，低有效 |
| TIM8_BKIN | TIM8 Break 输入 | PA6 | Break 使能，低有效 |

TIM1/TIM8 均为高级定时器，当前只完成 PWM 通道与 Break 输入初始化，应用层尚未启动 PWM 输出或写入占空比控制逻辑。

### 编码器

| 电机 | 定时器 | 引脚 | 配置 |
| --- | --- | --- | --- |
| M1 | TIM2 | PA15 CH1, PB3 CH2 | Encoder TI12, IC filter `8`, Period `0xFFFFFFFF` |
| M2 | TIM3 | PB4 CH1, PB5 CH2 | Encoder TI12, IC filter `8`, Period `65535` |
| M3 | TIM4 | PD12 CH1, PD13 CH2 | Encoder TI12, IC filter `8`, Period `65535` |
| M4 | TIM5 | PA0 CH1, PA1 CH2 | Encoder TI12, IC filter `8`, Period `0xFFFFFFFF` |

当前工程完成编码器模式初始化；实际计数启动、速度计算、方向处理和闭环控制逻辑仍待实现。

### ADC 与电源/电流采样

ADC1 配置为 12-bit、连续扫描、软件触发、DMA continuous requests，使用 DMA2 Stream0 circular 模式。

| Rank | 通道 | 引脚 | 信号 |
| --- | --- | --- | --- |
| 1 | ADC1_IN10 | PC0 | M1_CURRENT |
| 2 | ADC1_IN11 | PC1 | M2_CURRENT |
| 3 | ADC1_IN12 | PC2 | M3_CURRENT |
| 4 | ADC1_IN13 | PC3 | M4_CURRENT |
| 5 | ADC1_IN14 | PC4 | VBAT_SENSE |

采样时间均为 `ADC_SAMPLETIME_84CYCLES`。当前只完成 ADC/DMA 初始化，采样缓冲区、启动调用和工程量换算尚未实现。

### 通信接口

| 接口 | 引脚 | 当前用途标签 | 参数 |
| --- | --- | --- | --- |
| USART1 | PB6 TX, PB7 RX | DEBUG_TX / DEBUG_RX | 115200 8N1 |
| USART2 | PD5 TX, PD6 RX | ESP_TX / ESP_RX | 115200 8N1 |
| USART3 | PD8 TX, PD9 RX | RPI_TX / RPI_RX | 115200 8N1，RX DMA |
| UART4 | PC10 TX, PC11 RX | LINE_TX / LINE_RX | 115200 8N1，RX DMA |

USART3 RX 使用 DMA1 Stream1，UART4 RX 使用 DMA1 Stream2。当前工程只完成 UART 与 DMA 初始化，中断接收、空闲中断、帧协议和缓冲管理还未落地。

### IMU 与 I2C

| 资源 | 引脚 | 配置 |
| --- | --- | --- |
| SPI2 SCK / MISO / MOSI | PB13 / PB14 / PB15 | Master, 2-line, 8-bit, Mode 0, prescaler 16 |
| IMU_CS | PB12 | GPIO output，默认高 |
| IMU_INT1 | PE0 | EXTI0 输入 |
| I2C1 SCL / SDA | PB8 / PB9 | 100 kHz, 7-bit addressing |

SPI2 在 `.ioc` 中计算速率约为 2.625 Mbit/s。当前工程只完成总线与片选引脚初始化，具体 IMU 型号、寄存器驱动和数据融合未在本仓库实现。

### 控制与状态 GPIO

| 信号 | 引脚 | 方向 / 默认状态 |
| --- | --- | --- |
| PS2_CLK | PE2 | Output, 默认高 |
| PS2_CS | PE3 | Output, 默认高 |
| PS2_DO | PE4 | Input, Pull-up |
| PS2_DI | PE5 | Output, 默认高 |
| TEST_LED | PE6 | Output |
| DRV_SLEEP_ALL | PE7 | Output |
| ESP_EN | PB10 | Output, 默认高 |
| ESP_RST | PB11 | Output, 默认高 |
| ESP_IO0 | PD7 | Output, 默认高 |
| LINE_EN | PD2 | Output |
| LINE_SET | PC12 | Output |
| M1_FAULT | PA2 | Input, Pull-up |
| M2_FAULT | PA3 | Input, Pull-up |
| M3_FAULT | PD14 | Input, Pull-up |
| M4_FAULT | PD15 | Input, Pull-up |

GPIO 名称来自 `F407_V2.0.ioc` 与 `Core/Inc/main.h`。README 不推断实际板卡接线，只记录当前固件配置。

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

当前已生成 `vApplicationStackOverflowHook()` 和 `vApplicationMallocFailedHook()`，但 hook 内仍是空实现。后续若接入调试串口或安全停机逻辑，应优先补这里。

### 当前线程表

| 线程 | 优先级 | 栈大小 | 入口函数 | 当前状态 |
| --- | --- | --- | --- | --- |
| `defaultTask` | Low | 128 words | `StartDefaultTask` | `osDelay(1)` 占位 |
| `safetyTask` | High | 256 words | `StartTask02` | `osDelay(1)` 占位 |
| `motorTask` | AboveNormal | 512 words | `StartTask03` | `osDelay(1)` 占位 |
| `rpiCommTask` | Normal | 512 words | `StartTask04` | `osDelay(1)` 占位 |
| `imuTask` | Normal | 512 words | `StartTask05` | `osDelay(1)` 占位 |
| `lineTask` | BelowNormal | 256 words | `StartTask06` | `osDelay(1)` 占位 |
| `espTask` | Low | 512 words | `StartTask07` | `osDelay(1)` 占位 |

线程名称体现了后续模块划分意图，但当前不代表功能已经完成。实现业务逻辑时应写在 CubeMX 保留的 `USER CODE` 区域内，避免重新生成代码时丢失。

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

| Preset | RAM | FLASH |
| --- | --- | --- |
| Debug | 66248 B / 128 KB, 50.54% | 35372 B / 512 KB, 6.75% |
| Release | 66248 B / 128 KB, 50.54% | 20420 B / 512 KB, 3.89% |

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

## 开发注意事项

- 不要把 `build/`、IDE 临时文件、系统噪声或下载产物提交入库。
- CubeMX 生成区外的业务代码应优先放入 `USER CODE BEGIN/END` 区域。
- 修改 `.ioc` 后重新生成代码时，需要复查 `Core/Src/freertos.c`、`cmake/stm32cubemx/CMakeLists.txt` 和外设初始化文件的差异。
- 修改时钟、链接脚本、FreeRTOS heap 或 CMake 工具链后，应同时验证 Debug 和 Release。
- 在业务任务真正实现前，README 中的任务说明只代表当前线程框架和预留职责，不代表控制功能已经可用。

## 提交前检查

建议至少执行：

```bash
git status --short --branch
git diff --check -- README.md
cmake --preset Debug
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
- 重新生成 CubeMX 后构建失败：优先检查 `cmake/stm32cubemx/CMakeLists.txt` 的源文件清单和 include path 是否同步。
