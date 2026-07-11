# SlamRobot Chassis Control — F407 V2.0

基于 STM32F407VET6 的四轮左右差速底盘控制固件。构建系统采用 STM32CubeMX + FreeRTOS + CMake + Ninja + GNU Arm Embedded Toolchain。

> **当前状态**：V2.0 已接入完整底盘控制链，默认两驱启用 M2（左侧）和 M3（右侧），可通过布局配置切换为四驱或自定义组合。电机驱动为 DRV8874 H-bridge，IMU 为 Bosch BMI270，无线模块为板载 ESP12F，巡线使用 HiWonder 八路传感器。

## 快速开始

```bash
# 固件构建
cmake --preset Debug   && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release

# Host 单元测试（无需交叉编译工具链）
cmake -S tests/host -B build/host-tests-ninja -G Ninja
cmake --build build/host-tests-ninja
ctest --test-dir build/host-tests-ninja --output-on-failure
```

## 工程概览

| 类别 | 配置 |
| --- | --- |
| **MCU** | STM32F407VET6 (ARM Cortex-M4F, 168 MHz, LQFP100) |
| **工具链** | GNU Arm Embedded Toolchain (`arm-none-eabi-gcc`, newlib-nano) |
| **构建系统** | CMake 3.22+ / Ninja, Presets (`Debug` / `Release`) |
| **RTOS** | FreeRTOS Kernel V10.3.1 + CMSIS-RTOS2 |
| **CubeMX** | STM32Cube FW_F4 V1.28.3, `.ioc` 配置 |
| **链接脚本** | `STM32F407XX_FLASH.ld` |
| **控制优先级** | `上位机(USART3) > PS2 > ESP12F > 巡线(UART4) > 调试台(USART1)` |
| **底盘布局** | 默认两驱 M2+M3；支持 M1+M2 左侧 / M3+M4 右侧四驱及自定义布局 |
| **构建验证** | Debug — RAM 81.2KB (61.9%) / FLASH 180.7KB (34.5%) — Host 测试 18/18 通过 — CI 双 preset + host-tests + format-check + static-analysis + CubeMX 安全配置检查 |

> V2.0 实板逻辑映射以 BSP 为准：M2 使用 EN/PWM=`PE11`、PH/GPIO=`PC7`、nFAULT=`PD14`、编码器=`TIM4 PD12/PD13`、电流采样=`PC1`；M3 使用 EN/PWM=`PE13`、PH/GPIO=`PC8`、nFAULT=`PA3`、编码器=`TIM3 PB4/PB5`、电流采样=`PC2`。CubeMX 生成文件中的 M2/M3 GPIO label 保留旧命名。

## 目录结构

```
├── App/              应用层（业务逻辑，不直接操作 HAL 外设句柄）
│   ├── chassis/       底盘控制核心 (差速/布局/PID/PWM/任务入口)
│   ├── control/       控制源仲裁 (优先级/超时/ESTOP/fault-stop)
│   ├── debug/         USART1 调试命令台 + Reset Trace
│   ├── display/       OLED 三阶段 UI (欢迎/自检/运行)
│   ├── monitor/       电压/电流/编码器/DRV fault 状态聚合
│   ├── param/         运行时参数与校准快照
│   └── protocol/      上位机帧协议 (USART3 + ESP12F 共用)
├── BSP/              板级驱动层（硬件抽象，每个外设独立目录）
│   ├── adc/           ADC DMA 采样换算
│   ├── encoder/       编码器计数/回绕差分/速度计算
│   ├── esp12f/        ESP12F 协议与烧录桥
│   ├── flash/         STM32 Flash 参数镜像读写
│   ├── imu/           BMI270 SPI 驱动 (配置表/校准/姿态估计/诊断)
│   ├── led/           状态 LED
│   ├── line/          八路巡线传感器 (帧解析 + P 控制)
│   ├── motor/         DRV8874 H-bridge PWM 驱动
│   ├── oled/          SSD1306 OLED 驱动 (I2C1 128×64)
│   ├── pid/           速度环 PID
│   └── ps2/           PS2 手柄硬件读取 (DWT 时序容错)
├── Core/              CubeMX 生成区 (HAL 初始化/FreeRTOS 入口/中断)
├── Drivers/           CMSIS + STM32F4 HAL/LL (第三方，禁止手动修改)
├── Middlewares/       FreeRTOS 内核 (第三方，禁止手动修改)
├── cmake/             工具链文件 + CubeMX CMake 目标
├── docs/              详细文档
├── firmware/          ESP12F Arduino 固件源码
│   └── esp12f/        WiFi 帧协议通信 + 网页遥控固件 (F407_ESP12F/)
│       └── F407_ESP12F/  Arduino 工程目录 (F407_ESP12F.ino)
└── .github/workflows/ CI 构建流水线
```

## 详细文档

| 文档 | 内容 |
| --- | --- |
| [外设资源](docs/peripherals.md) | 电机 PWM/编码器/ADC/通信接口/BMI270/ESP12F GPIO/PS2/巡线传感器 — 完整硬件规格 |
| [控制体系](docs/control-system.md) | 控制链数据流、五级优先级仲裁、安全语义、FreeRTOS 十任务模型、调度监控、底盘布局配置 |
| [Upper Protocol v2](docs/upper-protocol-v2.md) | USART3/ESP12F 帧格式、65B STATUS、99B IMU、温度编码与黄金测试向量 |
| [调试命令台](docs/debug-console.md) | USART1 全命令参考、CSV 日志字段过滤、line 命令输出格式 |
| [STM32 主控烧录](docs/stm32-flashing.md) | STM32CubeProgrammer / OpenOCD / st-flash 烧录流程 |
| [ESP12F 烧录](docs/esp12f-flashing.md) | esptool.py / Arduino IDE 烧录流程、透明桥行为、常见故障排查 |
| [ESP12F 固件](firmware/esp12f/) | Arduino 源码：首次配置 AP、WebSocket owner 租约、遥测广播、upper_protocol 帧协议通信 |
| [Bring-up 验收](docs/bring-up.md) | 11 步分阶段验收：从 RTOS 状态到巡线跟踪 |
| [开发指南](docs/development.md) | 环境搭建、构建命令、GitHub Actions CI、Git 策略、开发约束、常见排查 |
| [HIL 冒烟](docs/hil-smoke.md) | USART1 只读硬件在环冒烟脚本与验收边界 |
| [教学实验](docs/labs/README.md) | 13 个分级实验：开环运动 → 编码器反馈 → PID 调参 → 电流限制 → IMU → 巡线 → PS2 遥操 → 系统安全 → FreeRTOS → 仲裁 → ADC → OLED → ESP12F WiFi |
