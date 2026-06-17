# SlamRobot Chassis Control — F407 V2.0

基于 STM32F407VET6 的四轮左右差速底盘控制固件。构建系统采用 STM32CubeMX + FreeRTOS + CMake + Ninja + GNU Arm Embedded Toolchain。

> **当前状态**：V2.0 已在 CubeMX 外设骨架上接入 V1.0 验证控制链。左侧 M1+M2、右侧 M3+M4 差速布局。电机驱动 DRV8874 H-bridge。IMU 为 Bosch BMI270。无线模块为板载 ESP12F。巡线使用 HiWonder 八路传感器。

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
| **底盘布局** | 四轮差速, M1+M2 左侧 / M3+M4 右侧, 可通过 `CHASSIS_Mx_ENABLED` + `CHASSIS_Mx_SIDE` 配置 |
| **构建验证** | Debug — RAM 70,904 B (54%) / FLASH 107,596 B (21%) — Host 测试 2/2 通过 — CI 双 preset |

## 目录结构

```
├── App/              应用层（业务逻辑，不直接操作 HAL 外设句柄）
│   ├── chassis/       底盘控制核心 (差速/布局/PID/PWM/任务入口)
│   ├── control/       控制源仲裁 (优先级/超时/ESTOP/fault-stop)
│   ├── debug/         USART1 调试命令台
│   ├── monitor/       电压/电流/编码器/DRV fault 状态聚合
│   └── protocol/      上位机帧协议 (USART3 + ESP12F 共用)
├── BSP/              板级驱动层（硬件抽象，每个外设独立目录）
│   ├── adc/           ADC DMA 采样换算
│   ├── encoder/       编码器计数/回绕差分/速度计算
│   ├── esp12f/        ESP12F 协议与烧录桥
│   ├── imu/           BMI270 SPI 驱动 (配置表/校准/姿态估计/诊断)
│   ├── led/           状态 LED
│   ├── line/          八路巡线传感器 (帧解析 + P 控制)
│   ├── motor/         DRV8874 H-bridge PWM 驱动
│   ├── pid/           速度环 PID
│   └── ps2/           PS2 手柄语义
├── Core/              CubeMX 生成区 (HAL 初始化/FreeRTOS 入口/中断)
├── Drivers/           CMSIS + STM32F4 HAL/LL (第三方，禁止手动修改)
├── Middlewares/       FreeRTOS 内核 (第三方，禁止手动修改)
├── cmake/             工具链文件 + CubeMX CMake 目标
├── docs/              详细文档
├── firmware/          ESP12F Arduino 固件源码
│   └── esp12f/        WiFi 桥接 + 网页遥控固件 (F407_ESP12F.ino)
└── .github/workflows/ CI 构建流水线
```

## 详细文档

| 文档 | 内容 |
| --- | --- |
| [外设资源](docs/peripherals.md) | 电机 PWM/编码器/ADC/通信接口/BMI270/ESP12F GPIO/PS2/巡线传感器 — 完整硬件规格 |
| [控制体系](docs/control-system.md) | 控制链数据流、五级优先级仲裁、安全语义、FreeRTOS 十任务模型、调度监控、底盘布局配置 |
| [调试命令台](docs/debug-console.md) | USART1 全命令参考、CSV 日志字段过滤、line 命令输出格式 |
| [ESP12F 烧录](docs/esp12f-flashing.md) | esptool.py / Arduino IDE 烧录流程、透明桥行为、常见故障排查 |
| [ESP12F 固件](firmware/esp12f/) | Arduino 源码：AP+STA WiFi、WebSocket 摇杆遥控、upper_protocol 帧协议 |
| [Bring-up 验收](docs/bring-up.md) | 11 步分阶段验收：从 RTOS 状态到巡线跟踪 |
| [开发指南](docs/development.md) | 环境搭建、构建命令、GitHub Actions CI、Git 策略、开发约束、常见排查 |
| [教学实验](docs/labs/README.md) | 13 个分级实验：开环运动 → 编码器反馈 → PID 调参 → 电流限制 → IMU → 巡线 → PS2 遥操 → 系统安全 → FreeRTOS → 仲裁 → ADC → OLED → ESP12F WiFi |
