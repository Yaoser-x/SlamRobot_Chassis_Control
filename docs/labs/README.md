# F407 V2.0 底盘控制实验教程

## 课程概述

本教程基于 STM32F407VET6 四轮差速底盘平台，涵盖从开环运动控制到多传感器融合的 13 个分级实验。每个实验包含原理讲解、操作步骤、数据分析和思考题，适合高校嵌入式系统、机器人控制、机电一体化等课程的实验教学。

## 硬件平台

| 组件 | 型号/参数 | 用途 |
|------|-----------|------|
| 主控 | STM32F407VET6 (Cortex-M4, 168 MHz) | 核心控制 |
| 电机驱动 | DRV8874 × 4 (H 桥) | PWM 调速 |
| 编码器 | 增量式 × 4 (TIM2/3/4/5 四倍频) | 速度反馈 |
| IMU | BMI270 (SPI, 6 轴) | 姿态估计 |
| 巡线 | HiWonder 八路红外 (UART4) | 黑线检测 |
| 电压/电流 | ADC1 DMA 5 通道 (PC0-PC4) | 电池与电流监测 |
| 显示 | SSD1306 OLED 128×64 (I2C1) | 状态显示 |
| 无线 | ESP12F (USART2, WiFi) | 远程控制 |
| 手柄 | PS2 (GPIO bit-bang) | 遥操作 |
| 调试 | USART1 (115200 8N1) | 命令台 + CSV 日志 |

## 软件准备

- **串口终端**：PuTTY / Tera Term / minicom，115200 8N1，连接 USART1 (PB6/PB7)
- **数据分析**：Python 3 + pandas + matplotlib，或 SerialPlot、Excel
- **固件构建**（仅 Lab 03 需要）：arm-none-eabi-gcc + CMake 3.22+ + Ninja，参见 [`docs/development.md`](../development.md)

## 实验分级

| 级别 | 实验 | 核心知识点 | 依赖 |
|------|------|-----------|------|
| **入门** | [Lab 01 基础运动控制](lab01_basic_motion.md) | PWM 开环控制、差速转向原理 | — |
| | [Lab 02 编码器与速度反馈](lab02_encoder_feedback.md) | 正交编码器、速度换算、方向验证 | Lab 01 |
| **进阶** | [Lab 03 PID 速度环调参](lab03_pid_tuning.md) | 闭环控制、PID 参数、阶跃响应 | Lab 02 |
| | [Lab 04 电流监测与限流](lab04_current_limit.md) | 分流电阻、过流保护、负载特性 | Lab 02 |
| | [Lab 05 IMU 与姿态估计](lab05_imu_analysis.md) | 陀螺仪/加速度计、互补滤波、零偏校准 | — |
| | [Lab 06 巡线传感器与跟踪控制](lab06_line_following.md) | 红外阵列、加权平均、P 控制 | Lab 01 |
| **综合** | [Lab 07 PS2 手柄遥操作](lab07_ps2_teleop.md) | SPI bit-bang、摇杆映射、宏指令 | Lab 01 |
| | [Lab 08 系统安全与故障诊断](lab08_system_safety.md) | 故障锁存、ESTOP、过流/DRV 保护 | Lab 01 |
| | [Lab 09 FreeRTOS 实时性能分析](lab09_freertos_analysis.md) | 任务调度、堆栈监测、实时性评估 | — |
| | [Lab 10 控制源仲裁与优先级抢占](lab10_control_arbitration.md) | 多源优先级、超时退让、安全拒绝 | Lab 01/07 |
| **系统** | [Lab 11 ADC 采样与电池监测](lab11_adc_battery.md) | 分压网络、分流电阻、EMA 滤波、校准 | — |
| | [Lab 12 OLED 显示与 I2C 自检](lab12_oled_i2c.md) | I2C 总线扫描、SSD1306、POST 自检 | — |
| | [Lab 13 ESP12F WiFi 桥接与远程控制](lab13_esp12f_wifi.md) | 协议帧、UART 透明桥、ESP8266 烧录 | — |

## 教学路线建议

### 32 学时课程（推荐）

```
第 1 周: Lab 01 (2 学时) + Lab 02 (2 学时)
第 2 周: Lab 03 (4 学时)                    ← 核心，需要构建环境
第 3 周: Lab 04 (2 学时) + Lab 05 (2 学时)
第 4 周: Lab 06 (2 学时) + Lab 07 (2 学时)
第 5 周: Lab 08 (2 学时) + Lab 09 (2 学时)
第 6 周: Lab 10 (2 学时) + Lab 11 (2 学时)
第 7 周: Lab 12 (2 学时) + Lab 13 (2 学时)
第 8 周: 综合项目 (4 学时)                   ← 巡线竞速 / 遥控避障
```

### 16 学时短课程

```
Lab 01 → Lab 02 → Lab 03 → Lab 05 → Lab 06 → Lab 08
（跳过 PS2/仲裁/ADC/OLED/ESP12F，聚焦运动控制核心链）
```

## 实验前安全检查

> **每次实验前必须确认以下事项，不可跳过。**

1. **架空车轮**：首次通电或修改电机方向后，必须将底盘架空（车轮离地），避免意外飞车
2. **ESTOP 准备**：熟悉 `estop 1` 和 `stop` 命令的位置，出现异常立即输入
3. **电池电压**：`status` 确认 `vbat` 在 10.5V 以上，低电压可能导致控制异常
4. **DRV 故障**：`status` 确认 `drv_fault=0,0,0,0`，任一为 1 表示驱动板故障
5. **串口终端就绪**：确保终端已连接 USART1（115200 8N1），回车后能看到命令提示

## 命令速查

| 分类 | 命令 | 说明 |
|------|------|------|
| 状态 | `status` / `s` | 全系统快照（编码器/底盘/ADC/IMU/系统） |
| | `rtos` | FreeRTOS 任务与堆栈状态 |
| | `line` | 巡线传感器原始数据 |
| 运动 | `motor L R` | 左右侧开环 permille（-900~900） |
| | `left P` / `right P` | 单侧开环 |
| | `m1 F R` ~ `m4 F R` | 单路电机 EN/PH raw 测试，输出按 `F-R` 解析 |
| | `vel V [W]` | 闭环速度控制 V(mm/s) W(mrad/s) |
| | `stop` | 停止全部运动 |
| 安全 | `estop 0\|1` | 清除/设置紧急停止 |
| | `clearfault` | 清除锁存过流/DRV 故障 |
| 日志 | `log 0` | 停止 CSV 日志 |
| | `log 1 [fld...]` | 启动 CSV 日志（可选字段过滤） |
| | `header` | 打印全字段 CSV 标题 |
| IMU | `imutest` | 探测 BMI270 芯片 |
| | `imuinit` | 初始化 BMI270 |
| | `imucal [n]` | 陀螺零偏校准 |
| | `imudiag` | SPI 硬件诊断 |
| 巡线 | `line on` / `line off` | 启用/禁用巡线控制 |
| ESP | `espflash on\|off\|status` | 烧录桥控制 |
| | `espreset` / `espboot 0\|1` | 复位/下载模式 |
| 其他 | `i2cscan` | I2C1 总线扫描 |
| | `help` | 打印命令列表 |

## 参考文档

- [`docs/peripherals.md`](../peripherals.md) — 外设引脚与接线
- [`docs/control-system.md`](../control-system.md) — 控制体系架构
- [`docs/debug-console.md`](../debug-console.md) — 调试命令台详解
- [`docs/bring-up.md`](../bring-up.md) — 硬件验收流程
- [`docs/development.md`](../development.md) — 构建与开发指南
