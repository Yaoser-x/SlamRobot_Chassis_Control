# F407 V2.0 完整功能清单

> **固件版本：** 1.0.0
> **目标平台：** STM32F407VET6 (ARM Cortex-M4F, 168MHz)
> **RTOS：** FreeRTOS Kernel V10.3.1 + CMSIS-RTOS2
> **构建系统：** CMake 3.22+ / Ninja + GNU Arm Embedded Toolchain
> **生成日期：** 2026-07-12

---

## 目录

1. [系统架构概览](#一系统架构概览)
2. [底盘运动控制](#二底盘运动控制)
3. [控制源与仲裁](#三控制源与仲裁)
4. [传感器与外设驱动](#四传感器与外设驱动)
5. [通信协议](#五通信协议)
6. [安全与诊断](#六安全与诊断)
7. [UI 与显示](#七ui-与显示)
8. [参数与持久化](#八参数与持久化)
9. [测试体系](#九测试体系)
10. [构建身份与工具链](#十构建身份与工具链)

---

## 一、系统架构概览

### 1.1 硬件平台

| 项目 | 规格 |
|------|------|
| **MCU** | STM32F407VET6, ARM Cortex-M4F with FPU, LQFP100 |
| **主频** | SYSCLK 168MHz（HSE 8MHz → PLL: M=8, N=336, P=÷2, Q=7） |
| **Flash** | 512KB |
| **RAM** | 192KB（含 64KB CCM） |
| **电机驱动** | DRV8874 H-bridge ×4（TIM1 四通道 PWM + GPIO PH/DIR 方向控制） |
| **编码器** | 四路正交编码器（TIM2/TIM3/TIM4/TIM5 四倍频） |
| **IMU** | Bosch BMI270（SPI2, INT1 中断, 100Hz ODR） |
| **无线模块** | ESP8266 ESP-12F（USART2, WiFi + WebSocket） |
| **巡线传感器** | HiWonder 八路红外（USART4, 二进制帧协议） |
| **显示屏** | SSD1306 128×64 OLED（I2C1, 地址 0x3C） |
| **调试口** | USART1（命令台 + CSV/JSON 日志流） |
| **上位机口** | USART3（Raspberry Pi 通信, DMA 环形缓冲） |
| **手柄** | PS2 无线手柄（GPIO bit-bang SPI, DWT 时序容错） |
| **看门狗** | IWDG（预分频 32, 重载 1000, 超时 ~1.0s） |
| **硬件保护** | TIM1_BKIN (PE15) + TIM8_BKIN (PA6) 低电平硬件切断 PWM |

### 1.2 软件分层

```
App/                    应用层（业务逻辑，不直接操作 HAL 外设句柄）
  chassis/              底盘控制核心（差速模型、PID、PWM 输出、任务入口）
  control/              控制源仲裁（优先级、超时、ESTOP/fault-stop）
  current/              电流保护
  debug/                USART1 调试命令台 + Reset Trace 崩溃追踪
  display/              OLED 三阶段 UI 与 IMU 校准显示
  imu/                  IMU 自动校准门控
  monitor/              电压/电流/编码器/DRV fault 状态聚合 + POST
  param/                运行时参数存储
  protocol/             上位机帧协议（USART3 + ESP12F 共用）
BSP/                    板级驱动层（硬件抽象，每个外设独立目录）
  adc/                  ADC DMA 采样换算
  encoder/              编码器计数、回绕差分、速度计算
  esp12f/               ESP12F 通信透传桥（烧录 + AT 双模式）
  flash/                STM32 Flash 参数持久化
  imu/                  BMI270 SPI 驱动（配置表、校准、姿态估计）
  led/                  状态 LED
  line/                 八路巡线传感器驱动
  motor/                DRV8874 H-bridge PWM 驱动
  oled/                 SSD1306 OLED 驱动
  pid/                  速度环 PID
  ps2/                  PS2 手柄硬件读取
Core/                   CubeMX 生成区（HAL 初始化、FreeRTOS 入口、中断）
Drivers/                CMSIS + STM32F4 HAL/LL（第三方，禁止手动修改）
Middlewares/            FreeRTOS 内核（第三方）
cmake/                  工具链文件 + CubeMX CMake 目标
tests/host/             Host 测试（34 target）
firmware/esp12f/        ESP12F Arduino 固件源码
```

### 1.3 FreeRTOS 任务模型

共 **10 个业务任务**，通过 `Core/Src/freertos.c` 的 `MX_FREERTOS_Init()` 创建：

| # | 任务名 | 入口函数 | 栈大小 | 优先级 | 周期 | 调度方式 | 核心职责 |
|---|--------|----------|--------|--------|------|----------|----------|
| 1 | `safetyTask` | `Task_Safety` | 4096B | High (osPriorityHigh) | 20ms | osDelayUntil | 状态聚合、命令超时检测、fault-stop 触发、IWDG 条件喂狗、POST 运行时检查 |
| 2 | `motorTask` | `Task_MotorControl` | 2048B | AboveNormal (osPriorityAboveNormal) | 10ms | osDelayUntil | 编码器更新、ChassisService_Step（差速+PID+PWM）、ADC 静止门控 |
| 3 | `rpiCommTask` | `Task_RpiComm` | 2048B | Normal (osPriorityNormal) | 5ms | osDelayUntil | USART3 DMA 上位机协议收发（200Hz） |
| 4 | `imuTask` | `Task_Imu` | 2048B | Normal (osPriorityNormal) | 事件驱动 | osThreadFlagsWait (EXTI0) | BMI270 FIFO/直接采样、Mahony 融合、校准门控、自动校准状态机 |
| 5 | `ps2Task` | `Task_Ps2` | 2048B | Normal (osPriorityNormal) | 20ms | osDelayUntil | PS2 手柄读取、摇杆归一化、定角宏控制、巡线启停 |
| 6 | `lineTask` | `Task_Line` | 4096B | BelowNormal (osPriorityBelowNormal) | 5ms | osDelayUntil | UART4 巡线帧解析、PD 控制指令提交（200Hz） |
| 7 | `espTask` | `Task_Esp12f` | 2048B | BelowNormal (osPriorityBelowNormal) | 5ms | osDelayUntil | ESP12F 协议处理、状态/诊断帧发送、flash bridge 服务（200Hz） |
| 8 | `debugTask` | `Task_Debug` | 8192B | BelowNormal (osPriorityBelowNormal) | 10ms | osDelay | USART1 调试命令台、CSV/JSON 日志流（100Hz） |
| 9 | `ledTask` | `Task_Led` | 512B | Low (osPriorityLow) | 50ms | osDelayUntil | 状态 LED 闪烁模式 |
| 10 | `oledTask` | `Task_Oled` | 1024B | Low (osPriorityLow) | 100ms | osDelayUntil | OLED 三阶段 UI 刷新 |

**FreeRTOS 配置（FreeRTOSConfig.h）：**
- Tick Rate: 1000Hz
- Max Priorities: 56
- Total Heap: 49152 bytes (heap_4)
- Max Task Name Length: 16
- 特性：抢占式调度、静态+动态内存分配、互斥锁/递归互斥锁/计数信号量、软件定时器、栈溢出检测（方法 2）、FPU 使能、任务通知/SuspendResume/Enumerate/EventFlags (ISR)
- `configASSERT` 自定义映射到 `vApplicationAssertHook`（写入 ResetTrace 后死循环）

**FreeRTOS Hooks：**
- `vApplicationStackOverflowHook` — 捕获溢出的任务名，写入 ResetTrace，死循环
- `vApplicationMallocFailedHook` — 写入 ResetTrace，死循环
- `vApplicationAssertHook` — `configASSERT` 失败时写入 ResetTrace（`FREERTOS_FATAL_ASSERT`），死循环
- `FreeRtos_FatalStop` — 置低 SLEEP_ALL 关断电机，关全局中断，`__NOP()` 死循环

### 1.4 数据流与控制链

```
控制源提交命令 → ControlService（优先级仲裁 + 独立超时 + reject-and-stop）
    → ChassisService_Step（每 10ms，motorTask 驱动）
        → MotorHardwareLayout 查表（电机启用/侧别/方向）
        → DifferentialDriveKinematics_ResolveDifferentialTargets（linear_x/angular_z → 左右轮目标速度）
        → 速度斜坡（加速/减速限制 + 方向反转状态机）
        → 直行补偿（轮速耦合 + IMU 航向保持）
        → 轮速等比缩放（一侧达到上限时等比压缩另一侧）
        → 每电机独立 PID（Kp/Ki/Kd，含抗积分饱和 + 执行器方向感知）
        → CurrentGuard 电流保护（observe / soft-limit / fault-latch）
        → MotorDriver_SetPermille → TIM1 PWM + GPIO PH/DIR 输出
```

**控制源优先级：** `上位机(USART3) > PS2 > ESP12F > 巡线(UART4) > 调试台(USART1)`

### 1.5 构建系统

**预设配置：**

| Preset | 编译选项 | 链接脚本 | 产物 |
|--------|---------|---------|------|
| Debug | `-O0 -g3` | STM32F407XX_FLASH.ld | .elf / .hex / .bin / .map |
| Release | `-Os -g0` | STM32F407XX_FLASH.ld | .elf / .hex / .bin / .map |

**编译器：** `arm-none-eabi-gcc`（newlib-nano, `-u,_printf_float`）
**编译标准：** C11 with GNU extensions
**告警级别：** `-Wall -Wextra -Wpedantic -Werror`（App/BSP 源码）
**替代工具链：** 支持 ST ARM Clang（`cmake/starm-clang.cmake`，三种配置：STARM_HYBRID / STARM_NEWLIB / STARM_PICOLIBC）

**编译宏：**
- `USE_HAL_DRIVER`, `STM32F407xx`（始终定义）
- `DEBUG`（Debug preset）
- `DEBUG_CONSOLE_RELEASE_REQUIRES_ARM`（Release preset，强制维护授权）

**构建命令：**
```bash
cmake --preset Debug   && cmake --build --preset Debug
cmake --preset Release && cmake --build --preset Release
```

---

## 二、底盘运动控制

### 2.1 差速运动学模型

**模块：** `App/chassis/chassis_math.c`
**头文件：** `App/chassis/chassis_math.h`

**核心公式（标准两轮差速）：**
```
v_left  = linear_x - (angular_z × track_width / 2)
v_right = linear_x + (angular_z × track_width / 2)
```

**函数：**
- `DifferentialDriveKinematics_ResolveDifferentialTargets(linear_x, angular_z, track_width_m, *left_mps, *right_mps)` — 计算左右轮目标速度，线性速度单位 m/s，角速度单位 rad/s
- `DifferentialDriveKinematics_ControlDt(now_ms, *last_step_ms, *initialized, *dt_s)` — 控制周期时间差计算：
  - 首次调用默认 0.010s
  - 两次调用间隔 >100ms 返回超时（0）
  - 防止零时间间隔（返回 0）

### 2.2 电机布局系统

**模块：** `BSP/motor_hardware_layout.c`
**头文件：** `BSP/motor_hardware_layout.h`
**配置：** `Domain/config/control_config.h / BSP/bsp_config.h`

**编译期配置常量（control_config.h / bsp_config.h）：**

| 宏 | 默认值 | 说明 |
|----|--------|------|
| `CHASSIS_M1_ENABLED` | 0 | M1 启用（默认禁用） |
| `CHASSIS_M2_ENABLED` | 1 | M2 启用（默认左侧） |
| `CHASSIS_M3_ENABLED` | 1 | M3 启用（默认右侧） |
| `CHASSIS_M4_ENABLED` | 0 | M4 启用（默认禁用） |
| `CHASSIS_M1_SIDE` | SIDE_LEFT | M1 侧别 |
| `CHASSIS_M2_SIDE` | SIDE_LEFT | M2 侧别（左侧） |
| `CHASSIS_M3_SIDE` | SIDE_RIGHT | M3 侧别（右侧） |
| `CHASSIS_M4_SIDE` | SIDE_RIGHT | M4 侧别 |
| `CHASSIS_M1_MOTOR_DIR` | 1 | M1 电机方向（1=正, -1=反） |
| `CHASSIS_M2_MOTOR_DIR` | 1 | M2 电机方向 |
| `CHASSIS_M3_MOTOR_DIR` | 1 | M3 电机方向 |
| `CHASSIS_M4_MOTOR_DIR` | 1 | M4 电机方向 |
| `CHASSIS_M1_ENCODER_DIR` | 1 | M1 编码器方向 |
| `CHASSIS_M2_ENCODER_DIR` | 1 | M2 编码器方向 |
| `CHASSIS_M3_ENCODER_DIR` | 1 | M3 编码器方向 |
| `CHASSIS_M4_ENCODER_DIR` | 1 | M4 编码器方向 |

**查表 API：**
- `MotorHardwareLayout_IsMotorEnabled(motor)` — 电机是否启用
- `MotorHardwareLayout_GetSide(motor)` — 返回 SIDE_LEFT / SIDE_RIGHT / SIDE_NONE
- `MotorHardwareLayout_GetMotorDir(motor)` — 返回 1 或 -1
- `MotorHardwareLayout_GetEncoderDir(motor)` — 返回 1 或 -1
- `MotorHardwareLayout_GetMotorCount()` — 启用的电机总数
- `MotorHardwareLayout_GetEnabledMask()` — 启用电机位掩码

**V2.0 硬件映射（以 BSP 为准）：**

| 逻辑编号 | 物理 TIM/GPIO | 侧别 | 备注 |
|---------|---------------|------|------|
| M1 | TIM1 CH1 / PE9, PH=PC7, nFAULT=PC6, 编码器=TIM2 | LEFT (禁用) | 默认禁用 |
| M2 | TIM1 CH2 / PE11, PH=PC7, nFAULT=PD14, 编码器=TIM4 | LEFT | **左侧驱动轮** |
| M3 | TIM1 CH3 / PE13, PH=PC8, nFAULT=PA3, 编码器=TIM3 | RIGHT | **右侧驱动轮** |
| M4 | TIM1 CH4 / PE14, PH=PC9, nFAULT=PA4, 编码器=TIM5 | RIGHT (禁用) | 默认禁用 |

### 2.3 PID 速度闭环

**模块：** `BSP/pid/pid_controller.c`
**头文件：** `BSP/pid/pid_controller.h`

**四路独立 PID 控制器**（M1-M4），M1-M3 运行完整 PID（Kp/Ki/Kd 非零），M4 默认纯 P。

**数据结构：**
```c
typedef struct {
    float kp, ki, kd;          // PID 增益
    float integral_limit;      // 积分限幅（抗饱和）
    float output_limit;        // 输出范围 [-output_limit, output_limit]
} pid_params_t;

typedef struct {
    pid_params_t params;
    float integral;            // 积分累积量
    float prev_error;          // 上一次误差（用于微分项）
    uint8_t initialized;
} pid_state_t;
```

**函数：**
- `PidController_Init(pid, params)` — 设置参数，复位状态
- `PidController_Reset(pid)` — 清零积分和上次误差（用于方向切换）
- `PidController_SetParams(pid, params)` — 运行时更新参数，复位积分
- `PidController_Step(pid, target, actual, dt_s)` — 基础 PID 步进（对称输出限幅）
- `PidController_StepLimited(pid, target, actual, dt_s, actuator_limit_direction)` — 带执行器方向感知的抗饱和（方向反转时冻结积分）
- `PidController_StepBounded(pid, target, actual, dt_s, actuator_dir, out_min, out_max)` — 非对称输出边界

**抗积分饱和策略：**
1. 积分限幅：|integral| ≤ integral_limit
2. 条件冻结：输出超出边界且误差方向继续超出时，停积积分
3. 执行器饱和感知：actuator 在误差推方向已达极限时，冻结积分

**默认 PID 参数（control_config.h / bsp_config.h）：**

| 电机 | Kp | Ki | Kd | 积分上限 |
|------|-----|-----|-----|---------|
| M1 | 8.0 | 0.3 | 0.1 | 60.0 |
| M2 | 8.0 | 0.3 | 0.1 | 60.0 |
| M3 | 8.0 | 0.3 | 0.1 | 60.0 |
| M4 | 0.5 | 0.0 | 0.0 | 0.0（纯 P） |

### 2.4 速度斜坡

**配置（control_config.h / bsp_config.h / param_model_t）：**
- `CHASSIS_SPEED_RAMP_MPS2` — 线速度斜坡，默认 0.5 m/s²
- `CHASSIS_ANGULAR_RAMP_RPS2` — 角速度斜坡，默认 2.0 rad/s²
- `CHASSIS_OPENLOOP_FULL_MPS` — 开环全速映射，默认 0.5 m/s

**实现（ChassisService_Step 内部）：**
- 左/右目标速度分别应用对称斜坡（加速/减速同斜率）
- 斜坡通过当前速度向目标速度渐进逼近，每个控制周期增量 = `ramp_rate × dt_s`
- 方向反转时：先降到零 → 反转制动 → 相位切换 → 反向加速
- PID 误差输入使用斜坡后的目标速度（不是原始目标）

### 2.5 直行补偿

**模块：** `App/chassis/straight_controller.c`
**头文件：** `App/chassis/straight_controller.h`

**四层补偿叠加（按优先级，全部以差速方式注入：左轮 −correction，右轮 +correction）：**

1. **方向性 trim（前馈）**：按前进/后退分别对 0.15/0.30m/s 两点线性插值（`straight_trim_forward/reverse_015/030_mps`），补偿万向轮不对称和机械偏差。
2. **轮速交叉耦合**：`wheel_coupling_gain × (vL_actual − vR_actual)`，利用左右实际速度差反馈纠正，纠正同向偏斜。
3. **gyro-z 航向 PI**：对陀螺 Z 轴角速度直接积分累积航向偏差（不使用 Mahony yaw），`heading_kp × error + heading_ki × ∫error`（积分限幅 `heading_integral_limit_deg_s`）。
4. **饱和降级**：PWM≥850‰ 或电流限制时冻结 PI 积分、减速对侧而非继续抬高弱侧（`straight_derated=1`）；IMU 不可用时航向 PI 退化为仅 trim+coupling（`straight_heading_degraded=1`）。

**激活/复位条件**：`angular_z=0` 且 `linear_x≠0` 时自动激活；换向、停车、非零角速度、控制源变更或 motion-revoke generation 变化时复位所有累积状态。

**纠偏总限幅**：`min(25% × base_speed, 0.075m/s)`。换向后累计前 0.30m 万向轮翻转路程（`straight_in_transition`），全程持续控制，仅遥测标记。

**补偿适用范围**：`straight_max_speed_mps` 只限制直行补偿，不限制底盘基础速度。超过该范围时左右轮保持原始目标、清空航向积分，并报告 `straight_out_of_range=1`。

**状态输出：** 方向、transition 路程、trim、轮速修正、heading error/积分/修正、总修正、限幅与降速状态均进入 `chassis_control_state_t`，由 CSV/JSON 同源输出。

**可调参数（param_model_t）：**
- `straight_wheel_coupling_gain` — 轮速耦合增益
- `straight_heading_kp` — 航向保持 P 增益
- 四个 `straight_trim_*` — 前进/后退 0.15/0.30m/s 前馈
- `straight_heading_ki` / `straight_heading_integral_limit_deg_s` — PI 积分参数
- `straight_max_speed_mps` — trim、轮速耦合和航向 PI 的最大适用速度
- `straight_heading_hold_enabled` — 航向保持总开关

### 2.6 电机输出逻辑

**模块：** `BSP/motor/motor_ph_en_mapper.c`
**头文件：** `BSP/motor/motor_ph_en_mapper.h`

**功能：** 将有符号 permille（-900~+900）分解为（绝对值 permille + 方向位）

```c
typedef struct {
    int16_t en_permille;   // 0~900（始终为正）
    uint8_t phase_high;    // 1=正转(forward), 0=反转(reverse)
} motor_output_phase_enable_t;

MotorPhEnMapper_ResolvePhaseEnable(signed_permille)
    → 正数: phase_high=1, en_permille=绝对值
    → 负数: phase_high=0, en_permille=绝对值
```

### 2.7 电机驱动（DRV8874）

**模块：** `BSP/motor/motor_driver.c`
**头文件：** `BSP/motor/motor_driver.h`

**硬件接口：**
- PWM：TIM1 四通道（CH1/CH2/CH3/CH4）→ DRV8874 EN/IN1
- 方向：GPIO PH 引脚（PC7/PC7/PC8/PC9）→ DRV8874 PH/IN2
- 故障检测：四路 nFAULT 开漏 → GPIO 输入
- 全局休眠：DRV_SLEEP_ALL (PE7)
- Break 输入：TIM1_BKIN (PE15) + TIM8_BKIN (PA6)

**方向反转状态机（6 阶段）：**
```
IDLE_BRAKE → RAMP_DOWN → REVERSE_BRAKE → PH_SETTLE → RAMP_UP → RUN
```
- RAMP_DOWN：PWM 每周期递减 25 步（快降）
- REVERSE_BRAKE：低侧制动，保持 ≥2 个周期，等速度 <0.02m/s 或超时
- PH_SETTLE：各电机独立切换相位；切换当轮保持 EN=0，下一控制周期再恢复 PWM
- RAMP_UP：PWM 每周期递增 15 步（慢升）

**关键常量（bsp_config.h）：**
| 常量 | 值 | 说明 |
|------|-----|------|
| `MOTOR_PWM_RISE_STEP_PER_CYCLE` | 15 | PWM 上升步长 |
| `MOTOR_PWM_FALL_STEP_PER_CYCLE` | 25 | PWM 下降步长（更快） |
| `MOTOR_REVERSE_BRAKE_CYCLES` | 2 | 最小反向制动周期 |
| `MOTOR_MAX_PERMILLE` | 900 | 最大输出 permille |
| `MOTOR_STOP_MODE_LOW_SIDE_BRAKE` | — | 停止模式（低侧制动） |
| `MOTOR_STOP_MODE_COAST` | — | 停止模式（惯性滑行） |

**启动资格验证（MotorDriver_Init）：**
1. 唤醒 DRV_SLEEP_ALL
2. 确认 BKIN 输入为高（无外部故障）
3. 确认所有 nFAULT 为高（无驱动故障）
4. 记录启动前 BIF/MOE 状态用于诊断
5. 使能 TIM1 MOE（主输出使能）

**函数：**
- `MotorDriver_Init()` — 启动资格验证 + MOE 使能
- `MotorDriver_SetPermille(motor, permille)` — 设置单电机目标（-900~900），含方向状态机
- `MotorDriver_SetSidePermille(side, permille)` — 设置整侧电机
- `MotorDriver_Stop(motor, mode)` — 停止单电机
- `MotorDriver_StopSide(side, mode)` — 停止整侧
- `MotorDriver_StopAll(mode)` — 停止全部
- `MotorDriver_UpdateFaults()` — 读取 nFAULT + Break 状态
- `MotorDriver_HasFault()` — 是否有活跃故障
- `MotorDriver_OnTim1BreakFromIsr()` — ISR 中锁存 Break 事件
- `MotorDriver_ClearBreakLatch()` — 清除 Break 锁存（需硬件 BKIN 已释放 + PWM/CCR 全部为零）
- `MotorDriver_GetState(*state)` — 线程安全状态快照

**状态字段：**
- `fault_active[4]` — 每电机 nFAULT 引脚状态
- `output_permille[4]` — 当前输出 permille
- `current_ph_dir[4]` — 当前相位的方向
- `phase[4]` — 每电机方向状态机阶段
- `tim1_moe_active` / `tim1_break_latched` / `tim1_break_count` — TIM1 Break 状态
- `tim8_break_flag` / `tim8_break_count` — TIM8 冗余诊断
- `startup_qualified` / `startup_pre_wake_bif` / `startup_bkin_high` / `startup_nfault_high_mask` — 启动诊断
- `break_origin` — Break 来源（BKIN / 软件 / 未知）
- `fault_edge_count[4]` / `fault_last_change_ms[4]` — nFAULT 边沿计数与时间戳

### 2.8 ChassisService 主控制循环

**模块：** `Service/chassis/chassis_service.c`（~1043 行）
**头文件：** `Service/chassis/chassis_service.h`

**数据结构：**
```c
typedef struct {
    // 每电机（M1-M4）
    float target_mps[4];           // 最终目标速度
    float requested_mps[4];       // 斜坡后的请求速度
    float actual_mps[4];          // 编码器实测速度
    float error_mps[4];           // 速度误差
    int16_t output_permille[4];   // PID 输出 permille
    uint8_t speed_valid[4];       // 速度是否有效
    uint8_t pid_active[4];        // PID 是否活跃
    uint8_t feedback_lost[4];     // 反馈丢失标记
    uint8_t current_limited[4];   // 是否被电流限制

    // 左右聚合
    float left_target, right_target;
    float left_requested, right_requested;
    float left_actual, right_actual;

    // 直行补偿（14 字段全量观测）
    uint8_t straight_active;
    int8_t straight_direction;
    uint8_t straight_in_transition;
    uint8_t straight_heading_degraded;
    uint8_t straight_derated;
    uint8_t straight_out_of_range;
    float straight_trim_mps;
    float straight_wheel_correction_mps;
    float straight_heading_error_deg;
    float straight_heading_integral_deg_s;
    float straight_heading_correction_mps;
    float straight_total_correction_mps;
    float straight_transition_distance_m;

    // 全局
    uint8_t output_enabled;       // PWM 输出总开关
    uint8_t pwm_saturated;        // 任一侧 PWM≥850‰
    uint8_t control_source;       // 当前活跃控制源
} chassis_control_state_t;
```

**ChassisService_Step(now_ms) 每 10ms 执行流程：**

1. **重入防护：** 设置 `control_step_active` 标记，防止维护锁期间重入
2. **刷新参数：** 检测 `ParamService` generation 变化，热重载 PID 参数
3. **故障更新：** 读取 `MotorDriver_UpdateFaults()` 检查 nFAULT / Break
4. **编码器更新：** 读取 `WheelEncoderDriver_Update()` 获取各轮速度
5. **安全检查：**
   - 维护锁激活 → 清除输出，等待解锁
   - ESTOP/Fault-Stop → 紧急停止
6. **测试模式：** 处理开环/raw 指令（需维护锁授权，400ms 租约）
7. **获取命令：** `ControlService_GetCommand()` → 最高优先级活跃命令
8. **差速分解：** `DifferentialDriveKinematics_ResolveDifferentialTargets()` → 左右目标速度
9. **速度斜坡：** 带方向感知的增量斜坡（加速/减速同斜率）
10. **直行补偿：** `StraightController_Step()` 注入方向 trim、轮速与 gyro PI 修正
11. **轮速等比缩放：** 一侧超限时等比压缩另一侧
12. **PID 步进：** 每电机 `PidController_StepLimited`（带执行器方向感知）
13. **电流保护：** `CurrentGuard_ApplyMotorLimit()` 对 PID 输出做限幅
14. **反馈丢失检测：** 任一启用编码器无效 + 非零请求 150ms 内无运动 → 整车锁停
15. **PWM 输出：** `MotorDriver_SetPermille()` 输出最终 permille

**关键常量：**
| 常量 | 值 | 说明 |
|------|-----|------|
| `CHASSIS_CONTROL_PERIOD_MS` | 10 | 控制周期 |
| `CHASSIS_MAX_LINEAR_MPS` | 0.5 | 最大线速度 |
| `CHASSIS_MAX_ANGULAR_RPS` | 10.0 | 最大角速度 |
| `CHASSIS_ENCODER_FEEDBACK_TIMEOUT_MS` | 150 | 反馈丢失检测超时 |
| `CHASSIS_FEEDBACK_LOSS_DEBOUNCE_CYCLES` | 50 | 反馈丢失去抖周期数 |

**函数：**
- `ChassisService_Init()` — 初始化全部子模块
- `ChassisService_Step(now_ms)` — 主控制循环
- `ChassisService_EmergencyStop()` — 紧急停止（复位 PID、取消测试模式、低侧制动）
- `ChassisService_OpenLoopTest(left_permille, right_permille)` — 侧开环测试（400ms 租约）
- `ChassisService_RawInputTest(lf, lr, rf, rr)` — 侧 raw 信号测试
- `ChassisService_RawMotorInputTest(motor, fwd, rev)` — 单电机 raw 测试
- `ChassisService_CancelTestMode()` — ISR 安全取消测试
- `ChassisService_ResolveSideTargets(linear_x, angular_z, *left, *right)` — 公开的差速分解
- `ChassisService_GetState(*state)` — 线程安全状态快照
- `ChassisService_IsStepActive()` — 重入检查（供维护锁使用）

### 2.9 任务入口与初始化

**模块：** `App/tasks/`
**头文件：** `App/tasks/app_tasks.h`

**App_InitHardware() 初始化序列（37 步）：**
1. 捕获复位原因（ResetReason）
2. 复位任务计时系统
3. 从 Flash 加载持久化参数
4. 初始化 MotorDriver（启动资格验证 + MOE）
5. 初始化 WheelEncoderDriver（四路编码器定时器）
6. 初始化 PowerAdcDriver（DMA + TIM8 触发）
7. 初始化 IMU BMI270（SPI + 配置表上传）
8. 初始化 IMU 校准门控
9. 应用已保存的 IMU 校准数据
10. 初始化 LED
11. 初始化 SafetyService
12. 初始化 ChassisService（含 PID、ControlService、CurrentGuard）
13. 初始化 UpperUart（USART3 DMA RX）
14. 初始化 LineSensorDriver（USART4 DMA RX）
15. 初始化 LineControl
16. 初始化 Ps2Control
17. 初始化 Esp12fComm
18. 初始化 Esp12fFlashBridge
19. 初始化调试控制台
20. 运行 POST 上电自检
21. 初始化 OLED UI
22. ...（配置调试台命令表、设置初始日志模式等）

### 2.10 任务计时与心跳

**模块：** `App/chassis/chassis_task_timing.c`
**头文件：** `App/chassis/chassis_task_timing.h`

**每个任务的心跳与超时追踪：**

```c
typedef enum {
    CHASSIS_TASK_SAFETY, CHASSIS_TASK_MOTOR, CHASSIS_TASK_RPI,
    CHASSIS_TASK_IMU, CHASSIS_TASK_LINE, CHASSIS_TASK_ESP,
    CHASSIS_TASK_PS2, CHASSIS_TASK_LED, CHASSIS_TASK_OLED,
    CHASSIS_TASK_COUNT
} chassis_task_timing_id_t;
```

**函数：**
- `ChassisTaskTiming_DelayUntil(task, *next_wake_ms, period_ms)` — 记录心跳 + missed-deadline 检测 + `osDelayUntil`
- `ChassisTaskTiming_Heartbeat(task, now_ms)` — 仅记录心跳
- `ChassisTaskTiming_UpdateTimeouts(now_ms)` — 检测所有任务的超时（阈值 4x-8x 周期）
- `ChassisTaskTiming_GetTimeoutMask()` — 返回所有任务超时位掩码
- `ChassisTaskTiming_GetMissedCount(task)` — 返回 missed-deadline 计数

**超时阈值（相对于任务周期）：**
| 任务 | 超时倍率 | 实际超时 |
|------|---------|---------|
| safetyTask | 4x | 80ms |
| motorTask | 4x | 40ms |
| rpiCommTask | 8x | 40ms |
| imuTask | 8x | 80ms |
| lineTask | 8x | 40ms |
| espTask | 8x | 40ms |
| ps2Task | 4x | 80ms |
| ledTask | 4x | 200ms |
| oledTask | 4x | 400ms |

### 2.11 维护锁

**模块：** `App/chassis/chassis_maintenance.c`
**头文件：** `App/chassis/chassis_maintenance.h`

**目的：** 确保在参数修改/保存/校准操作前底盘完全静止，操作期间禁止任何运动命令。

**函数：**
- `ChassisMaintenance_Begin()` — 获取维护锁：
  1. 调用 `ControlService_BeginMaintenance()`（禁止所有新命令，释放旧命令）
  2. 检查 `ChassisService_IsStepActive()` 为假（不在控制步中间）
  3. 取消测试模式
  4. 执行紧急停止
  5. 验证所有电机 PWM=0 且速度 < 阈值
  6. 返回 `OK` / `BUSY` / `NOT_STATIONARY`
- `ChassisMaintenance_End()` — 释放维护锁（`ControlService_EndMaintenance()`），恢复命令接收

**使用场景：**
- `set save` / `set reset` 参数持久化
- IMU 自动校准保存
- `espflash on` flash bridge 激活
- `set motor_dir` / `set encoder_dir` 方向修改

---

## 三、控制源与仲裁

### 3.1 五级优先级仲裁

**模块：** `Service/control/control_service.c`
**头文件：** `Service/control/control_service.h`

**命令结构：**
```c
typedef struct {
    float linear_x;          // 线速度 (m/s)
    float angular_z;         // 角速度 (rad/s)
    uint8_t enable;          // 是否启用（enable=0 视为撤回）
    control_source_t source; // 来源标识
    uint32_t timestamp_ms;   // 命令时间戳
} chassis_cmd_t;
```

**控制源枚举：**
```c
typedef enum {
    CONTROL_SOURCE_NONE = 0,
    CONTROL_SOURCE_UPPER = 1,   // USART3 上位机 (RPI)
    CONTROL_SOURCE_PS2 = 2,     // PS2 手柄
    CONTROL_SOURCE_ESP12F = 3,  // ESP12F WiFi
    CONTROL_SOURCE_LINE = 4,    // 巡线传感器
    CONTROL_SOURCE_DEBUG = 5,   // USART1 调试台
} control_source_t;
```

**仲裁规则：**
1. 按优先级排序：UPPER > PS2 > ESP12F > LINE > DEBUG
2. 每源独立超时检测，过期自动失效
3. `enable=0` 的命令视为该源"让权"
4. 返回最高优先级有效（enable=1 + 未超时）的命令
5. 所有源都无效时返回 NONE（速度全零）

**超时配置（control_config.h / bsp_config.h）：**

| 控制源 | 超时 | 说明 |
|--------|------|------|
| UPPER | 200ms | USART3 上位机（高频命令流） |
| PS2 | 500ms | PS2 手柄（20Hz 周期） |
| ESP12F | 500ms | WiFi 遥控（含网络延迟） |
| LINE | 50ms | 巡线传感器（200Hz 高频） |
| DEBUG | 2000ms | 调试台（手动测试，长时间容忍） |

### 3.2 安全语义

**reject-and-stop 规则：**

以下情况命令被拒绝且进入 stop 状态（不静默夹紧，不继续运行）：
- `linear_x` 或 `angular_z` 为 NaN（`isnan()`）
- `linear_x` 或 `angular_z` 为 Inf（`isinf()`）
- `source` 超出合法范围
- `enable=0` 时直接清除该源
- 当前处于 ESTOP 状态
- 当前处于 Fault-Stop 状态
- 当前处于维护锁状态
- 运动撤销 generation 不匹配（旧命令不能恢复）
- 运动学参数无效（`wheel_radius_m ≤ 0` 或 `track_width_m ≤ 0`）

**命令提交 API：**
- `ControlService_SetCommand(*cmd)` → `ACCEPTED / REJECTED / REJECTED_AND_STOPPED`
- `ControlService_SetCommandForGeneration(*cmd, gen)` — 带 generation 校验的命令提交（防止竞态）
- `ControlService_GetCommand(*cmd, now_ms)` — 获取当前最高优先级活跃命令
- `ControlService_ClearCommand()` — 清除所有命令
- `ControlService_ClearSource(source)` — 清除特定源

**安全状态 API：**
- `ControlService_SetEmergencyStop(enabled)` — 设置/清除 ESTOP（清除所有命令）
- `ControlService_SetFaultStop(enabled)` — 设置/清除 Fault-Stop（清除所有命令）
- `ControlService_BeginMaintenance()` — 进入维护模式（清除命令 + 递增 revoke generation）
- `ControlService_EndMaintenance()` — 退出维护模式
- `ControlService_IsEmergencyStop()` / `ControlService_IsFaultStop()` / `ControlService_IsMaintenanceLocked()`
- `ControlService_GetMotionRevokeGeneration()` — 获取当前 generation（用于命令合法性校验）
- `ControlService_GetActiveSource()` — 返回当前活跃的控制源

### 3.3 紧急停止与故障停止

**ESTOP：**
- 由 `estop 1` 调试命令、上位机 ESTOP 帧、ESP12F 远程 ESTOP 触发
- 只能软件置位（`estop 1`），不可由 ESP 远程清除
- 触发后：所有命令被清除、ChassisService 执行紧急停止
- 清除：`estop 0` 调试命令

**Fault-Stop：**
- 由 SafetyService 自动触发（过流、DRV_FAULT、TIM_BREAK、编码器反馈丢失、电池临界）
- 与 ESTOP 独立，但效果相同（清除所有命令、紧急停止）
- 需 `clearfault` 命令手动清除（需故障源已消失）

### 3.4 PS2 手柄控制

**模块：** `Service/control/ps2_control_service.c`（~447 行）
**头文件：** `Service/control/ps2_control_service.h`
**硬件驱动：** `BSP/ps2/ps2_controller_driver.c`

**数据结构：**
```c
typedef struct {
    uint8_t online;              // 手柄在线
    uint8_t analog_mode;         // 模拟模式（红灯/绿灯）
    uint8_t cmd_dat_swapped;     // CMD/DAT 线序是否交换
    uint8_t drive_enabled;       // 驾驶使能
    // 按键状态
    uint8_t btn_select, btn_l3, btn_r3, btn_start;
    uint8_t btn_up, btn_right, btn_down, btn_left;
    uint8_t btn_l1, btn_l2, btn_r1, btn_r2;
    uint8_t btn_triangle, btn_circle, btn_cross, btn_square;
    // 摇杆（归一化 -1.0~1.0）
    float left_x, left_y, right_x, right_y;
    // 定角宏
    uint8_t macro_active, heading_active;
    uint8_t heading_gate_flags;
    float heading_target_deg, heading_accumulated_deg;
    // 运动命令
    float linear_x, angular_z;
    // 巡线
    uint8_t line_tracking_enabled;
    // 诊断
    uint32_t rx_ok_count, rx_fail_count;
} ps2_control_state_t;
```

**功能：**
- **摇杆归一化：** 中位 128，死区 ±18（约 ±14%），输出范围 -1.0~+1.0
- **D-Pad 支持：** 十字键映射为离散速度指令
- **三角形按钮：** 切换巡线模式（开/关）
- **L1/L2/R1/R2 定角宏：**
  - L1: +90° 右转四分之一圈（超时 6s）
  - R1: -90° 左转四分之一圈（超时 6s）
  - L2: +360° 右转一整圈（超时 20s）
  - R2: -360° 左转一整圈（超时 20s）
- **手动打断：** 摇杆输入超过 `PS2_MANUAL_CANCEL_THRESHOLD` (0.12) 时取消定角宏
- **IMU 门控：** 定角宏需要 IMU 在线 + 已校准 + 数据新鲜（<50ms）+ 无关键质量异常
- **巡线模式：** PS2 在巡线启用时显式让权（不提交运动命令），巡线关时持续提交零速（维持占用）
- **离线保护：** 连续 3 次读取失败后标记离线，取消所有宏

**关键常量（control_config.h / bsp_config.h）：**

| 常量 | 值 | 说明 |
|------|-----|------|
| `PS2_OFFLINE_FAIL_LIMIT` | 3 | 离线判定连续失败次数 |
| `PS2_AXIS_CENTER` | 128 | 摇杆中心值 |
| `PS2_AXIS_DEADZONE` | 18 | 摇杆死区 |
| `PS2_MANUAL_CANCEL_THRESHOLD` | 0.12 | 手动打断阈值 |
| `PS2_HEADING_QUARTER_TIMEOUT_MS` | 6000 | 90° 超时 |
| `PS2_HEADING_FULL_TIMEOUT_MS` | 20000 | 360° 超时 |
| `PS2_HEADING_IMU_FRESH_MS` | 50 | IMU 数据新鲜度阈值 |

**IMU 门控标志（阻止定角宏的条件）：**
- `PS2_HEADING_GATE_IMU_OFFLINE` — IMU 离线
- `PS2_HEADING_GATE_IMU_UNCALIBRATED` — 未校准
- `PS2_HEADING_GATE_IMU_STALE` — 数据超时（>50ms）
- `PS2_HEADING_GATE_IMU_QUALITY` — 存在关键质量异常

**关键 IMU 质量位掩码（以下任何一位置位则阻止定角宏）：**
`SPI_ERROR | INIT_FAILED | FIFO_OVERFLOW | TIMESTAMP_ERROR | GYRO_SATURATION | ATTITUDE_INVALID | PROFILE_MISMATCH`

**PS2 硬件协议（BSP/ps2/ps2_controller_driver.c）：**
- 4 线 GPIO bit-bang SPI（无硬件 SPI 外设）：PS2_DO(CMD)、PS2_DI(DAT)、PS2_CLK、PS2_CS(ATT)
- DWT 周期计数器精确时序：半周期 10µs → ~50kHz SPI 时钟
- 9 字节帧协议：TX `[0x01, 0x42, ...]` → RX `[0xFF, MODE, 0x5A, BTN1, BTN2, RX, RY, LX, LY]`
- 初始化流程：进入配置模式 → 设置模拟模式 → 使能振动 → 退出配置
- 握手验证：`RX[1] != 0xFF && RX[2] == 0x5A`
- 模拟模式判断：mode=0x73（红灯）或 mode=0x79（绿灯）
- 按钮为 active-high（取反）

### 3.5 相对航向控制（陀螺积分闭环）

**模块：** `App/control/relative_yaw_control.c`
**头文件：** `App/control/relative_yaw_control.h`

**核心思路：** 不用 Mahony yaw（因 360° 测试中丢失约 40%），改用陀螺 Z 轴角速度直接积分，误差 <1%。

```c
typedef struct {
    uint8_t active, settling;
    relative_yaw_end_reason_t end_reason;
    float target_delta_deg;        // 目标旋转角度
    float accumulated_delta_deg;   // 已累积的旋转量
    uint32_t start_ms, timeout_ms;
    uint32_t settle_start_ms;
    uint32_t last_update_ms;
} relative_yaw_control_t;
```

**结束原因枚举：**
- COMPLETED — 达到目标
- TIMEOUT — 超时
- IMU_INVALID — IMU 数据异常
- CONTROLLER_OFFLINE — 手柄离线
- MANUAL_OVERRIDE — 手动打断
- SAFETY_STOP — 安全停止

**函数：**
- `RelativeYawControl_Start(*ctrl, target_delta_deg, initial_yaw_deg, now_ms, timeout_ms)` — 启动旋转
- `RelativeYawControl_Update(*ctrl, yaw_deg, yaw_rate_dps, now_ms, *angular_z)` — 每周期更新：
  1. 积分 `accumulated += gyro_z_dps × dt`
  2. P 控制器 `angular_z = Kp × (target - accumulated) × DEG_TO_RAD`
  3. 输出限幅 `±RELATIVE_YAW_MAX_RPS` (1.5 rad/s)
  4. 收敛判断：|error| < 2.0° 且 |rate| < 5.0 dps 持续 100ms → COMPLETED
- `RelativeYawControl_Cancel(*ctrl, reason)` — 取消旋转

**参数：**
| 常量 | 值 | 说明 |
|------|-----|------|
| `RELATIVE_YAW_KP_PER_S` | 2.0 | P 增益 |
| `RELATIVE_YAW_MAX_RPS` | 1.5 | 最大角速度 |
| `RELATIVE_YAW_TOLERANCE_DEG` | 2.0 | 角度容差 |
| `RELATIVE_YAW_SETTLE_RATE_DPS` | 5.0 | 稳定速率阈值 |
| `RELATIVE_YAW_SETTLE_MS` | 100 | 稳定持续时间 |

### 3.6 巡线控制（PD）

**模块：** `Service/control/line_control_service.c`（~275 行）
**头文件：** `Service/control/line_control_service.h`

**算法：**
1. 读取 8 路传感器模拟量（通过 `line_threshold_raw[]` 和 `line_active_low` 做阈值判定）
2. 加权平均线位置（0.0~7.0，中心 = 3.5）
3. 误差 = 3.5 - 线位置（正=偏左，负=偏右）
4. PD 控制器：`angular_z = kp × error + kd × (error - prev_error)`
5. 角速度限幅：`±LINE_ANGULAR_MAX_RPS` (2.0 rad/s)
6. 自适应降速：`speed = line_speed_mps / (1 + slowdown_gain × |error|)`
7. 去抖：检测到线需连续 `line_detect_debounce_frames` 帧、丢线需连续 `line_lost_debounce_frames` 帧

**函数：**
- `LineControl_Init()` — 初始化标定状态和追踪状态
- `LineControl_Update()` — 主控制循环（由 lineTask 每 5ms 调用）：
  1. 检查安全条件/revoke generation（维护锁后旧 LINE enable 不恢复）
  2. 验证传感器数据新鲜度（50ms 超时）
  3. 执行加权平均 + PD + 去抖 + 自适应降速
  4. 通过 `ControlService_SetCommandForGeneration` 提交
- `LineControl_Enable(enable)` — 启用/停用（带 generation 捕获）
- `LineControl_IsEnabled()` / `LineControl_GetState(*state)` — 状态查询

**可调参数（param_model_t）：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `line_kp` | 2.5 | P 增益 |
| `line_kd` | 0.1 | D 增益 |
| `line_speed_mps` | 0.15 | 基准线速度 |
| `line_slowdown_gain` | 0.0 | 自适应降速增益 |
| `line_threshold_raw[8]` | 500 | 每通道阈值 |
| `line_active_low` | 0 | 0=白底黑线, 1=黑底白线 |
| `line_detect_debounce_frames` | 4 | 检测去抖帧数 |
| `line_lost_debounce_frames` | 10 | 丢线去抖帧数 |

**观测字段（line_control_state_t）：**
- `line_position` / `error` / `error_derivative` — 位置与误差
- `detected_count` — 检测到的线传感器数
- `sensor_state[8]` — 每通道二值状态
- `sensor_raw[8]` — 每通道原始模拟量
- `tracking_active` / `globally_enabled` / `active_low`
- `output_saturated` / `lost_reason`

### 3.7 巡线双表面自动标定

**模块：** `App/control/line_calibration.c`
**头文件：** `App/control/line_calibration.h`

**流程：**
1. `linecal floor <N>` — 在地面采集 N 个样本
2. `linecal line <N>` — 在线上采集 N 个样本
3. `linecal apply` — 计算每通道均值中点作为阈值，验证分离度 ≥ 50（`LINE_CALIBRATION_MIN_SEPARATION_RAW`），确定 `active_low` 并仅应用到 RAM
4. `linecal show` — 查看当前标定结果
5. `linecal cancel` — 取消标定

PS2 双表面自动标定完成后同样只应用到 RAM；需要掉电保存时显式执行 `set save`。

**数据结构：**
```c
typedef struct {
    uint32_t sum[8], min[8], max[8];  // 每通道统计
    uint16_t count[8];
    uint16_t target_samples;
    uint8_t collecting, surface;
    uint8_t ready_mask;   // bit0=地面完成, bit1=线完成
    uint8_t fail_mask;    // 分离度不足的通道
} line_calibration_t;
```

**采样范围：** 4-2000 样本
**分离度要求：** 地面与线均值差 ≥ 50
**极性判定：** 比较地面总均值 vs 线总均值，地面更高 → `active_low=1`

---

## 四、传感器与外设驱动

### 4.1 编码器系统

**模块：** `BSP/encoder/wheel_encoder_driver.c` + `BSP/encoder/encoder_math.c`
**头文件：** `BSP/encoder/wheel_encoder_driver.h` + `BSP/encoder/encoder_math.h`

**硬件配置：**

| 电机 | 定时器 | 编码模式 | PPR | 倍频 | 减速比 | 每圈计数 |
|------|--------|---------|-----|------|--------|---------|
| M1 | TIM2 | 四倍频 | 11 | ×4 | 56 | 2464 |
| M2 | TIM4 | 四倍频 | 11 | ×4 | 56 | 2464 |
| M3 | TIM3 | 四倍频 | 11 | ×4 | 56 | 2464 |
| M4 | TIM5 | 四倍频 | 11 | ×4 | 56 | 2464 |

**数据结构：**
```c
typedef struct {
    int32_t count[4];              // 累积编码器计数
    int32_t delta[4];              // 本周期增量
    float speed_mps[4];            // 速度 (m/s)
    uint8_t speed_valid[4];        // 速度有效性
    uint8_t reject_streak[4];      // 连续拒绝计数
    uint16_t window_rebuild_count[4]; // 窗口重建次数
    uint16_t anomaly_count[4];     // 异常计数
    uint8_t consecutive_anomalies[4];

    // 左右聚合
    int32_t left_count, right_count;
    int32_t left_delta, right_delta;
    float left_speed_mps, right_speed_mps;
    uint8_t left_speed_valid, right_speed_valid;
    uint8_t speed_valid_all;

    // 同侧一致性
    uint32_t side_consistency_flags; // ENCODER_SIDE_CONSISTENCY_*
    uint32_t last_update_ms;
} wheel_encoder_state_t;
```

**侧一致性检查标志（6 个）：**
- `ENCODER_SIDE_CONSISTENCY_LEFT_SPEED` — 左侧多电机速度一致
- `ENCODER_SIDE_CONSISTENCY_LEFT_COUNT` — 左侧多电机计数一致
- `ENCODER_SIDE_CONSISTENCY_LEFT_DIRECTION` — 左侧多电机方向一致
- `ENCODER_SIDE_CONSISTENCY_RIGHT_SPEED` / `_COUNT` / `_DIRECTION` — 同右

**关键函数：**
- `WheelEncoderDriver_Init()` — 启动所有编码器定时器
- `WheelEncoderDriver_Update(now_ms)` — 读取计数器、计算增量/速度、异常检测、侧一致性交叉检查
- `WheelEncoderDriver_GetState(*state)` — 原子状态快照
- `WheelEncoderDriver_GetCountsPerRev()` — 返回 2464 (counts/rev)
- `WheelEncoderDriver_GetMotorSpeedMps(motor)` — 单电机速度查询
- `WheelEncoderDriver_DiffCount(now, last, period)` — 16/32 位计数器回绕安全差分

**编码器数学（encoder_math.c）：**
- `EncoderMath_DiffCount(now, last, period)` — 回绕安全差分
- `EncoderMath_CountDeltaSpeedMps(delta, dt_ms, counts_per_rev, wheel_radius_m)` — 增量转 m/s
- `EncoderMath_SpeedWindowReset(window)` — 清空滑动窗口
- `EncoderMath_SpeedWindowPush(window, delta, dt_ms)` — 推入样本
- `EncoderMath_DeltaAccepted(...)` — 尖峰拒绝（当前 vs 窗口历史速度对比）
- `EncoderMath_RecordDeltaOrRebuild(...)` — 主滤波器：接受/拒绝/窗口重建

**速度滤波器：**
- 滑动窗口大小：`CHASSIS_ENCODER_SPEED_WINDOW_SAMPLES` (5)
- 运行窗口求和（避免每次重新计算）
- 尖峰拒绝：当前增量在 3x 窗口平均速度的上/下界外 → 拒绝
- 连续拒绝超限后自动重建窗口

**发布临界区（P0 安全要求）：**
- 编码器状态发布时临界区最坏时间 <20µs（需逻辑分析仪验证）
- 使用局部 next-state 计算 + GPIO 包围最终状态发布

### 4.2 ADC 电流与电压采样

**模块：** `BSP/adc/power_adc_driver.c`
**头文件：** `BSP/adc/power_adc_driver.h`

**硬件配置：**
- **ADC：** ADC1，5 通道扫描（4 路电机 IPROPI 电流 + 1 路电池分压）
- **触发：** TIM8 TRGO，2kHz
- **传输：** DMA2 Stream0 循环模式

**数据结构：**
```c
typedef struct {
    uint16_t raw_current[4];          // M1-M4 原始 ADC
    uint16_t current_zero_raw[4];     // 零点参考值
    uint16_t raw_battery;
    float current_a[4];               // EMA 滤波后的电流
    float current_mean_a[4];          // 修剪均值
    float current_rms_a[4];           // RMS
    float current_peak_a[4];          // 峰值
    float current_signed_mean_a[4];   // 有符号均值（保留方向）
    float current_noise_a[4];         // 噪声（标准差）
    float battery_voltage;
    float left_current_a, right_current_a; // 侧平均
    uint32_t valid_flags;             // POWER_ADC_DRIVER_VALID_SAMPLES_READY
    uint32_t invalid_reason_flags;    // POWER_ADC_DRIVER_INVALID_*（8 种）
    uint32_t current_quality_flags[4]; // 零点不稳定/窗口太小/窗口尖峰
    // 诊断计数器
    uint32_t dma_half_count, dma_full_count;
    uint32_t dma_error_count, zero_cal_count;
} power_adc_driver_state_t;
```

**统计数据（每电机）：**
- **修剪均值（Trimmed Mean）：** 剔除最小/最大样本后的均值
- **RMS：** 均方根（反映有效值）
- **峰值：** 窗口内最大值
- **有符号均值：** 保留电机方向符号的均值
- **噪声：** 有符号样本的标准差

**低通滤波：** EMA α=`MOTOR_CURRENT_FILTER_ALPHA`（默认 0.25）

**关键函数：**
- `PowerAdcDriver_Init()` — 启动 ADC DMA + TIM8 触发
- `PowerAdcDriver_Update()` — ISR 安全原子快照 + 统计计算（由 safetyTask 调用）
- `PowerAdcDriver_GetState(*state)` — 原子状态读取
- `PowerAdcDriver_RequestCurrentZeroCalibration()` — 启动零点校准
- `PowerAdcDriver_ApplyCurrentZeroCalibration(zero_raw[4])` — 强设零点参考
- `PowerAdcDriver_SetCurrentZeroStationary(stationary)` — 静止门控（零点累积需底盘停稳）
- `HAL_ADC_ConvCpltCallback()` / `HAL_ADC_ErrorCallback()` — ISR 处理

**有效性标志：**
- `POWER_ADC_DRIVER_VALID_SAMPLES_READY` — DMA 有足够样本
- `POWER_ADC_DRIVER_VALID_CURRENT_ZERO_READY` — 零点校准完成

**无效原因（8 种）：**
`POWER_ADC_DRIVER_INVALID_NO_SAMPLES` / `_DMA_ERROR` / `_ZERO_NOT_CALIBRATED` / `_ZERO_STALE` / `_ZERO_UNSTABLE` / `_WINDOW_EMPTY` / `_ADC_OVERVOLTAGE` / `_TIMEOUT`

**质量标志（3 种）：**
`POWER_ADC_DRIVER_QUALITY_ZERO_UNSTABLE` / `_WINDOW_TOO_SMALL` / `_WINDOW_SPIKE`

### 4.3 BMI270 IMU

**模块：** `BSP/imu/bmi270_driver.c`（主驱动）、`imu_bmi270_calibration.c`（校准）、`imu_bmi270_config.c`（配置表）、`imu_bmi270_fifo.c`（FIFO 解析）、`bmi270_driver_math.c`（数学/融合）、`imu_bmi270_profile.c`（运行模式）、`imu_bmi270_time.c`（时间处理）

**硬件接口：**
- SPI2，自定义 CS (IMU_CS)
- INT1 数据就绪中断（EXTI0, PE0）
- 支持 bit-bang 回退诊断（MISO pull 检测）

**配置表（imu_bmi270_config.c）：**
- Bosch SensorAPI v2.86.1 官方配置文件
- 8192 字节，写入 BMI270 内部 feature engine RAM
- ODR: 100Hz（acc + gyro）
- 量程：加速度 ±2g (LSB=16384/G)，陀螺 ±500dps (LSB=65.6/dps)

**三种运行 Profile（imu_bmi270_profile.c）：**

| Profile | 加速度 | 陀螺 | FIFO 水位 | fifo_downs | 说明 |
|---------|--------|------|-----------|-----------|------|
| NORMAL | 100Hz, 2g | 100Hz, 500dps | 64 bytes | 0x88 (filtered) | 基线 |
| PERFORMANCE | 100Hz, 2g | 100Hz, 500dps | 96 bytes | 0x88 (filtered) | 默认 |
| DEBUG | 100Hz, 2g | 100Hz, 500dps | 32 bytes | 0x00 (unfiltered) | 原始数据 |

**FIFO 解析（imu_bmi270_fifo.c）：**
- 帧头类型：`0x84` (Accel 6B) / `0x88` (Gyro 6B) / `0x8C` (Accel+Gyro 12B) / `0x44` (SensorTime 3B) / `0x40` (Skip) / `0x48` (Config Change) / `0x80` (Overread)
- 每批最多解析 8 个样本
- 跟踪跳过帧计数和 overflow 标志

**传感器时间（imu_bmi270_time.c）：**
- 24-bit 微秒计数器 @ 25.6kHz
- `IMU_BMI270_SENSOR_TIME_TICKS_PER_SEC` = 25600.0
- 24-bit 溢出安全差分计算

**核心数据结构（187 字节）：**
```c
typedef struct {
    uint8_t enabled, online, chip_id, last_error, init_state, profile;
    uint32_t error_count, last_update_ms, sensor_time;
    uint8_t sensor_time_valid;
    uint32_t sample_count, drdy_count, poll_fallback_count;
    int16_t accel_raw[3], gyro_raw[3];
    float accel_g[3], body_accel_g[3], ros_accel_g[3];
    float gyro_dps[3], gyro_bias_dps[3], gyro_corrected_dps[3];
    float gyro_filtered_dps[3], body_gyro_dps[3], ros_gyro_dps[3];
    float quaternion[4];
    float roll_deg, pitch_deg, yaw_deg;
    float temperature_c;
    uint8_t temperature_valid;
    float accel_correction_weight;        // Mahony 加速度校正权重
    uint32_t quality_flags;              // 当前质量标志
    uint32_t quality_latched_flags;      // 锁存质量标志
    // 诊断计数器
    uint32_t fifo_overflow_count, timestamp_error_count;
    uint32_t gyro_saturation_count, accel_anomaly_count;
    uint32_t poll_fallback_count;
    uint32_t init_attempt_count, spi_error_count;
    // 陀螺校准状态
    imu_bmi270_gyro_cal_state_t gyro_cal;
} bmi270_driver_state_t;
```

**质量标志（10 种）：**

| 标志 | 含义 |
|------|------|
| `IMU_BMI270_QUALITY_SPI_ERROR` | SPI 通信错误 |
| `IMU_BMI270_QUALITY_INIT_FAILED` | 初始化失败 |
| `IMU_BMI270_QUALITY_FIFO_OVERFLOW` | FIFO 溢出 |
| `IMU_BMI270_QUALITY_TIMESTAMP_ERROR` | 时间戳异常 |
| `IMU_BMI270_QUALITY_GYRO_SATURATION` | 陀螺饱和（>490dps） |
| `IMU_BMI270_QUALITY_ACCEL_ANOMALY` | 加速度异常 |
| `IMU_BMI270_QUALITY_ATTITUDE_INVALID` | 姿态无效 |
| `IMU_BMI270_QUALITY_POLL_FALLBACK` | 回退到轮询模式 |
| `IMU_BMI270_QUALITY_PROFILE_MISMATCH` | 配置 profile 不匹配 |
| `IMU_BMI270_QUALITY_TEMPERATURE_INVALID` | 温度读数无效 |

**关键函数（主驱动）：**
- `Bmi270Driver_Init()` — 清零状态，加载默认值
- `Bmi270Driver_SetEnabled(enabled)` / `Bmi270Driver_SetProfile(profile)`
- `Bmi270Driver_ProbeNow()` — 读 chip ID（期望 0x24）
- `Bmi270Driver_ConfigNow()` — 全初始化：复位 → 加载 8192 字节配置文件 → 验证 profile 寄存器回读
- `Bmi270Driver_Update()` — 主更新：优先读 FIFO，失败则回退直接读寄存器
- `Bmi270Driver_OnDataReadyFromIsr()` — ISR 递增 DRDY 计数（通知 imuTask）
- `Bmi270Driver_Diagnose(*diag)` — SPI 健康检查 + bit-bang MISO pull 检测
- `Bmi270Driver_CalibrateGyro(samples, delay_ms)` — 手动陀螺校准请求
- `Bmi270Driver_ServiceCalibration(now_ms, stationary)` — 自动校准状态机服务
- `Bmi270Driver_ClearCalibration()` / `Bmi270Driver_ApplyGyroBias(bias_dps)`
- `Bmi270Driver_ApplyCalibration(*cal)` / `Bmi270Driver_GetCalibration(*cal)`
- `Bmi270Driver_GetState(*state)` — 原子读取
- `Bmi270Driver_RawFrameHasSignal(...)` — 全零检测（死传感器）

**Mahony AHRS 融合（bmi270_driver_math.c）：**
- 算法：Mahony 互补滤波器
- 输入：陀螺角速度 (dps) + 加速度 (g)
- 输出：四元数 (w,x,y,z) + roll/pitch/yaw (degrees)
- 默认参数：Kp=1.0, Ki=0.0（无积分偏置跟踪）
- 加速度校正权重自适应：
  - `|norm-1g| < 0.25`（0.75-1.25g）→ weight=1.0（全权重）
  - `|norm-1g| < 0.60`（0.40-1.80g）→ weight=0.1（降权，退化模式）
  - 超出范围 → weight=0.0（仅陀螺积分）
- 初始化：从加速度计方向确定初始 roll/pitch，yaw=0

**坐标映射：**
- 传感器坐标 → 车体坐标（3x3 旋转矩阵）
- 车体坐标 → ROS REP-103 坐标（当前为恒等映射）
- 上位机 IMU 输出使用车体坐标

**陀螺校准（imu_bmi270_calibration.c）：**

校准数据结构（FNV-1a 校验）：
```c
typedef struct {
    uint32_t version;
    float accel_bias_g[3], accel_scale[3];
    float gyro_bias_dps[3];
    float temperature_offset_c;
    float temperature_gyro_slope_dps_per_c[3];  // 温度补偿斜率
    float sensor_to_body[3][3];                  // 传感器→车体旋转矩阵
    uint32_t crc;  // FNV-1a
} imu_bmi270_calibration_t;
```

校准累积器状态机：`IDLE → COLLECTING → READY / FAIL_ABS / FAIL_SPAN`

关键参数：
| 参数 | 值 | 说明 |
|------|-----|------|
| `BMI270_GYRO_CAL_DEFAULT_SAMPLES` | 500 | 默认采样数 |
| `BMI270_GYRO_CAL_MAX_ABS_DPS` | 20.0 | 静止时最大角速度绝对值 |
| `BMI270_GYRO_CAL_STILL_SPAN_DPS` | 5.0 | 静止时角速度峰峰值上限 |
| 自动校准：启用/最多 5 次/启动延迟 1000ms/重试延迟 2000ms |

温度补偿：`bias_at_T = base_bias + slope × (T_current - T_offset)`（每个轴独立斜率）

**调试命令：**
- `imutest` — 快速 SPI 探测
- `imudiag` — 全 SPI 诊断（含 bit-bang 回退 + MISO pull 检测）
- `imuinit` — 重新初始化
- `imucal [N]` — 手动触发陀螺校准（默认 500 样本）
- `imucalclear` — 清除校准数据
- `imu 0|1` — 禁用/启用 IMU

### 4.4 IMU 自动校准门控

**模块：** `App/imu/imu_calibration_gate.c`
**头文件：** `App/imu/imu_calibration_gate.h`

**目的：** 自动检测底盘静止状态，触发后台陀螺零偏校准。

**判定条件（全部满足）：**
1. 所有启用电机 PWM = 0
2. 所有启用电机速度 < 0.02 m/s 且编码器有效
3. 加速度计范数在 [0.95, 1.05] g（非运动、非大幅度倾斜）
4. 滑动窗口（100 样本）内：
   - 加速度范数方差 < 0.0004 g²
   - 三轴陀螺方差均 < 0.25 dps²
5. 需累积满 100 样本（去抖）

**滑动窗口实现：**
- 环缓冲 `accel_norm_window[100]` + `gyro_window[3][100]`
- 运行均值和平方和（避免每次重算）
- 样本去重（按 IMU sample_count）

**关键阈值：**
| 常量 | 值 | 说明 |
|------|-----|------|
| `IMU_CALIBRATION_GATE_MAX_SPEED_MPS` | 0.02 | 最大允许轮速 |
| `IMU_CALIBRATION_GATE_ACCEL_MIN_G` | 0.95 | 加速度范数下限 |
| `IMU_CALIBRATION_GATE_ACCEL_MAX_G` | 1.05 | 加速度范数上限 |
| `IMU_CALIBRATION_GATE_ACCEL_VARIANCE_MAX_G2` | 0.0004 | 加速度方差上限 |
| `IMU_CALIBRATION_GATE_GYRO_VARIANCE_MAX_DPS2` | 0.25 | 陀螺方差上限 |
| `IMU_CALIBRATION_GATE_WINDOW_SAMPLES` | 100 | 滑动窗口大小 |

### 4.5 巡线传感器

**模块：** `BSP/line/line_sensor_driver.c`
**头文件：** `BSP/line/line_sensor_driver.h`

**硬件：** HiWonder 八通道红外巡线模块，USART4 + DMA 循环缓冲（128 字节）

**二进制帧协议：**
```
[0x55] [0xAA] [CMD] [DATA_LEN] [DATA...] [CHECKSUM]
```
- CMD=0x02：模拟量查询响应，DATA_LEN=16（8 通道 × 2 字节）
- CHECKSUM = `~sum(CMD + DATA_LEN + DATA[0..N-1])`（反码和）
- 传感器初始化：发送 `0x00`（手动模式），然后 `0x02`（触发模拟量查询）

**数据结构：**
```c
typedef struct {
    uint8_t state[8];        // 每通道 0/1（二值化结果）
    uint16_t analog[8];      // 每通道原始 ADC
    uint32_t timestamp_ms;   // 时间戳
    uint8_t valid;           // 数据有效标志
} line_sensor_data_t;

typedef struct {
    uint32_t rx_bytes, rx_frames;
    uint32_t overflow_count, rx_protocol_errors;
    uint32_t tx_frames, tx_busy_drops, tx_failures;
    uint32_t uart_errors, dma_restarts;
    uint16_t last_frame_len;
    uint8_t last_frame[32];
    uint8_t tx_busy;
} line_sensor_driver_state_t;
```

**关键函数：**
- `LineSensorDriver_Init()` — 启动 DMA RX
- `LineSensorDriver_Update()` — 处理 DMA 循环缓冲区，解析帧
- `LineSensorDriver_GetState(*state)` / `LineSensorDriver_GetSensorData(*data)` — 状态/数据查询
- `LineSensorDriver_InitSensor()` — 发送 0x00 进入手动模式
- `LineSensorDriver_RequestAnalog()` — 发送 0x02 触发模拟量查询
- `LineSensorDriver_OnTxCplt()` / `LineSensorDriver_OnUartError()` — UART 事件处理

**关键常量：**
| 常量 | 值 | 说明 |
|------|-----|------|
| `LINE_SENSOR_CHANNELS` | 8 | 通道数 |
| `LINE_SENSOR_FRAME_LEN` | 21 | 帧长 |
| `LINE_SENSOR_HEADER_0` | 0x55 | 帧头字节 0 |
| `LINE_SENSOR_HEADER_1` | 0xAA | 帧头字节 1 |
| `LINE_ANALOG_THRESHOLD` | 500 | 默认模拟量阈值 |

### 4.6 电池电压监测

**模块：** 集成在 `BSP/adc/power_adc_driver.c`、`Service/power_management/` 和 `Service/safety_management/` 中

**采样：**
- ADC1 通道（VBAT_SENSE, PC4）
- 电阻分压比配置在 `bsp_config.h`
- EMA 低通滤波

**3S 锂电池欠压状态机（safety_service.c）：**

| 阈值 | 值 | 行为 |
|------|-----|------|
| `BATTERY_LOW_WARN_V` | 10.5V | 触发 `SYSTEM_ERROR_LOW_BATTERY` 告警 + LED 低电量闪烁 |
| `BATTERY_LOW_CLEAR_V` | 11.0V | 清除低电量告警 |
| `BATTERY_CRITICAL_V` | 9.0V | 连续 500ms → 触发 `SYSTEM_ERROR_BATTERY_CRITICAL` + Fault-Stop |
| `BATTERY_RECOVER_V` | 9.6V | 连续 2s → 自动清除 `SYSTEM_ERROR_BATTERY_CRITICAL`（不清除其他锁存、不恢复旧运动） |

**欠压去抖：** 使用连续超阈值周期计数，防止瞬时噪声误触发

---

## 五、通信协议

### 5.1 Upper Protocol V2

**模块：** `Service/communication/internal/robot_link_protocol.c`
**头文件：** `Service/communication/internal/robot_link_protocol.h`

**帧格式：**
```
[0xA5] [0x5A] [CMD_LEN] [CMD] [PAYLOAD...] [CRC8]
  ↑       ↑       ↑       ↑        ↑          ↑
 Header (2B)   长度(1B)  命令(1B)  数据(0-99B)  校验(1B)
```
- 最大帧长：104 字节（2+1+1+99+1）
- CRC8：表驱动，256 项 LUT，多项式同 Maxim/Dallas 1-Wire

**命令集：**

| 命令 | 值 | 方向 | Payload | 说明 |
|------|-----|------|---------|------|
| SET_VELOCITY | 0x01 | → | 10B: float lx + float az + enable + mode | 设置目标速度 |
| ESTOP | 0x02 | → | 1B: 0/1 | 紧急停止（远程只能置位） |
| LINE_CTRL | 0x03 | → | 1B: 0/1 | 巡线启停 |
| CLEAR_FAULT | 0x04 | → | 1B: mask | 清除锁存故障 |
| STATUS | 0x81 | ← | 65B | 四电机完整状态上报 |
| DIAGNOSTIC | 0x82 | ← | 28B | 诊断帧（POST/ADC/任务/IMU） |
| IMU_STATUS | 0x83 | ← | 99B | IMU 数据帧 |

**STATUS 帧 (0x81) — 65 字节：**
- 电池电压 (mV, uint16)
- 每电机：速度 (mm/s, int16) / 编码器增量 (int16) / 编码器累计 (int32) / 电流 (mA, int16) / 目标速度 (mm/s, int16) / PWM (permille, int16)
- 错误标志 (uint32) / 状态/通信健康标志 (uint32) / 电机启用掩码 (uint8) / 速度有效掩码 (uint8) / 编码器异常掩码 (uint8)

**DIAGNOSTIC 帧 (0x82) — 28 字节（schema version 1）：**
- POST done / IMU 状态
- POST 错误标志 (uint8)
- ADC 无效原因 (uint8)
- 任务超时掩码 (uint16)
- IMU 质量标志 (uint8)
- 复位原因 (uint8)
- 运行时间 (uint32, 秒)

**IMU_STATUS 帧 (0x83) — 99 字节：**
- 加速度 (3×float, 车体坐标)
- 陀螺修正值 (3×float, 车体坐标)
- Euler 角 (3×float, roll/pitch/yaw)
- 四元数 (4×float, w/x/y/z)
- 时间戳 (uint32, ms) / 传感器时间 (uint32) / 样本计数 (uint32)
- 质量标志 + 7 个质量计数器
- 状态标志 (uint8) / 温度 (int16, 0.01°C)

**关键函数：**
- `UpperProtocol_Checksum8(*data, length)` — CRC8 计算
- `UpperProtocol_BuildFrame(cmd, *payload, len, *out, out_len)` — 构建完整帧
- `UpperProtocol_ParseVelocityPayload(...)` — 反序列化 10 字节速度命令
- `UpperProtocol_RemoteEstopSetRequested(...)` — 检查 1 字节 ESTOP payload
- `UpperProtocol_BuildStatusPayload(...)` — 序列化 65 字节状态帧
- `UpperProtocol_BuildImuStatusPayload(...)` — 序列化 99 字节 IMU 帧
- `UpperProtocol_BuildDiagnosticPayload(...)` — 序列化 28 字节诊断帧

### 5.2 USART3 上位机链路

**模块：** `Service/communication/host_communication_service.c`
**头文件：** `Service/communication/host_communication_service.h`

**硬件：** USART3 (PD8/PD9)，DMA 循环接收（128 字节环缓冲）

**RX 状态机：**
```
WAIT_HEAD0 → WAIT_HEAD1 → WAIT_LEN → WAIT_BODY → 帧完成 → CRC 校验 → 分发
```
- 帧间超时：100ms 空闲后重置解析状态
- 覆盖检测：缓冲区风险时重新同步
- CRC8 完整性校验

**TX 优先级队列（4 槽位）：**
| 优先级 | 帧类型 | 发送周期 |
|--------|--------|---------|
| 0（最高） | IMU_STATUS (0x83) | 每 20ms |
| 1 | DIAGNOSTIC (0x82) | 每 200ms |
| 2 | STATUS (0x81) | 每 50ms |

- 队列满时丢弃最旧的 IMU 帧（影响最小）
- TX 使用中断完成回调串联

**诊断统计：**
- `tx_frames` / `tx_busy_drops` — 发送统计
- `rx_checksum_errors` / `rx_timeout_resets` / `uart_errors` — 接收错误统计
- `rx_dma_half_count` / `rx_dma_full_count` — DMA 中断计数
- `rx_overwrite_count` / `rx_resync_restarts` — 覆盖/重新同步计数
- `last_valid_frame_ms` — 最近有效帧时间（供 OLED 在线检测）

**关键函数：**
- `HostCommunication_Init()` — 配置 DMA RX，重置解析状态
- `HostCommunication_Update()` — 主循环（200Hz）：轮询 DMA 环缓冲、驱动 RX 状态机、发送周期帧
- `HostCommunication_HandleFrame(cmd, *payload, len)` — 帧分发：SET_VELOCITY→ControlService, ESTOP, LINE_CTRL, CLEAR_FAULT
- `HostCommunication_OnDmaHalf()` / `_OnDmaFull()` / `_OnTxComplete()` / `_OnUartError()` — HAL 回调

### 5.3 ESP12F WiFi 通信

**模块：** `Service/communication/wireless_communication_service.c`
**头文件：** `Service/communication/wireless_communication_service.h`

**硬件：** USART2 (PD5/PD6)，中断驱动环缓冲接收

**与 Upper Protocol 共享帧格式**（相同的 0xA5 0x5A 帧头 + CRC8）

**数据结构：**
```c
typedef struct {
    uint32_t rx_frames, tx_frames;
    uint32_t tx_busy_drops;
    uint32_t rx_checksum_errors, rx_length_errors;
    uint32_t rx_overflow_errors, rx_timeout_resets;
    uint32_t uart_errors;
    uint32_t last_rx_timestamp_ms;
    uint8_t boot_mode_download;
} esp12f_comm_state_t;
```

**关键函数：**
- `WirelessCommunication_Init()` — 初始化环缓冲，启动 RX
- `WirelessCommunication_Update()` — 200Hz 主循环：跳过隔离/flash bridge 状态，帧间超时检测（100ms），轮询 RX 环，发送周期状态（100ms）和诊断（200ms）
- `WirelessCommunication_HandleFrame(cmd, *payload, len)` — 同 Upper 分发：SET_VELOCITY(source=ESP12F), ESTOP, LINE_CTRL, CLEAR_FAULT
- `WirelessCommunication_OnRxCplt()` — ISR 环缓冲入队
- `WirelessCommunication_ResetModule()` — GPIO 复位 ESP（5ms RST 脉冲）
- `WirelessCommunication_Isolate()` — 全隔离：禁用 USART2 IRQ/DMA，中止 UART，保持 ESP 复位
- `WirelessCommunication_SetDownloadMode(enabled)` — 控制 ESP IO0（烧录模式）
- `WirelessCommunication_GetState(*state)` — 诊断快照

### 5.4 ESP12F 通信透传桥

**编排模块：** `App/adapters/esp12f_flash_bridge.c`
**硬件模块：** `BSP/esp12f/esp12f_boot_control.c`、`BSP/transport/uart_bridge_transport.c`

**功能：** PC ↔ ESP8266 双向 UART 透传，用于固件烧录。

**数据通路：**
```
PC (USART1) ←→ [4096B 环缓冲] ←→ ESP8266 (USART2)
```

**双缓冲：** `pc_to_esp_ring[4096]` + `esp_to_pc_ring[4096]`
**TX 分块：** 128 字节/次
**空闲自动退出：** 30 秒无活动自动退出 bridge 模式

**ESP 启动模式控制：**
- **烧录模式：** EN→低 → IO0→低 → RST→低 → EN→高 → RST→高（IO0 采样为低 = download boot）
- **正常模式：** 同上但 IO0 保持高

**函数：**
- `Esp12fFlashBridge_Init()` — 初始化缓冲
- `Esp12fFlashBridge_Enable(download_mode)` — 获取维护锁，中止 UART，配置 ESP 启动模式
- `Esp12fFlashBridge_Disable()` — 正常启动 ESP，重启通信
- `Esp12fFlashBridge_Update(now_ms)` — 服务 TX + 空闲超时检查
- `Esp12fFlashBridge_IsActive()` / `Esp12fFlashBridge_GetIdleMs()` / `Esp12fFlashBridge_GetState()`
- `UartBridgeTransport_OnRx()` / `_OnTxComplete()` / `_OnError()` — BSP UART 事件处理

**调试命令：**
- `espflash on` — 进入烧录模式（自动保持维护锁）
- `espflash off` — 退出烧录模式
- `espflash status` — 查看 bridge 状态
- `espat on` — AT 指令透传（IO0=1）
- `espat off` — 退出 AT 模式
- `espreset` — 复位 ESP8266
- `espboot` — ESP 进入下载模式
- `espisolate` — 隔离 ESP（保持复位+禁用）

### 5.5 ESP12F Arduino 固件

**位置：** `firmware/esp12f/F407_ESP12F/F407_ESP12F.ino`

**平台：** ESP8266 (Core 3.1.2) + arduinoWebSockets 2.7.2

**功能模块：**

**首次配置：**
- 开 AP `F407_Chassis_Setup`（开放网络）
- Web 配置页：设置 8-63 字节密码
- 存储到 EEPROM：magic(F407) + version + length + password + CRC32
- 回读验证后重启

**正常运行：**
- AP `F407_Chassis`（WPA2 加密）
- HTTP Server（端口 80）：gzip 压缩控制页面 + `/status` JSON API + `/configure` POST
- WebSocket Server（端口 81）：实时遥测与控制

**WebSocket 安全模型（owner 租约）：**
- 客户端发送 `claim` 获取 owner 权
- 单 owner：新 claim 拒绝（需等旧 owner 超时）
- 心跳：200ms 间隔 `heartbeat`
- 租约超时：500ms 无心跳 → 释放 owner → 发送一次 neutral 命令
- Readonly 客户端：可查看遥测但不可控制
- 断开：自动释放 owner + neutral 命令

**WebSocket 命令：**
| 命令 | 说明 |
|------|------|
| `claim` | 申请 owner（仅 owner 可发控制命令） |
| `heartbeat` | 维持租约 |
| `vel` | 速度控制 (lx/az) |
| `stop` | 停止 |
| `estop` | 急停（**仅置位**，不可由 ESP 清除） |
| `line` | 巡线启停 |
| `clearfault` | 清除故障 |

**遥测广播：**
- 周期：每 100ms 向所有 WebSocket 客户端广播 STATUS 帧
- 内容：四电机速度/PWM/电流/编码器/电池/错误标志/控制源
- 离线检测：500ms 无 STATUS → 标记离线 → LED 显示

**安全策略（CI 自动检查）：**
- 密码长度 8-63 字节
- WebSocket 控制需 owner 守卫
- ESTOP 只能置位（不可清除）
- 断开/启动时发送 neutral 命令
- 维护锁激活时禁止 flash bridge
- UART 错误处理先于 RX 消费

**Web 控制页面（~620 行 SPA）：**
- 虚拟摇杆（触控 + 鼠标）
- 速度缩放滑块（20%-100%）
- 四电机仪表盘（速度/编码器/电流/PWM）
- 电池电压表
- 错误码显示
- 控制源指示器
- 数据陈旧检测（300ms）

### 5.6 UART 回调分发

**模块：** `App/adapters/uart_callbacks.c`

**目的：** 集中管理所有 HAL UART 回调，按 `huart` 句柄路由到对应模块。

**回调路由表：**

| 回调 | huart1 (USART1) | huart2 (USART2) | huart3 (USART3) | huart4 (USART4) |
|------|-----------------|-----------------|-----------------|-----------------|
| `RxCplt` | DebugConsole / FlashBridge | ESP12F / FlashBridge | Upper DMA Full | — |
| `RxHalfCplt` | — | — | Upper DMA Half | — |
| `TxCplt` | — | FlashBridge | Upper TX | Line TX |
| `Error` | DebugConsole / FlashBridge | ESP12F / FlashBridge | Upper | Line |

---


## 六、安全与诊断

### 6.1 系统错误体系

**模块：** `Service/safety_management/safety_management_service.c`
**头文件：** `Service/safety_management/safety_management_service.h`

**19 个错误位定义：**

| 位 | 宏 | 含义 | 类型 | 触发 Fault-Stop |
|----|-----|------|------|:---:|
| 0 | `SYSTEM_ERROR_LOW_BATTERY` | 电池电压 < 10.5V 告警 | 告警 | — |
| 1 | `SYSTEM_ERROR_M1_OVERCURRENT` | M1 过流 | 故障 | — |
| 2 | `SYSTEM_ERROR_M2_OVERCURRENT` | M2 过流（左侧） | 故障 | ✓ |
| 3 | `SYSTEM_ERROR_M3_OVERCURRENT` | M3 过流（右侧） | 故障 | ✓ |
| 4 | `SYSTEM_ERROR_M4_OVERCURRENT` | M4 过流 | 故障 | — |
| 5 | `SYSTEM_ERROR_ESTOP` | 紧急停止激活 | 安全 | — |
| 6 | `SYSTEM_ERROR_FAULT_STOP` | 故障停止激活 | 安全 | — |
| 7 | `SYSTEM_ERROR_ENCODER_INVALID` | 编码器数据无效 | 告警 | — |
| 8 | `SYSTEM_ERROR_DRV_FAULT` | 电机驱动 nFAULT | 故障 | ✓ |
| 9 | `SYSTEM_ERROR_TIM_BREAK` | TIM Break 触发 | 安全 | ✓ |
| 17 | `SYSTEM_ERROR_ENCODER_FEEDBACK_LOST` | 编码器反馈丢失 | 故障 | ✓ |
| 18 | `SYSTEM_ERROR_BATTERY_CRITICAL` | 电池临界 (<9.0V) | 故障 | ✓ |

**Fault-Stop 触发掩码（以下任何一位置位自动触发 Fault-Stop）：**
- M2_OVERCURRENT / M3_OVERCURRENT（左右侧聚合过流）
- DRV_FAULT / TIM_BREAK / ENCODER_FEEDBACK_LOST / BATTERY_CRITICAL

**错误处理语义：**
- 告警类：记录标志，不打断运动
- 故障类：锁存（latch），触发 Fault-Stop，需 `clearfault` 清除
- `clearfault` 条件清除：过流需电流 < 故障阈值、DRV_FAULT 需 nFAULT 已释放、TIM_BREAK 需 break latch 已清除、编码器反馈丢失需所有启用的编码器有效
- `BATTERY_CRITICAL` 不可手动清除（仅电压恢复后自动清除）

### 6.2 SafetyService 健康聚合

**模块：** `Service/safety_management/safety_management_service.c`

**SafetyService_Update() 每 20ms（safetyTask）执行流程：**

1. **任务超时更新：** 调用 ChassisTaskTiming_UpdateTimeouts，获取 task_timeout_mask
2. **ADC 更新：** PowerAdcDriver_Update() → 电流/电压/有效性标志
3. **电机故障：** MotorDriver_UpdateFaults() → DRV nFAULT 状态
4. **过流评估（带去抖）：**
   - 启动消隐：上电后 PWM=0 期间 ARM，激活后 blank 期
   - 连续超阈值计数：需 5 次连续 @ 20ms = 100ms 去抖
   - 长期静止后重新 ARM
5. **电池评估（带滞回去抖）：**
   - < 10.5V → 告警；> 11.0V → 清除告警
   - < 9.0V 连续 500ms → 临界故障 + Fault-Stop
   - > 9.6V 连续 2s → 自动清除临界故障（不清除其他锁存）
6. **编码器有效性：** 检查 speed_valid_all
7. **TIM Break 锁存：** 检测 MotorDriver 的 break_latched 标志
8. **错误锁存与 Fault-Stop 触发：** 新故障出现时锁存并调用 ControlService_SetFaultStop

### 6.3 CurrentGuard 三层电流保护

**模块：** `App/current/current_guard.c`
**头文件：** `App/current/current_guard.h`

独立于 SafetyService 的实时电流保护层（在 ChassisService_Step 的 PID 输出后、PWM 输出前执行）。

**三层架构：**

| 层级 | 阈值参数 | 动作 |
|------|---------|------|
| **Observe** | `current_observe_a[]` | 仅记录超限计数，不影响输出 |
| **Soft-Limit** | `current_soft_limit_a[]` | PWM 等比缩放：`output × (soft_limit / measured)` |
| **Fault-Latch** | `current_fault_a[]` + `current_fault_debounce_ms` | 去抖后锁存故障（由 SafetyService 触发 Fault-Stop） |

**编译开关：**
- `MOTOR_CURRENT_GUARD_OBSERVE_ONLY` — 设为 1 时仅观察不保护（dry-run 模式）
- `MOTOR_CURRENT_SOFT_LIMIT_ENABLED` — 设为 1 时启用 soft-limit 等比缩放

**电流保护执行流程（CurrentGuard_ApplyMotorLimit）：**
1. 检查控制有效性（ADC 电流零点就绪 + 每电机掩码）
2. 评估 observe 阈值 → 记录超限计数
3. 评估 fault 阈值 + 去抖（累加直到 >= fault_debounce_ms/10ms 周期）
4. 评估 soft-limit：若 current > soft_limit_a 且 PWM != 0 → `requested × (soft_limit / measured)`
5. 返回限幅后的 PWM

### 6.4 IWDG 硬件看门狗

- `.ioc` 启用 IWDG，预分频器 32，重载值 1000，超时 ~1.0s（LSI 40kHz）
- `safetyTask`（High 优先级，20ms 周期）循环中条件喂狗：仅在 motorTask 心跳在 200ms 以内时喂狗
- motorTask 卡死 → 停止喂狗 → IWDG 超时 → 硬件复位
- 喂狗余量：50 个周期

### 6.5 TIM1/TIM8 Break 硬件保护

**TIM1 Break（权威保护）：**
- BKIN 输入：PE15，低电平有效
- 硬件自动切断 PWM（Automatic Output 禁用，需软件恢复）
- Break 锁存：只能在 BKIN/nFAULT 释放且 PWM/CCR 全零后软件清除
- ISR：`TIM1_BRK_TIM9_IRQHandler` → `MotorDriver_OnTim1BreakFromIsr()`

**TIM8 Break（冗余诊断）：**
- BKIN 输入：PA6，低电平有效
- 与 TIM1 共享同一 nFAULT 网络（四路 nFAULT 经二极管汇聚）
- 仅增加冗余诊断计数和时间戳，不独立切断 PWM

**诊断输出（status 命令）：** MOE 状态 / BIF 标志 / 累计 Break 次数 / 最后时间戳 / Break 来源

### 6.6 POST 上电自检

**模块：** `App/monitor/power_on_self_test.c`

**两阶段评估：**

**阶段一 — 启动前（PowerOnSelfTest_Run，调度器启动前）：**
1. 探测 DRV nFAULT（所有 nFAULT 必须为高）
2. 探测 IMU（BMI270 chip ID 必须为 0x24）
3. 输出结果到 USART1

**阶段二 — 延迟检查（PowerOnSelfTest_UpdateRuntime，safetyTask 驱动）：**
1. 等待 ADC 电流零点就绪（超时 2000ms）
2. 等待编码器速度有效（超时 2000ms）
3. 最终结果输出

**错误标志：** `PowerOnSelfTest_ERROR_DRV_FAULT` / `PowerOnSelfTest_ERROR_ADC` / `PowerOnSelfTest_ERROR_IMU` / `PowerOnSelfTest_ERROR_ENCODER`

### 6.7 Reset Trace v4 崩溃追踪

**模块：** `Platform/platform_reset_trace.c`
**存储：** `.noinit.reset_trace` 段（44 字结构体，不随复位清零），checksum 保护

**追踪类型：** NONE / NMI / HARDFAULT / MEMMANAGE / BUSFAULT / USAGEFAULT / ERROR_HANDLER / FREERTOS / DMA_GUARD

**记录内容（44 字段）：**
- 基础：magic(0x52545243), version(4), sequence, kind, reason, line, task
- SCB 寄存器：CFSR, HFSR, BFAR, MMFAR
- 心跳：5 个任务的时间戳
- 控制状态：source, estop, fault
- 异常帧：exc_return, stack pointer, MSP, PSP, CONTROL, FPCCR
- 栈帧：stacked LR, PC, XPSR（含 FPU 扩展帧偏移处理）
- DMA：DMA2 LISR, Stream0 CR/NDTR/FCR
- ADC：ADC1 SR, CR2
- 4 个 detail word（自定义扩展）

### 6.8 调试命令台

**模块：** `App/debug/usart1_debug_console.c`（~2171 行，最大单文件）
**硬件：** USART1 (PB6/PB7)，中断驱动环缓冲

**50+ 命令完整清单：**

**信息类：** `help` / `status` / `header` / `version` / `config export` / `errors` / `rtos`

**参数操作：** `get <name>` / `set <name> <value>` / `set save` / `set reset` / `set motor_dir <m1..m4> <1/-1>` / `set encoder_dir <m1..m4> <1/-1>`

**维护授权：** `maint arm`（激活 60 秒）/ `maint off`

**日志流：** `log 0` / `log 1 [fields...]` / `log rate <ms>` / `log csv|json`

**日志字段组（9 组）：** `motor` / `adc` / `adcraw` / `imu` / `errors` / `source` / `ps2` / `line` / `esp`

**运动控制：** `vel <V> [W]` / `motor <L> <R>` / `left <P>` / `right <P>` / `m<N> <F> <R>` / `raw <LF> <LR> <RF> <RR>` / `stop` / `estop 0|1` / `clearfault`

**巡线：** `line` / `line on|off` / `linecal floor|line <N>` / `linecal show|apply|cancel`

**IMU：** `imutest` / `imudiag` / `imuinit` / `imucal [N]` / `imucalclear` / `imu 0|1`

**ESP12F：** `espflash on|off|status` / `espat on|off` / `espreset` / `espboot` / `espisolate`

**ADC：** `adccal show|zero` / `adccal plan <mN> <known_mA>`

**其他：** `i2cscan`

**CSV 日志字段（全量 ~50 字段）：** 时间戳、运行时间、四电机速度/PWM/目标、电池电压、四电机电流、左侧/右侧电流、IMU roll/pitch/yaw、四元数、加速度/陀螺、IMU 质量标志、错误标志、控制源、PS2 状态、巡线位置/误差、ESP 状态

**JSON 日志：** PID 字段 + IMU 字段 + 电流字段，稳定 schema

### 6.9 调试维护授权策略

**模块：** `App/debug/debug_maintenance_policy.c`

- Debug 构建：始终允许运动测试命令
- Release 构建：需 `maint arm` 激活 60 秒授权
- 超时自动撤销

### 6.10 任务心跳与超时检测

**模块：** `App/chassis/chassis_task_timing.c`

9 个任务各自的心跳记录与超时检测：

| 任务 | 超时倍率 | 实际超时 |
|------|---------|---------|
| safetyTask | 4x 周期 | 80ms |
| motorTask | 4x 周期 | 40ms |
| rpiCommTask | 8x 周期 | 40ms |
| imuTask | 8x 周期 | 80ms |
| lineTask | 8x 周期 | 40ms |
| espTask | 8x 周期 | 40ms |
| ps2Task | 4x 周期 | 80ms |
| ledTask | 4x 周期 | 200ms |
| oledTask | 4x 周期 | 400ms |

超时掩码被纳入 DIAGNOSTIC (0x82) 帧上报。

---

## 七、UI 与显示

### 7.1 SSD1306 OLED 驱动

**模块：** `BSP/oled/ssd1306.c`
**硬件：** I2C1，地址 0x3C，128×64 单色

**帧缓冲：** 8 页 × 128 字节 = 1024 字节，脏页追踪（只刷新变更页）

**函数：**
- `SSD1306_Init()` — 全初始化序列（100ms 等待、充电泵使能、显示开启）
- `SSD1306_Refresh()` — I2C 批量写入脏页
- `SSD1306_SetPixel(x, y, on)` / `SSD1306_DrawChar(...)` / `SSD1306_DrawString(...)` — 基础绘制
- `SSD1306_FillRect(x, y, w, h, on)` — 填充矩形
- `SSD1306_DrawProgressBar(x, y, w, h, percent)` — 进度条（含边框）

**I2C 错误恢复：** 连续 3 次 I2C 失败后 deinit → reinit

**字体：** 8×16 ASCII（95 字符，页主序）、12×12 中文（57 字，列主序）、16×16 中文（57 字，列主序）

### 7.2 OLED 三阶段 UI

**模块：** `App/display/oled_ui.c`（~648 行）
**状态机：** `WELCOME(5s) → SELFCHECK(8项×600ms) → NORMAL`

**自检项（8 项）：**

| 序号 | 检测项 | 检测方法 | OK 条件 |
|------|--------|---------|---------|
| 1 | I2C | OLED I2C 探测 (0x3C) | ACK 正常 |
| 2 | IMU | BMI270 chip ID | 0x24 |
| 3 | ADC | 电池电压 | > 6.0V |
| 4 | Motor | DRV nFAULT | 全部为高 |
| 5 | Encoder | 编码器速度 | 全部有效 |
| 6 | UART3 RPI | Upper 通信 | 最后 RX 在 500ms 内 |
| 7 | UART4 LINE | 巡线传感器 | 数据新鲜（50ms 内） |
| 8 | ESP12F | ESP 通信 | 最后 RX 在 500ms 内（下载模式跳过） |

**运行屏显示：** 运行时间 (HH:MM:SS) / 电池电压 (V) / 错误码（十六进制，闪烁） / TIM Break 时显示 "BKIN!" / 控制源标签

### 7.3 OLED 校准进度显示

**模块：** `App/display/oled_calibration_view.c`

非阻塞显示模型，从 IMU 校准状态机构建 view struct：

| 校准状态 | 标题 | 详情 | 自动消失 |
|---------|------|------|---------|
| WAIT / RETRY_WAIT | IMU CAL WAIT | 失败原因 | — |
| RUNNING | IMU CAL RUN | SAMPLING % | — |
| DONE | IMU CAL DONE | SAVED IN RAM | 3s 后 |
| FAILED | IMU CAL FAILED | 原因 | 3s 后 |

**失败原因：** CONFIG ERROR / READ ERROR / BIAS TOO HIGH / NOISE TOO HIGH / KEEP STILL / STILL GATE

### 7.4 LED 状态指示

**模块：** `BSP/led/status_led_driver.c`
**硬件：** TEST_LED (PE6)

**5 种闪烁模式：**

| 模式 | 周期 | 含义 |
|------|------|------|
| NORMAL | 1s (500ms on) | 正常运行 |
| UPPER_LINK | 600ms (300ms on) | 上位机已连接 |
| LOW_BATTERY | 200ms (100ms on) | 电池低压告警 |
| FAULT | 200ms (100ms on) | 故障锁存 |
| ESTOP | 200ms (100ms on) | 急停激活 |

---

## 八、参数与持久化

### 8.1 运行时参数存储（ParamService）

**模块：** `Service/parameter_management/parameter_management_service.c`

**核心数据结构（50+ 字段）：**

| 类别 | 字段 | 默认值 |
|------|------|--------|
| 运动学 | `max_linear_mps` | 0.5 |
| 运动学 | `max_angular_rps` | 10.0 |
| 运动学 | `wheel_radius_m` | 0.035 |
| 运动学 | `track_width_m` | 0.176 |
| 斜坡 | `speed_ramp_mps2` | 0.5 |
| 斜坡 | `angular_ramp_rps2` | 2.0 |
| PID | `motor_pid_kp[4]` | 8.0/8.0/8.0/0.5 |
| PID | `motor_pid_ki[4]` | 0.3/0.3/0.3/0.0 |
| PID | `motor_pid_kd[4]` | 0.1/0.1/0.1/0.0 |
| PID | `pid_integral_limit` | 60.0 |
| 方向 | `motor_dir[4]` | 1/1/1/1 |
| 方向 | `encoder_dir[4]` | 1/1/1/1 |
| 电流 | `current_observe_a[4]` | 运行时 |
| 电流 | `current_soft_limit_a[4]` | 运行时 |
| 电流 | `current_fault_a[4]` | 运行时 |
| 电流 | `current_fault_debounce_ms` | 运行时 |
| 巡线 | `line_threshold_raw[8]` | 500 |
| 巡线 | `line_active_low` | 0 |
| 巡线 | `line_kp` / `line_kd` | 2.5 / 0.1 |
| 巡线 | `line_speed_mps` | 0.15 |
| 巡线 | `line_slowdown_gain` | 0.0 |
| 直行 | `straight_wheel_coupling_gain` | 运行时 |
| 直行 | `straight_heading_kp` | 运行时 |
| 直行 | 四个 `straight_trim_*` | 运行时 |
| 直行 | `straight_heading_ki` / `straight_heading_integral_limit_deg_s` | 运行时 |
| 直行 | `straight_max_speed_mps` | 运行时 |
| 直行 | `straight_heading_hold_enabled` | 运行时 |
| IMU | `imu_gyro_bias_dps[3]` | 0.0 |
| ADC | `current_zero_raw[4]` | 运行时 |

**Generation 追踪：** 32 位单调递增计数器，每次 Set/SetDefaults 递增。ChassisService 通过 GetSnapshot 检测变化后热重载 PID 参数。

**可绑定命名参数（get/set 命令用）：** 原有参数外，新增四个 `straight_trim_*`、`straight_heading_ki`、`straight_heading_integral_limit_deg_s` 与 `straight_max_speed_mps`。

### 8.2 Flash 参数持久化

**模块：** `BSP/flash/flash_param.c`
**硬件：** STM32F407 内部 Flash，Sector 6 (0x08040000) + Sector 7 (0x08060000)，各 128KB

**A/B 双区交替写入：** 每次保存写到与当前有效区不同的区。加载时选 sequence 号更大且 CRC32 校验通过的区。双区均损坏时退化为默认值。

**镜像格式（Schema V4）：**
```
[MAGIC: 0x464B3037 (4B)] [SCHEMA_VERSION: 4 (4B)] [SEQUENCE (4B)]
[PAYLOAD_SIZE (4B)] [param_model_t + imu_bmi270_calibration_t]
[CRC32 (4B)] [COMMIT_MARKER: 0xC01117ED (4B)]
```

**写入原子性：** 所有数据写入后，最后写 commit_marker。若写入期间掉电，无 marker 的 slot 在下次启动时被判定为无效。

**Schema 迁移：** V1 (legacy)、V2、V3 均显式解码到 V4；V3 不按新结构解释，迁移后强制关闭新航向控制器。

**安全措施：** 擦除/写入期间扩展 IWDG（预分频 256, reload=4095），避免 16KB sector 擦除（100-200ms）触发看门狗。写入前通过 ChassisMaintenance_Begin() 获取维护锁（确保 PWM=0）。

---

## 九、测试体系

### 9.1 Host 测试（34 tests）

| # | Target | 覆盖模块 |
|---|--------|---------|
| 1 | `f407_v2_host` | 协议 V2、控制仲裁、ESTOP/fault-stop、差速、编码器差分、任务时序、IMU 状态 |
| 2 | `motor_hardware_layout_2wd` | 默认两驱布局 |
| 3 | `runtime_direction_apply` | 运行时电机/编码器方向覆盖 |
| 4 | `straight_controller` | 双向 trim、gyro PI、饱和降速与 IMU 退化 |
| 5 | `esp_link_policy` | ESP 链路策略（在线/离线/500ms 超时/neutral） |
| 6 | `esp_frame_parser` | ESP 帧解析器（帧头/CRC/长度校验） |
| 7 | `power_adc_driver` | ADC 电流零点、M2/M3 映射、电流 EMA、电池 EMA |
| 8 | `power_adc_driver_stack_budget` | ADC monitor 任务栈预算（<=256 字节） |
| 9 | `line_sensor_driver` | 巡线帧解析、校验失败、丢线恢复 |
| 10 | `line_control` | LineControl enable generation、超时、安全撤销 |
| 11 | `line_calibration` | 双表面标定：采集、均值、分离度、active_low |
| 12 | `upper_uart` | USART3 协议解析、超时重置、CRC 错误 |
| 13 | `motor_driver_gpio` | 电机驱动 GPIO 输出逻辑 |
| 14 | `safety_service` | 电池/电流/DRV/任务超时聚合、去抖、fault-stop |
| 15 | `safety_service_fault_enabled` | 过流 fault 全链路（fault 使能编译开关） |
| 16 | `current_guard` | CurrentGuard observe 模式 |
| 17 | `current_guard_soft_limit` | CurrentGuard soft-limit 等比缩放 |
| 18 | `chassis_control_current_limit` | 电流限制与底盘控制集成 |
| 19 | `chassis_maintenance` | 统一维护锁、静止检查、释放路径 |
| 20 | `debug_maintenance_policy` | Release 维护授权、60s 超时、撤销 |
| 21 | `debug_log_policy` | 日志周期/格式策略 |
| 22 | `debug_straight_telemetry` | 直行遥测 CSV/JSON 格式化 |
| 23 | `relative_yaw_control` | 相对 yaw 积分、限幅、稳定窗、超时、取消 |
| 24 | `ps2_control` | PS2 中位让权、定角宏、IMU 门控、手动打断 |
| 25 | `oled_ssd1306` | SSD1306 OLED 驱动 |
| 26 | `imu_pipeline` | IMU 校准门控、FIFO 解析、Mahony 融合、坐标映射 |
| 27 | `param_service` | ParamService 读写、校验、默认值、generation |
| 28 | `wheel_encoder_driver` | Encoder 短临界区发布、运行时轮径 |
| 29 | `flash_param` | Flash A/B 双区、CRC、schema 迁移、legacy 兼容 |
| 30 | `power_on_self_test` | POST 快照评估、运行时就绪、超时 |
| 31 | `oled_selfcheck` | OLED 自检逻辑（纯函数） |
| 32 | `oled_calibration_view` | IMU 校准 OLED 显示模型 |
| 33 | `analysis_scripts` | Python 分析脚本单元测试 |
| 34 | `upper_v2_golden` | Upper Protocol V2 黄金帧跨仓一致性 |

**Mock 框架：** tests/host/ 下提供 cmsis_os2.h、adc.h、usart.h、i2c.h、tim.h、main.h 等主机端替代头文件。

### 9.2 CI 门禁

| Job | 内容 | 平台 |
|-----|------|------|
| **build** | Debug + Release 构建 + 大小限制（<=240KB Flash, <=112KB RAM） + 产物上传 | ubuntu-24.04 |
| **host-tests** | 34 个 ctest target + CubeMX 安全配置检查 | ubuntu-24.04 |
| **format-check** | clang-format 干运行（App/BSP/tests） | ubuntu-24.04 |
| **static-analysis** | cppcheck --enable=warning,style（App/BSP/tests） | ubuntu-24.04 |
| **esp8266-build** | Arduino CLI 编译 ESP8266 (Core 3.1.2) + ESP 安全策略检查 | ubuntu-24.04 |
| **CodeQL** | 安全漏洞扫描（push/PR + 周扫描） | ubuntu-24.04 |

### 9.3 HIL 与数据分析脚本

| 脚本 | 功能 |
|------|------|
| `hil_smoke.py` | USART1 只读冒烟测试（version/status/i2cscan/imutest/espflash） |
| `hil_imu_calibration.py` | IMU 校准不增加任务超时的验证 |
| `analyze_roadmap_data.py` | 四模式数据分析：电流/编码器/巡线/几何 |
| `analyze_imu.py` | IMU 静止噪声/yaw 漂移/水平恢复/温度相关性 |
| `check_generated_safety_config.py` | CubeMX 生成代码安全检查（TIM Break/NVIC/ISR） |

---

## 十、构建身份与工具链

### 10.1 版本身份

**文件：** `VERSION`（内容：`1.0.0`）

**构建时注入（cmake/build_identity.h.in → generated/build_identity.h）：**
```c
#define F407_FIRMWARE_VERSION  "1.0.0"     // 来自 VERSION 文件
#define F407_GIT_SHA           "10a1462e"  // git rev-parse --short=8 HEAD
#define F407_BUILD_TYPE        "Debug"     // Debug / Release
#define F407_BUILD_DIRTY       0           // 1 = 工作区有未提交变更
```

### 10.2 构建目标总览

**预设：** Debug (`-O0 -g3`) / Release (`-Os -g0`)
**编译器：** `arm-none-eabi-gcc` (newlib-nano, `-u,_printf_float`)
**编译标准：** C11 with GNU extensions
**告警：** `-Wall -Wextra -Wpedantic -Werror`
**替代工具链：** ST ARM Clang（cmake/starm-clang.cmake, 三种配置）

**子库：** stm32cubemx (INTERFACE) / STM32_Drivers (OBJECT) / FreeRTOS (OBJECT)

**后处理：** objcopy → .hex / .bin + arm-none-eabi-size → Flash/RAM 摘要

### 10.3 源码清单

**五层源码：** App 仅保留初始化、十任务、ISR、UI/调试适配；Service 持有 chassis/control/safety/param/communication；Domain 持有纯算法和值类型；BSP 持有设备与 transport；Platform 持有 RTOS/Cortex-M 系统能力。详见 `docs/architecture.md`。

**固件：** firmware/esp12f/F407_ESP12F.ino（~620 行 Web 控制页面）

---

## 附录 A：底盘控制全流程详解

```
1. motorTask 每 10ms 调用 ChassisService_Step(now_ms)

2. 重入防护：设置 control_step_active=true，防止维护锁期间重入

3. ParamService 热重载：检测 generation 变化，更新 PID 参数

4. 故障检查：MotorDriver_UpdateFaults() → nFAULT / Break 状态

5. 编码器更新：WheelEncoderDriver_Update(now_ms) → 四轮速度

6. 安全检查：
   ├── 维护锁激活 → 清零输出，跳过控制
   ├── ESTOP/Fault-Stop → ChassisService_EmergencyStop()
   └── 测试模式：检查维护授权 + 400ms 租约

7. 获取命令：ControlService_GetCommand() → 最高优先级活跃命令

8. 差速分解：DifferentialDriveKinematics_ResolveDifferentialTargets(lx, az, track, &left, &right)

9. 速度斜坡：左右目标速度分别渐进（加速度/减速度同斜率）
   方向反转时：降到零 → 反制动 → 相位切换 → 反加速

10. 直行补偿：StraightController_Step()
    ├── 方向分档 trim + wheel_gain × (vL_actual - vR_actual)
    ├── gyro-z 积分航向 PI，IMU 失效时同周期退化
    └── 纠偏限幅、anti-windup 与弱侧饱和共同降速

11. 轮速等比缩放：一侧达到 max_linear_mps 时等比压缩另一侧

12. PID 步进：每电机 PidController_StepLimited（方向感知抗饱和）
    ├── 误差 = 目标 - 实际
    ├── P 项 = Kp × 误差
    ├── I 项 = Ki × 累积 × dt（带限幅 + 条件冻结）
    ├── D 项 = Kd × (误差 - 上次误差) / dt
    └── 输出 = P + I + D → 限幅到 ±900 permille

13. 电流保护：CurrentGuard_ApplyMotorLimit()
    ├── Observe：记录超限计数
    ├── Soft-Limit：等比缩放 output × (limit/measured)
    └── Fault：去抖计数

14. 反馈丢失检测：启用编码器无效 + 非零请求 150ms 内无运动 → 整车停车

15. PWM 输出：MotorDriver_SetPermille(motor, permille)
    → MotorPhEnMapper_ResolvePhaseEnable → (abs_permille, phase_bit)
    → TIM1 CCRx + GPIO PH 输出
```

## 附录 B：控制源切换时序

```
1. PS2 摇杆推前 0.3m/s → ControlService_SetCommandForGeneration(PS2, ...)
2. ControlService_GetCommand() → PS2 活跃，返回 0.3m/s
3. 上位机发送 0.2m/s → ControlService_SetCommand(UPPER, ...)
4. ControlService_GetCommand() → UPPER 优先级更高，返回 0.2m/s
5. 上位机停止发送 → 200ms 后 UPPER 超时
6. ControlService_GetCommand() → 回退到 PS2，返回 0.3m/s
7. PS2 掉线 → 500ms 后 PS2 超时
8. ControlService_GetCommand() → NONE（速度全零，底盘停止）

安全插入：
1. 运行中触发 ESTOP → 所有命令清除 → 紧急停止
2. 运行中 M2 过流 → fault-stop + latch → 紧急停止
3. clearfault → 故障消除后清除 latch → 恢复正常
4. 维护锁激活 → 所有命令清除 → 释放后需重新发送
```

## 附录 C：IMU 数据流

```
BMI270 INT1 (PE0) ──(上升沿)──→ EXTI0 ISR
  └── osThreadFlagsSet(imuTask, DRDY_BIT)
       └── imuTask 唤醒

Bmi270Driver_Update():
  ├── 尝试 FIFO 读取（最多 8 帧）
  │   └── ImuBmi270Fifo_Parse() → accel_raw[] + gyro_raw[] + sensor_time
  ├── FIFO 失败时回退直接寄存器读取
  └── 转换为物理量
       ├── accel_raw → accel_g (÷16384)
       └── gyro_raw → gyro_dps (÷65.6)

温度补偿：
  gyro_bias = base_bias + temp_slope × (T_current - T_offset)

EMA 滤波：
  gyro_filtered = α × gyro_raw + (1-α) × gyro_filtered_prev  (α=0.20)

Mahony AHRS：
  ImuBmi270Mahony_Update(gyro_corrected, accel_g, dt)
    ├── 陀螺积分：q = q + 0.5 × dt × q ⊗ [0, gx, gy, gz]
    ├── 加速度校正（自适应权重）：
    │   ├── |norm-1g| < 0.25 → weight=1.0
    │   ├── |norm-1g| < 0.60 → weight=0.1（退化）
    │   └── 超出 → weight=0.0（仅陀螺）
    └── 输出四元数 → ImuBmi270Quaternion_ToEulerDeg() → roll/pitch/yaw

坐标变换：
  sensor_frame → body_frame (3×3 rotation matrix) → ros_frame (REP-103)

自动校准门控：
  ImuCalibrationGate_Update() → 检测静止（PWM=0 + 速度<0.02 + 加速度稳定 + 陀螺稳定）
    → 满足 100 样本 → Bmi270Driver_ServiceCalibration() 触发自动校准
    → 累积 500 样本 → ImuBmi270Calibration_GyroBiasAtTemperature() → 保存到 Flash
```

## 附录 D：电机驱动状态机

```
     ┌──────────────┐
     │ IDLE_BRAKE   │  初始状态 / 停止后
     │ PWM=0, brake │
     └──────┬───────┘
            │ 收到非零 permille 目标
            ▼
     ┌──────────────┐
     │ RAMP_UP      │  每周期 +15 permille
     │ 递增到目标   │
     └──────┬───────┘
            │ 达到目标
            ▼
     ┌──────────────┐
     │ RUN          │  稳态运行
     │ 维持目标 PWM │
     └──┬───────┬───┘
        │       │ 方向反转请求
        │       ▼
        │ ┌──────────────┐
        │ │ RAMP_DOWN    │  每周期 -25 permille（快降）
        │ │ 递减到零     │
        │ └──────┬───────┘
        │        │ PWM=0
        │        ▼
        │ ┌──────────────┐
        │ │REVERSE_BRAKE │ 低侧制动 ≥2 周期
        │ │等速度<0.02   │
        │ └──────┬───────┘
        │        │ 制动完成或超时
        │        ▼
        │ ┌──────────────┐
        │ │ PH_SETTLE    │ 方向引脚切换 + 4 周期间隙
        │ │ 方向位翻转   │
        │ └──────┬───────┘
        │        │ 间隙完成
        │        ▼
        │ ┌──────────────┐
        │ │ RAMP_UP      │ 每周期 +15 permille（新方向）
        │ │ 递增到目标   │
        │ └──────────────┘
        │
        ▼ 目标=0
     ┌──────────────┐
     │ RAMP_DOWN    │
     │ 递减到零     │
     └──────┬───────┘
            │ PWM=0
            ▼
     ┌──────────────┐
     │ IDLE_BRAKE   │
     │ 回到初始状态 │
     └──────────────┘
```

---

> **文档版本：** 1.0
> **覆盖范围：** F407 V2.0 全部 50+ 功能模块、11 个 FreeRTOS 任务、35+ 子系统、34 个 Host 测试 target
> **最后更新：** 2026-07-12
