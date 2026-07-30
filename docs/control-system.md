
# 控制体系

> rc1 远程速度由 Upper Protocol v3 dispatcher 协调，业务操作经固定 mailbox 交给 App 编排。Communication 负责 wire/session/ACK/请求阶段；CommandManagement 唯一负责五来源租约、rearm 与仲裁；SafetyManagement 唯一决定运动许可；MotionControl 唯一写电机输出。

rc2 控制周期按实测时间分为 FIRST、EARLY（0--4 ms）、NORMAL（5--15 ms）、LATE（16--20 ms）和 MISSED（不少于 21 ms）。EARLY 保持上一输出且不更新 PID；FIRST/MISSED 归零并复位控制状态。轮速连续拒绝三次进入 REACQUIRING，拒绝样本不作为基线，连续三个物理有效样本后恢复。

## 1. 控制链数据流

```
控制源 → CommandManagement       优先级仲裁 + 超时 + motion gate
       → MotionControl_Update    每 10ms，motorTask 驱动
         → MotorHardwareLayout         电机启用/侧别/方向查表
         → 差速模型              linear_x / angular_z → left_mps / right_mps
         → 速度斜坡              按 CHASSIS_SPEED_RAMP_MPS2 平滑过渡
         → StraightController    方向 trim + 轮速耦合 + gyro PI (angular_z=0 自动激活)
         → PID 速度环            独立四路 PID 控制器 (M1-M3 PID, M4 纯 P)
         → permille 输出          限幅到 CHASSIS_PWM_MAX_PERMILLE
         → MotorDriver           TIM1 EN/PWM + GPIO PH/DIR 输出
```

---

## 2. 控制源优先级

```
上位机 (USART3)  >  PS2 手柄  >  ESP12F (WiFi)  >  巡线 (UART4)  >  调试台 (USART1)
```

| 控制源 | 枚举值 | 接口 | 激活方式 | 说明 |
| --- | --- | --- | --- | --- |
| **UPPER** | `CONTROL_SOURCE_UPPER (1)` | USART3 DMA | RPI 上位机发送帧 | 最高优先级，用于 ROS/上位机自主导航 |
| **PS2** | `CONTROL_SOURCE_PS2 (2)` | GPIO bit-bang | 手柄在线 + 摇杆/十字键操作 | 手柄三角键切换巡线模式 |
| **ESP12F** | `CONTROL_SOURCE_ESP12F (3)` | USART2 中断 | WiFi 模块发送 `SET_VELOCITY` 帧 | 支持 `LINE_CTRL` 远程启停巡线 |
| **LINE** | `CONTROL_SOURCE_LINE (5)` | UART4 DMA | 巡线启用 + 传感器检测到黑线 | 默认关闭，丢线自动退让 |
| **DEBUG** | `CONTROL_SOURCE_DEBUG (4)` | USART1 中断 | 调试台 `vel` 命令 | 最低优先级，用于人工验证调试 |

### 2.1 仲裁逻辑

`CommandManagement_GetActive()` 按优先级数组 `{HOST, PS2, ESP12F, LINE, DEBUG}` 顺序遍历各源命令槽：

1. 检查源命令的 `enable` 标志
2. 按控制源验证命令年龄：UPPER 为 200ms，PS2/ESP12F 为 500ms，LINE 为 50ms，DEBUG 为 2000ms
3. 命中即返回，不再检查更低优先级源
4. 若所有源均未命中，返回空命令（`enable=0`），底盘停止

### 2.2 源过期机制

每个控制源需在各自超时窗口内刷新命令（更新 `timestamp_ms`）。超时未刷新的命令槽自动失效，仲裁循环跳过该源。

---

## 3. 安全语义

### 3.1 安全状态

| 状态 | 触发条件 | 行为 |
| --- | --- | --- |
| **ESTOP (紧急停止)** | `estop 1` 命令 | 清空全部控制源命令，拒绝所有新命令 |
| **fault-stop (故障停止)** | 任一路 DRV8874 `nFAULT` 低有效 | 清空全部命令、EN=0 停止输出并进入 PH/EN 低侧慢衰减制动；注：**不**拉低 DRV_SLEEP_ALL（仅 fatal loop 和 HardFault 才会拉低 SLEEP） |
| **DRV 故障锁存** | DRV8874 nFAULT 拉低 | 触发 fault-stop，锁存 error flag。ADC 电流当前只用于日志诊断，`MOTOR_ADC_OVERCURRENT_FAULT_ENABLED=0U` 表示不使用 ADC 峰值触发 fault-stop |
| **硬件 Break** | 共享 nFAULT 网络拉低 PE15/TIM1_BKIN 与 PA6/TIM8_BKIN | TIM1 立即清 MOE/CCR，Automatic Output 禁用并软件锁存；只有 BKIN/nFAULT 释放且 PWM 全零时才可清除。TIM8 只记录同网冗余 BIF/count/last_ms，不重复锁存 |
| **RTOS 异常** | `configASSERT` / 栈溢出 / malloc 失败 / 任务创建失败 | 拉低 `DRV_SLEEP_ALL`，进入 fatal loop（不自动复位，保留故障现场） |

### 3.2 命令拒绝规则

`CommandManagement_Set()` 在以下任一条件时拒绝命令：

- 当前处于 ESTOP 或 fault-stop 状态
- 源 ID 非法（`NONE` 或超出 `CONTROL_SOURCE_LINE`）
- 速度值为 NaN 或 Inf → **额外触发 reject-and-stop**：拒绝并清空该源命令槽
- 命令 `enable == 0` → 清空该源槽位
- 运动学无效（轮半径/轴距 ≤ 0）且角速度非零 → 拒绝角速度，直线仍允许

clamping 规则：`linear_x` 钳位到 `±CHASSIS_MAX_LINEAR_MPS`（0.5 m/s），`angular_z` 钳位到 `±CHASSIS_MAX_ANGULAR_RPS`（10 rad/s）。

### 3.3 巡线特有安全行为

- **默认关闭**：`LINE_DEFAULT_ENABLED = 0U`，上电不启用
- **丢线退让**：8 路全白（无黑线）→ `ClearSource(LINE)` → 回退到 DEBUG 或更低源
- **超时退让**：传感器 50ms 无有效帧 → `ClearSource(LINE)`
- **手动覆盖**：PS2 摇杆操作时 LINE 被优先级仲裁自动跳过

---

### 3.3 P0 运动安全

- ParamService generation 快照供 ControlService、ChassisService 和 WheelEncoderDriver 同代消费；代际变化清 PID/斜坡。电机/编码器方向支持编译期默认（`MotorHardwareLayout`）和运行时覆盖（`set motor_dir`/`set encoder_dir`）。
- 维护锁先阻止新命令，再清控制源/raw/open-loop/PWM 并确认编码器静止。raw/open-loop 租约 400ms；Release 还需本地 `maint arm` 60s 授权。
- 维护、ESTOP 和 fault-stop 首次置位都会递增 motion-revoke generation；此前启用的 LINE 和自动续发 DEBUG `vel` 会失效，解除安全锁后必须重新执行 `line on` 或发送新的本地 `vel`，不会恢复旧运动模式。
- 正常闭环任一 enabled encoder 无效，或 RUN/PWM 有效时非零 requested target 持续 150ms 无运动，当前 motorTask 周期立即整车停车并锁存 bit17。
- PS2 非巡线中位提交零速；巡线中位让权给 LINE。L1/R1=±90°，L2/R2=±360°，使用 IMU yaw 增量累计。

## 4. FreeRTOS 任务模型

### 4.1 任务表

| 任务 | 入口函数 | 优先级 | 周期 | 调度方式 | 栈大小 | 核心职责 |
| --- | --- | --- | --- | --- | --- | --- |
| **safetyTask** | `Task_Safety` | High (osPriorityHigh) | 20ms | Platform 周期延时 | 4096B | 更新传感器、`SafetyService_Update`、ResetTrace 心跳与看门狗门控 |
| **motorTask** | `Task_MotorControl` | AboveNormal | 10ms | Platform 周期延时 | 512W (2048B) | `EncoderService_Update` + `ChassisService_Step` |
| **rpiCommTask** | `Task_RpiComm` | Normal | 5ms | `osDelayUntil` | 512W (2048B) | `HostCommunication_Update`：USART3 上位机协议收发 |
| **imuTask** | `Task_Imu` | Normal | DRDY 优先 / 10ms 超时降级 | `osThreadFlagsWait` | 512W (2048B) | BMI270 INT1 唤醒后读取 FIFO/SensorTime；超时走直读轮询降级 |
| **ps2Task** | `Task_Ps2` | Normal | 20ms | `osDelayUntil` | 512W (2048B) | `PlatformResetTrace_TaskHeartbeat` + `Ps2Control_Update`：PS2 手柄数据读取 + 巡线切换检测 |
| **lineTask** | `Task_Line` | BelowNormal | 5ms | `osDelayUntil` | 1024W (4096B) | `LineSensorDriver_Update` + `LineSensorDriver_RequestAnalog` + `LineControl_Update`：帧解析、传感器查询、P 控制提交 |
| **espTask** | `Task_Esp12f` | BelowNormal | 5ms | `osDelayUntil` | 512W (2048B) | `PlatformResetTrace_TaskHeartbeat` + `Esp12fFlashBridge_Update` + `WirelessCommunication_Update`：ESP12F 协议收发与烧录桥 |
| **debugTask** | `Task_Debug` | BelowNormal | 10ms | `osDelay` | 2048W (8192B) | `PlatformResetTrace_TaskHeartbeat` + 命令解析、2 Hz CSV 日志、`vel` 指令刷新 |
| **ledTask** | `Task_Led` | Low (osPriorityLow) | 50ms | `osDelayUntil` | 128W (512B) | `StatusLedDriver_TaskStep`：TEST_LED 状态闪烁 |
| **oledTask** | `Task_Oled` | Low (osPriorityLow) | 100ms | `osDelayUntil` | 256W (1024B) | `OLED_UI_Update`：SSD1306 OLED 三阶段 UI 刷新（欢迎/自检/运行） |

### 4.2 FreeRTOS 关键配置

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `configTICK_RATE_HZ` | `1000` | 1ms 系统节拍 |
| `configMAX_PRIORITIES` | `56` | 最大优先级数 |
| `configTOTAL_HEAP_SIZE` | `49152` (48KB) | heap_4 动态内存池 |
| `configCHECK_FOR_STACK_OVERFLOW` | `2` | 启动时检查栈顶 watermark + 运行时检查 |
| `configUSE_MALLOC_FAILED_HOOK` | `1` | malloc 失败时进入 fatal loop |
| `configENABLE_FPU` | `1` | 启用硬件浮点单元（lazy stacking） |
| `configUSE_TIMERS` | `1` | 启用软件定时器 |
| `configUSE_MUTEXES` | `1` | 启用互斥锁 |
| `configUSE_COUNTING_SEMAPHORES` | `1` | 启用计数信号量 |

### 4.3 调度监控

`rtos` 命令输出：

```
RTOS heap_free=XXXXXB heap_min=XXXXXB heap_used=XXXXXB tick=XXXXX
RTOS safety    state=X stack_free=XXXXB missed=X
...
RTOS oled      state=X stack_free=XXXXB missed=X
RTOS comm upper_tx=X upper_drop=X esp_tx=X esp_drop=X
```

- **`heap_free`**：当前 FreeRTOS heap 可用字节；**`heap_min`**：历史最低值；**`heap_used`**：当前已分配字节
- **`stack_free`**：各任务剩余栈空间（`uxTaskGetStackHighWaterMark`）。持续低于 128–256B 应增大任务栈
- **`missed`**：周期任务执行耗时超过目标唤醒点的累计次数。空载时核心任务 `missed` 不应持续增长
- **`upper_drop` / `esp_drop`**：USART TX 繁忙导致状态帧跳过的次数

---

## 5. 底盘布局配置

`Domain/config/control_config.h / BSP/bsp_config.h` 提供编译期布局配置，支持四驱/两驱和自定义左右侧：

| 宏 | 可选值 | 说明 |
| --- | --- | --- |
| `CHASSIS_Mx_ENABLED` | `0U` / `1U` | 电机启用。禁用后目标/PWM/PID/编码器/电流固定为零，nFAULT 不触发整车停机 |
| `CHASSIS_Mx_SIDE` | `MOTOR_SIDE_LEFT` / `MOTOR_SIDE_RIGHT` | 侧别归属，决定接收 `left_mps` 还是 `right_mps` |
| `CHASSIS_Mx_MOTOR_DIR` | `1` / `-1` | PWM 方向修正。`1`=正输出对应前进，`-1`=反向 |
| `CHASSIS_Mx_ENCODER_DIR` | `1` / `-1` | 编码器方向修正。前进时 `status` 速度应为正值 |

**默认布局**（两驱）：M2 左侧，M3 右侧（M1/M4 禁用），M1/M2 encoder dir `+1`，M3/M4 encoder dir `-1`。（V2.0 实板默认：`CHASSIS_M1_ENABLED=0U, CHASSIS_M2_ENABLED=1U, CHASSIS_M3_ENABLED=1U, CHASSIS_M4_ENABLED=0U`。）

**常用两驱**：`CHASSIS_M2_ENABLED=0U, CHASSIS_M4_ENABLED=0U`（仅 M1+M3）或 `CHASSIS_M1_ENABLED=0U, CHASSIS_M3_ENABLED=0U`（仅 M2+M4）或 `CHASSIS_M1_ENABLED=1U, CHASSIS_M4_ENABLED=0U`（仅 M1+M3 变体）。

**V2.0 实板 M2/M3 映射**：逻辑 M2 使用 EN/PWM `TIM1_CH2 PE11`、PH/GPIO `PC7`、nFAULT `PD14`、编码器 `TIM4 PD12/PD13`、电流采样 `PC1`；逻辑 M3 使用 EN/PWM `TIM1_CH3 PE13`、PH/GPIO `PC8`、nFAULT `PA3`、编码器 `TIM3 PB4/PB5`、电流采样 `PC2`。CubeMX 生成文件保留旧 M2/M3 标签，BSP 层负责逻辑归属。

**安全约束**：左右两侧必须各至少启用一路电机，否则 `MotorHardwareLayout` 拒绝运动输出。

---

## 6. 教育科研使用边界

- **raw/open-loop 命令**（`m1`~`m4`、`raw`、`left`、`right`、`motor`）：用于 bring-up 阶段的单轮方向验收、示波器波形测量、硬件排故。受 ESTOP、fault-stop、DRV fault 和过流锁存约束。建议架空轮、低占空比（<300‰）、有人看护
- **`vel` 闭环**：应在电压/电流/编码器方向/DRV fault/raw 方向全部确认后执行，从低速（50 mm/s）开始
- **IMU / PS2 / ESP12F / 巡线**：离线不阻塞底盘实验，状态在 `status` 中可见
- **巡线安全**：默认不启用；需 PS2/ESP12F/调试台显式开启；PS2 摇杆随时可覆盖；丢线自动退让不锁死
- **ESP12F flash bridge**：维护功能。进入前获取统一维护锁并确认静止，active 全生命周期拒绝所有控制源；USART1 被二进制透传接管，30s 无活动自动退出并释放锁
# P1-P3 运行时增强

- 电机/编码器方向、巡线阈值/极性/PD/速度、电流三层阈值与直行补偿增益均由 ParamService 原子快照驱动；编译期宏只提供安全默认值。
- 所有来源的 `angular_z=0` 在底盘层统一进入 `StraightController`。控制器按前进/后退和 0.15/0.30m/s 插值 trim，再叠加轮速耦合与 gyro-z 积分航向 PI；不等待 400ms，也不使用 Mahony yaw。
- IMU stale、未校准、SPI/timestamp/gyro quality 异常时同周期退化为 trim + wheel，恢复周期清零航向参考，禁止输出阶跃。纠偏/PWM/电流饱和时冻结积分；弱侧无法增速时降低共同速度以保留差速。
- 默认 trim、Kp、Ki 和航向保持关闭，必须用匹配固件身份的双向 2m 基线/改后报告决定参数。前 0.30m 万向轮翻转段单独记录，不计入稳态验收。
