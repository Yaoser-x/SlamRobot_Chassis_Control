
# 控制体系

> rc2 远程速度由 Upper Protocol v3 dispatcher 协调，业务操作经固定 mailbox 交给 App 编排。
> Communication 负责 wire/session/ACK；CommandManagement 唯一负责五来源槽、租约、rearm、mode mask 与仲裁；
> SafetyManagement 只决定能力和许可租约；MotionControl 只消费 App DTO、写电机并返回事件。

rc2 控制周期以 nominal period `P` 和 `uint64_t` 整数比较派生边界：`2dt<P` 为 EARLY，
`2dt<=3P` 为 NORMAL，`dt<=2P` 为 LATE，其余为 MISSED。EARLY 不更新 accepted baseline；
NORMAL/LATE 使用真实 dt；FIRST/MISSED 归零并复位 PI 与目标整形。轮速连续拒绝三次进入
REACQUIRING，异常原始计数仍累计；连续三个稳定样本只重建 baseline，不补算异常区间。

## 1. 控制链数据流

```
控制源 → CommandManagement       优先级仲裁 + 超时 + motion gate
       → MotionControl_StepWithInput 每 10ms，motorTask 驱动
         → MotorHardwareLayout         电机启用/侧别/方向查表
         → 差速模型              linear_x / angular_z → left_mps / right_mps
         → 速度斜坡              按 CHASSIS_SPEED_RAMP_MPS2 平滑过渡
         → StraightLineController 方向 trim + 轮速耦合 + gyro PI (angular_z=0 自动激活)
         → PID 速度环            独立四路 PID 控制器 (M1-M3 PID, M4 纯 P)
         → permille 输出          限幅到 CHASSIS_PWM_MAX_PERMILLE
         → MotorDriver           TIM1 EN/PWM + GPIO PH/DIR 输出
```

---

## 2. 控制模式与来源仲裁

App 先将当前 mode mask 与 Safety capability mask 求交，Command Management 只在允许集合中仲裁：

| 模式 | 允许来源 | 说明 |
| --- | --- | --- |
| `DISABLED` | 无 | 安全降级或 MANUAL 期间 PS2 断联 |
| `AUTO` | `COMMAND_SOURCE_HOST`、`COMMAND_SOURCE_ESP12F` | 远程自动控制；Host 优先于 ESP12F |
| `MANUAL` | `COMMAND_SOURCE_PS2` | PS2 去抖接管 |
| `LINE` | `COMMAND_SOURCE_LINE` | 显式进入并启用巡线 |
| `MAINTENANCE` | `COMMAND_SOURCE_DEBUG` | App 维护编排持锁期间的诊断来源 |

### 2.1 仲裁逻辑

`CommandManagement_GetActive()` 按优先级数组 `{HOST, PS2, ESP12F, LINE, DEBUG}` 顺序遍历各源命令槽：

1. 检查源命令的 `enable` 标志
2. 检查 mode/capability mask 与 rearm 状态，并验证命令年龄：Host 为 200ms，PS2/ESP12F 为 500ms，LINE 为 50ms，DEBUG 为 2000ms
3. 命中即返回，不再检查更低优先级源
4. 若所有源均未命中，返回空命令（`enable=0`），底盘停止

### 2.2 源过期机制

本地 PS2/Line 每次采样提交新 intent。Host/ESP duplicate 只有通过 Communication 的 session/sequence/payload
校验，并匹配 CommandManagement 的 slot/command/mode/revoke token 时才能刷新；超时或超过原始 2000 ms
最大寿命的槽不能复活。

### 2.3 五模式与 PS2 接管

模式为 `DISABLED / MANUAL / AUTO / LINE / MAINTENANCE`，App `ControlModeCoordinator` 发布只读快照。
PS2 摇杆超过 0.15 且连续 3 个有效样本进入 MANUAL，低于 0.10 才退出接管判定；D-pad 与定角宏按边沿触发。
进入 MANUAL 会清除 AUTO/LINE 已应用命令并要求来源 rearm。全部摇杆回中且无 D-pad/宏持续 2 s 后只恢复
AUTO/LINE 模式，机器人仍为 STANDBY；远程或巡线重新完成 neutral→active 后才能运动。接管期间 PS2 断联进入 DISABLED。
LINE 恢复事件只提交 neutral；必须等待之后独立的显式 LINE enable 操作，不能在同一恢复事件中自动 active。
每个 Safety 周期先把 capability source mask 与 mode mask 求交并原子应用，再执行命令选择；能力恢复仍保留 rearm。

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

Safety 每 20 ms 发布有效期 40 ms 的 permit。Motor 每 10 ms 检查 generation 与无符号年龄；
generation 变化时先复位控制状态，`age > 40 ms` 时立即安全停车。该机制保证 Safety task stall 后
PWM 在 IWDG 复位前归零。

### 3.2 命令拒绝规则

`CommandManagement_Set()` 在以下任一条件时拒绝命令：

- Safety motion gate 关闭，或来源不在当前 mode/capability mask 中
- 源 ID 非法（`COMMAND_SOURCE_NONE` 或超出 `COMMAND_SOURCE_COUNT`）
- 来源尚未完成 rearm，或 generation 已过期
- 速度值为 NaN 或 Inf → **额外触发 reject-and-stop**：拒绝并清空该源命令槽
- 命令 `enable == 0` → 清空该源槽位
- 运动学无效（轮半径/轴距 ≤ 0）且角速度非零 → 拒绝角速度，直线仍允许

clamping 规则：`linear_x` 钳位到 `±CHASSIS_MAX_LINEAR_MPS`（0.5 m/s），`angular_z` 钳位到 `±CHASSIS_MAX_ANGULAR_RPS`（10 rad/s）。

### 3.3 巡线特有安全行为

- **默认关闭**：App 注入的 `line.default_enabled = 0U`，上电不启用
- **丢线退让**：8 路全白（无黑线）→ release LINE source → 停止
- **超时退让**：传感器 50ms 无有效帧 → release LINE source
- **手动接管**：PS2 达到接管条件后 App 切换 MANUAL，撤销 LINE；恢复 LINE 模式后仍需新的显式 enable

---

### 3.3 P0 运动安全

- Parameter Management 发布 generation 快照；App 在同一 Motor 周期组装 `motion_control_input_t`，
  Motion Control 与状态估计消费同代参数，代际变化时清 PID、斜坡和直行控制状态。电机/编码器方向
  由 `App/config/robot_config.c` 提供 factory defaults，并可通过 `set motor_dir` / `set encoder_dir`
  整包更新运行时快照。
- 维护锁先阻止新命令，再清控制源/raw/open-loop/PWM 并确认编码器静止。raw/open-loop 租约 400ms；Release 还需本地 `maint arm` 60s 授权。
- 维护、ESTOP 和 fault-stop 首次置位都会递增 motion-revoke generation；此前启用的 LINE 和自动续发 DEBUG `vel` 会失效，解除安全锁后必须重新执行 `line on` 或发送新的本地 `vel`，不会恢复旧运动模式。
- 正常闭环任一 enabled encoder 无效，或 RUN/PWM 有效时非零 requested target 持续 150ms 无运动，当前 motorTask 周期立即整车停车并锁存 bit17。
- PS2 非巡线中位提交零速；巡线中位让权给 LINE。L1/R1=±90°，L2/R2=±360°，使用 IMU yaw 增量累计。
- 航向宏仅积分新的 IMU 样本，重复快照不推进角度；运行期 IMU 离线、未标定、质量失效或超过新鲜度上限时立即取消并提交零速，恢复后需要新的按键边沿。
- 首次 IMU 标定自动保存等待底盘已静止，再获取维护锁并复查；运动中或 Motor 周期进行中不撤销命令，获取失败退避 1s。显式手动维护仍先主动停车再确认静止。
- 巡线禁用期间仍发布传感器有效性；不影响当前模式允许来源的 capability 变化不撤销命令，来源能力恢复继续要求显式 rearm。

## 4. FreeRTOS 任务模型

### 4.1 任务表

| 任务 | 入口函数 | 优先级 | 周期 | 调度方式 | 栈大小 | 核心职责 |
| --- | --- | --- | --- | --- | --- | --- |
| **safetyTask** | `Task_Safety` | High (osPriorityHigh) | 20ms | Platform 周期延时 | 4096B | App 收集事实、Safety Evaluate/permit、快照发布、提交 Safety completion 与看门狗门控 |
| **motorTask** | `Task_MotorControl` | AboveNormal | 10ms | Platform 周期延时 | 768W (3072B) | 状态估计、电源静止更新、Motion DTO/事件、提交 Motor completion；闭环实测保留栈裕量 |
| **rpiCommTask** | `Task_RpiComm` | Normal | 5ms | `osDelayUntil` | 512W (2048B) | `HostCommunication_Update`：USART3 上位机协议收发 |
| **imuTask** | `Task_Imu` | Normal | DRDY 优先 / 10ms 超时降级 | `osThreadFlagsWait` | 512W (2048B) | BMI270 INT1 唤醒后读取 FIFO/SensorTime；超时走直读轮询降级 |
| **ps2Task** | `Task_Ps2` | Normal | 20ms | `osDelayUntil` | 512W (2048B) | `PlatformResetTrace_TaskHeartbeat` + `Ps2Control_Update`：PS2 手柄数据读取 + 巡线切换检测 |
| **lineTask** | `Task_Line` | BelowNormal | 5ms | `osDelayUntil` | 1024W (4096B) | `LineSensorDriver_Update` + `LineSensorDriver_RequestAnalog` + `LineFollowing_Update`：帧解析、传感器查询、PD 控制提交 |
| **espTask** | `Task_Esp12f` | BelowNormal | 5ms | `osDelayUntil` | 512W (2048B) | `PlatformResetTrace_TaskHeartbeat` + `Esp12fFlashBridge_Update` + `WirelessCommunication_Update`：ESP12F 协议收发与烧录桥 |
| **debugTask** | `Task_Debug` | BelowNormal | 10ms | `osDelay` | 2048W (8192B) | `PlatformResetTrace_TaskHeartbeat` + 命令解析、2 Hz CSV 日志、`vel` 指令刷新 |
| **ledTask** | `Task_Led` | Low (osPriorityLow) | 50ms | `osDelayUntil` | 128W (512B) | `StatusLedDriver_TaskStep`：TEST_LED 状态闪烁 |
| **oledTask** | `Task_Oled` | Low (osPriorityLow) | 250ms | `osDelayUntil` | 512W (2048B) | `OLED_UI_Update`：SSD1306 OLED 三阶段 UI 刷新（欢迎/自检/运行） |

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

`BSP/motor/internal/motor_hardware_layout_config.h` 提供编译期 enabled/side 布局；电机方向、
编码器方向、轮径和轮距的 factory defaults 位于 `App/config/robot_config.c`，运行时由 Parameter
Management 发布：

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

- 电机/编码器方向、巡线阈值/极性/PD/速度、电流三层阈值与直行补偿增益均由 Parameter Management 原子快照驱动；编译期宏只保留硬件布局事实。
- 所有来源的 `angular_z=0` 在 Motion Control 中统一进入 `StraightLineController`。控制器按前进/后退和 0.15/0.30m/s 插值 trim，再叠加轮速耦合与 gyro-z 积分航向 PI；不等待 400ms，也不使用 Mahony yaw。
- IMU stale、未校准、SPI/timestamp/gyro quality 异常时同周期退化为 trim + wheel，恢复周期清零航向参考，禁止输出阶跃。纠偏/PWM/电流饱和时冻结积分；弱侧无法增速时降低共同速度以保留差速。
- 默认 trim、Kp、Ki 和航向保持关闭，必须用匹配固件身份的双向 2m 基线/改后报告决定参数。前 0.30m 万向轮翻转段单独记录，不计入稳态验收。
