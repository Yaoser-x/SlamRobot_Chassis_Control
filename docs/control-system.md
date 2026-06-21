# 控制体系

## 1. 控制链数据流

```
控制源 → ControlManager          优先级仲裁 + 超时 + reject-and-stop
       → ChassisControl_Step     每 10ms，motorTask 驱动
         → ChassisLayout         电机启用/侧别/方向查表
         → 差速模型              linear_x / angular_z → left_mps / right_mps
         → 速度斜坡              按 CHASSIS_SPEED_RAMP_MPS2 平滑过渡
         → PID 速度环            独立四路 PI(D) 控制器
         → permille 输出          限幅到 CHASSIS_PWM_MAX_PERMILLE
         → MotorDriver           TIM1/TIM8 PWM 输出
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

`ControlManager_GetCommand()` 按优先级数组 `{UPPER, PS2, ESP12F, LINE, DEBUG}` 顺序遍历各源命令槽：

1. 检查源命令的 `enable` 标志
2. 验证 `timestamp_ms` 是否在 `CHASSIS_CMD_TIMEOUT_MS`（500ms）内
3. 命中即返回，不再检查更低优先级源
4. 若所有源均未命中，返回空命令（`enable=0`），底盘停止

### 2.2 源过期机制

每个控制源需在 500ms 内刷新命令（更新 `timestamp_ms`）。超时未刷新的命令槽自动失效，仲裁循环跳过该源。

---

## 3. 安全语义

### 3.1 安全状态

| 状态 | 触发条件 | 行为 |
| --- | --- | --- |
| **ESTOP (紧急停止)** | `estop 1` 命令 | 清空全部控制源命令，拒绝所有新命令 |
| **fault-stop (故障停止)** | 任一路 DRV8874 `nFAULT` 低有效 | 清空全部命令、停止 PWM 输出、拉低 DRV_SLEEP_ALL |
| **过流锁存** | 电机电流 > `MOTOR_CURRENT_LIMIT_A` 并持续 `MOTOR_OVERCURRENT_DEBOUNCE_COUNT` 次 | 触发 fault-stop，锁存 error flag |
| **RTOS 异常** | `configASSERT` / 栈溢出 / malloc 失败 / 任务创建失败 | 拉低 `DRV_SLEEP_ALL`，进入 fatal loop（不自动复位，保留故障现场） |

### 3.2 命令拒绝规则

`ControlManager_SetCommand()` 在以下任一条件时拒绝命令（`CONTROL_COMMAND_REJECTED`）：

- 当前处于 ESTOP 或 fault-stop 状态
- 源 ID 非法（`NONE` 或超出 `CONTROL_SOURCE_LINE`）
- 速度值为 NaN 或 Inf → **额外触发 reject-and-stop**：拒绝并清空该源命令槽
- 命令 `enable == 0` → 清空该源槽位
- 运动学无效（轮半径/轴距 ≤ 0）且角速度非零 → 拒绝角速度，直线仍允许

clamping 规则：`linear_x` 钳位到 `±CHASSIS_MAX_LINEAR_MPS`（0.5 m/s）。

### 3.3 巡线特有安全行为

- **默认关闭**：`LINE_DEFAULT_ENABLED = 0U`，上电不启用
- **丢线退让**：8 路全白（无黑线）→ `ClearSource(LINE)` → 回退到 DEBUG 或更低源
- **超时退让**：传感器 50ms 无有效帧 → `ClearSource(LINE)`
- **手动覆盖**：PS2 摇杆操作时 LINE 被优先级仲裁自动跳过

---

## 4. FreeRTOS 任务模型

### 4.1 任务表

| 任务 | 入口函数 | 优先级 | 周期 | 调度方式 | 栈大小 | 核心职责 |
| --- | --- | --- | --- | --- | --- | --- |
| **safetyTask** | `Task_Safety` | High (osPriorityHigh) | 20ms | `osDelayUntil` | 256W (1024B) | `SystemMonitor_Update` + `ResetTrace_UpdateControl` + `ResetTrace_TaskHeartbeat`：状态聚合、命令超时检测、fault-stop 锁存、崩溃追踪心跳 |
| **motorTask** | `Task_MotorControl` | AboveNormal | 10ms | `osDelayUntil` | 512W (2048B) | `ResetTrace_TaskHeartbeat` + `EncoderDriver_Update` + `ChassisControl_Step`：编码器刷新、差速+PID+PWM |
| **rpiCommTask** | `Task_RpiComm` | Normal | 5ms | `osDelayUntil` | 512W (2048B) | `UpperUart_Update`：USART3 上位机协议收发 |
| **imuTask** | `Task_Imu` | Normal | 10ms | `osDelayUntil` | 512W (2048B) | `ImuBmi270_Update`：100 Hz 采样、校准、姿态估计 |
| **ps2Task** | `Task_Ps2` | Normal | 20ms | `osDelayUntil` | 512W (2048B) | `ResetTrace_TaskHeartbeat` + `Ps2Control_Update`：PS2 手柄数据读取 + 巡线切换检测 |
| **lineTask** | `Task_Line` | BelowNormal | 5ms | `osDelayUntil` | 256W (1024B) | `LineUart_Update` + `LineUart_RequestAnalog` + `LineControl_Update`：帧解析、传感器查询、P 控制提交 |
| **espTask** | `Task_Esp12f` | BelowNormal | 5ms | `osDelayUntil` | 512W (2048B) | `ResetTrace_TaskHeartbeat` + `Esp12fFlashBridge_Update` + `Esp12fComm_Update`：ESP12F 协议收发与烧录桥 |
| **debugTask** | `Task_Usart1DebugConsole` | BelowNormal | 10ms | `osDelay` | 2048W (8192B) | `ResetTrace_TaskHeartbeat` + 命令解析、2 Hz CSV 日志、`vel` 指令刷新 |
| **ledTask** | `Task_Led` | Low (osPriorityLow) | 50ms | `osDelayUntil` | 128W (512B) | `LedStatus_TaskStep`：TEST_LED 状态闪烁 |
| **oledTask** | `Task_Oled` | Low (osPriorityLow) | 100ms | `osDelayUntil` | 256W (1024B) | `OLED_UI_Update`：SSD1306 OLED 三阶段 UI 刷新（欢迎/自检/运行） |
| **defaultTask** | `StartDefaultTask` | Low (osPriorityLow) | 1ms | `osDelay` | 128W (512B) | 空闲保活（无实质工作） |

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
RTOS heap_free=XXXXXB heap_min=XXXXXB tick=XXXXX
RTOS safety    state=X stack_free=XXXXB missed=X
...
RTOS oled      state=X stack_free=XXXXB missed=X
RTOS comm upper_tx=X upper_drop=X esp_tx=X esp_drop=X
```

- **`heap_free`**：当前 FreeRTOS heap 可用字节；**`heap_min`**：历史最低值
- **`stack_free`**：各任务剩余栈空间（`uxTaskGetStackHighWaterMark`）。持续低于 128–256B 应增大任务栈
- **`missed`**：周期任务执行耗时超过目标唤醒点的累计次数。空载时核心任务 `missed` 不应持续增长
- **`upper_drop` / `esp_drop`**：USART TX 繁忙导致状态帧跳过的次数

---

## 5. 底盘布局配置

`App/chassis/chassis_config.h` 提供编译期布局配置，支持四驱/两驱和自定义左右侧：

| 宏 | 可选值 | 说明 |
| --- | --- | --- |
| `CHASSIS_Mx_ENABLED` | `0U` / `1U` | 电机启用。禁用后目标/PWM/PID/编码器/电流固定为零，nFAULT 不触发整车停机 |
| `CHASSIS_Mx_SIDE` | `MOTOR_SIDE_LEFT` / `MOTOR_SIDE_RIGHT` | 侧别归属，决定接收 `left_mps` 还是 `right_mps` |
| `CHASSIS_Mx_MOTOR_DIR` | `1` / `-1` | PWM 方向修正。`1`=正输出对应前进，`-1`=反向 |
| `CHASSIS_Mx_ENCODER_DIR` | `1` / `-1` | 编码器方向修正。前进时 `status` 速度应为正值 |

**默认布局**（两驱）：M1 左侧，M3 右侧（M2/M4 禁用），M1/M2 encoder dir `+1`，M3/M4 encoder dir `-1`。

**常用两驱**：`CHASSIS_M2_ENABLED=0U, CHASSIS_M4_ENABLED=0U`（仅 M1+M3）或 `CHASSIS_M1_ENABLED=0U, CHASSIS_M3_ENABLED=0U`（仅 M2+M4）。

**V2.0 实板 M3 映射**：逻辑 M3 属于右侧，PWM 使用 `TIM1_CH3/TIM8_CH3` (`PE13/PC8`)，nFAULT 使用 `PD14`，编码器使用 `TIM4 PD12/PD13`，电流采样使用 `PC2`。M2（默认禁用）使用 `TIM1_CH2/TIM8_CH2` (`PE11/PC7`)，nFAULT `PA3`，编码器 `TIM3 PB4/PB5`，电流 `PC1`。

**安全约束**：左右两侧必须各至少启用一路电机，否则 `ChassisLayout` 拒绝运动输出。

---

## 6. 教育科研使用边界

- **raw/open-loop 命令**（`m1`~`m4`、`raw`、`left`、`right`、`motor`）：用于 bring-up 阶段的单轮方向验收、示波器波形测量、硬件排故。受 ESTOP、fault-stop、DRV fault 和过流锁存约束。建议架空轮、低占空比（<300‰）、有人看护
- **`vel` 闭环**：应在电压/电流/编码器方向/DRV fault/raw 方向全部确认后执行，从低速（50 mm/s）开始
- **IMU / PS2 / ESP12F / 巡线**：离线不阻塞底盘实验，状态在 `status` 中可见
- **巡线安全**：默认不启用；需 PS2/ESP12F/调试台显式开启；PS2 摇杆随时可覆盖；丢线自动退让不锁死
- **ESP12F flash bridge**：维护功能。active 期间 USART1 被二进制透传接管，普通调试命令不可用；30s 无活动自动退出
