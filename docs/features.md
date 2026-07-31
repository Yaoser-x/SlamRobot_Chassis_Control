# F407 V2.0 当前功能总览

> 固件版本：`1.0.0-rc2`
>
> 发布类型：`Pre-release`
>
> 软件状态：`FEATURE_FROZEN`
> 硬件状态：`HIL_PENDING`

本文只描述当前 `main@bc715ff` 的有效能力。历史 Beta 行为仲裁见
[Beta5 行为兼容契约](beta5-behavior-contract.md)，精确 wire 字段见
[Upper Protocol v3 schema](protocol/upper-v3.schema.json)，发布进度见
[Roadmap](ROADMAP.md)。

## 1. 平台与软件分层

### 1.1 硬件平台

| 项目 | 当前配置 |
| --- | --- |
| MCU | STM32F407VET6，Cortex-M4F，168 MHz |
| RTOS | FreeRTOS 10.3.1 + CMSIS-RTOS2 |
| 电机 | DRV8874 ×4，TIM1 PWM，默认启用 M2 左/M3 右 |
| 编码器 | TIM2/TIM4/TIM3/TIM5 对应逻辑 M1/M2/M3/M4 |
| 电流/电池 | ADC1 五通道，TIM8 TRGO 约 2 kHz，DMA2 Stream0 |
| IMU | BMI270，SPI2 + INT1，FIFO/直接采样 |
| 通信 | USART3 Host、USART2 ESP12F、USART1 Debug、UART4 巡线 |
| 显示 | SSD1306 128×64 OLED / I2C1 |
| 看门狗 | IWDG 名义约 800 ms；Motor/Safety 双 completion 门控 |
| 硬件保护 | TIM1/TIM8 BKIN 低有效，Automatic Output + MOE/BIF 诊断 |

完整引脚、DMA、定时器和逻辑映射见 [外设资源](peripherals.md)。

### 1.2 五层架构

```text
App/        任务入口、产品配置、跨 Service 工作流、显示和调试适配
Service/    Command/Safety/Motion/State/Power/Parameter/Communication 等能力所有者
Algorithm/  差速运动学、PID、直行控制、编码器/IMU 估计与标定纯算法
BSP/        电机、编码器、ADC、IMU、Flash、UART、PS2、巡线和 OLED 驱动
Platform/   临界区、时间、复位追踪、看门狗和系统事件
```

`Core/` 是 CubeMX 生成入口，`Drivers/` 与 `Middlewares/` 是第三方代码。生产树不存在业务
`Domain/` 层。最终静态目标为 `f407_algorithm / f407_platform / f407_bsp / f407_service /
f407_app / f407_app_adapters`。

### 1.3 任务模型

系统保留 10 个业务任务：Safety 20 ms、Motor 10 ms、Host/Line/ESP 5 ms、PS2 20 ms、
LED 50 ms、OLED 100 ms、Debug 10 ms，以及事件驱动的 IMU 任务。任务数量、周期、优先级和
`.ioc` 在 RC2 中冻结。

## 2. App 编排与唯一所有权

### 2.1 正常周期

```text
App 收集硬件和 Service 快照，形成带 timestamp/generation/validity/quality 的 DTO
  -> SafetyManagement 评估能力并发布 40 ms motion permit
  -> ControlModeCoordinator 生成当前模式允许的来源 mask
  -> CommandManagement 原子应用 mask、rearm 和最终优先级仲裁
  -> MotionControl 消费单份输入 DTO、执行控制并返回 MotionEvent
  -> App 把 MotionEvent 路由回 Safety，并发布一致系统快照
```

Safety 不拉取 Power/State/System/Motor/Command，也不直接写 Command gate；Motion 不拉取
Safety/Command/Power/State/Parameter，也不写 Safety fault。BSP 电机事实由 App adapter 转换，
维护、故障清除和远程操作由 App composition 按固定顺序编排。

### 2.2 Watchdog completion

- Motor 和 Safety 只在完整周期结束后提交 `generation + completed_at_ms`；
- App 先执行纯 Evaluate，只有实际喂狗后才 Commit 已消费代次；
- Motor completion 年龄上限 20 ms，Safety completion 年龄上限 40 ms；
- Safety permit 超过 40 ms 时 Motor 先归零输出，不等待 IWDG 复位；
- 任务入口 heartbeat 只用于诊断，不能代表周期完成。

## 3. 运动控制

### 3.1 差速、目标整形与 PID

`Algorithm/kinematics/differential_drive_kinematics.c` 负责：

```text
v_left  = linear_x - angular_z * track_width / 2
v_right = linear_x + angular_z * track_width / 2
```

非有限输入、非法轮距和非法周期直接产生无效结果。控制 timing 按 nominal period 分类为
FIRST/EARLY/NORMAL/LATE/MISSED：EARLY 不推进 accepted baseline；NORMAL/LATE 使用真实 dt；
MISSED 更新 baseline、输出零并复位 PI 与目标整形。

`MotionControl` 负责目标斜坡、换向状态机、左右轮等比缩放、每电机闭环和唯一 PWM 输出。
RC2 的 effective 参数强制四轮 `pid_kd=0`；旧 schema 4 中的非零 Kd 只在 RAM 模型规范化，
不会隐式写回 Flash。

### 3.2 直行控制

`Algorithm/control/straight_line_controller.c` 叠加：

- 前进/后退 0.15/0.30 m/s 两点 trim；
- 左右轮速度交叉耦合；
- gyro-z 航向 PI；
- PWM 饱和或电流限制时冻结积分并降级。

`straight_max_speed_mps` 只限定补偿适用范围，不截断底盘基础速度。IMU 不可用时退化为
trim+coupling；换向、停车、非零角速度、来源变化或 motion revoke 时复位状态。

### 3.3 编码器质量与反馈保护

- `encoder_count` 是方向校正后的 `uint32_t` 模 2^32 原始累计位模式；
- 轮速状态为 UNINITIALIZED/VALID/TRANSIENT_REJECT/REACQUIRING/STALE/FAULT；
- spike 拒绝不改变累计 count；连续三个稳定样本只重建 baseline，不补算异常区间；
- 任一启用轮在非零请求下反馈失效会产生 MotionEvent，由 App 路由为锁存安全故障；
- Host delivery latch 防止旧 STATUS completion 清除新一代编码器异常。

### 3.4 电流保护和诊断运动

CurrentGuard 支持 observe、soft-limit 和 fault-latch 三层策略；正式默认值以
`App/config/robot_config.c` 和 effective 参数模型为准。快速过流仍由 DRV8874 硬件路径承担，
软件阈值需要 HIL 标定。

Debug raw/open-loop 最多 300‰、租约 400 ms，并要求驱动、TIM Break、电流基础、ESTOP/fault
和显式维护授权均有效。Release 不提供运行时故障注入入口。

## 4. 模式、来源与安全

### 4.1 五模式

模式固定为：

- `DISABLED`：普通运动来源全部关闭；
- `MANUAL`：PS2 人工接管；
- `AUTO`：Host/ESP/Debug 等自动来源按策略仲裁；
- `LINE`：巡线来源显式启用；
- `MAINTENANCE`：参数、标定、烧录桥和受限诊断工作流。

PS2 摇杆使用 0.15 进入、0.10 退出阈值以及连续三个 20 ms 有效样本确认接管。回中 2 s
只恢复目标模式，不恢复旧速度；PS2 接管期间断联进入 DISABLED。

### 4.2 来源仲裁和 rearm

固定来源优先级为 Host > PS2 > ESP > Line > Debug，但只在当前模式与 Safety capability
共同允许的 mask 中仲裁。租约分别为 Host 200 ms、PS2 500 ms、ESP 500 ms、Line 50 ms、
Debug 2000 ms。

Safety gate、模式撤销或人工接管会清空受影响来源。AUTO/LINE 恢复后必须重新执行
neutral/disable→active；能力恢复只重新允许来源，不自动完成 rearm。MANUAL 中远程帧可推进
wire session/received sequence，但不能写执行槽、刷新租约或预先完成 rearm。

### 4.3 ESTOP、fault 与维护

ESTOP 可由本地调试、Host 或 ESP 置位；远程不能清除。普通 fault 只有物理条件消失且
Safety clear input 合法时才能清除，旧命令不会恢复。

`AppMotionMaintenance_Begin/End` 编排模式切换、Safety 维护锁、控制器复位、测试租约取消和
所有启用轮静止证明。参数保存/复位、IMU/巡线标定和 ESP 烧录桥均复用该工作流。

## 5. 传感器与外设

### 5.1 ADC、电流和电池

ADC1 由 TIM8 TRGO 约 2 kHz 触发五通道 DMA。`PowerAdcDriver_Update()` 每 20 ms 汇总窗口，
输出 mean/RMS/peak、零点跨度、样本率和质量标志；电池使用独立 EMA。启动零点校准完成前
电流能力保持 fail-closed。

### 5.2 BMI270

BMI270 支持 FIFO/直接采样、坐标映射、滤波、Mahony 姿态、自动/手动标定和质量计数。
IMU 故障默认只撤销 heading capability；是否要求 IMU 才允许远程速度由只读产品配置决定，
不进入 Flash schema 4。

`scripts/analyze_imu.py` 输出 gyro rate OADEV、ARW/BI/RRW、缺口/抖动检查、静止漂移、温度和
六面证据；工具不自动回写参数。

### 5.3 PS2、巡线、OLED 与 ESP12F

- PS2 使用 DWT 时序 GPIO 驱动，提供摇杆、按键边沿、人工接管和相对航向宏；
- HiWonder 八路巡线支持逐通道阈值/极性、双表面 RAM 标定、PD 控制和丢线停车；
- OLED 展示启动、自检、运行、故障和标定状态；
- ESP12F 使用首次配置 AP、WebSocket owner/observer、heartbeat 和 STATUS stale 停车策略，
  与 Host 共用 Upper v3 wire codec，但保留独立 session/sequence。

## 6. Upper Protocol v3

帧格式为 `A5 5A | LEN | CMD | PAYLOAD | CRC8/MAXIM`，最大 payload 99 B，多字节字段均为
little-endian。固定 payload 如下：

| 消息 | 长度 | 方向 |
| --- | --- | --- |
| SET_VELOCITY | 23 B | 上位机→固件 |
| ESTOP | 2 B | 上位机→固件 |
| LINE_CTRL | 2 B | 上位机→固件 |
| CLEAR_FAULT | 1 B | 上位机→固件 |
| GET_INFO | 1 B | 上位机→固件 |
| HELLO | 34 B | 固件→上位机 |
| STATUS | 92 B | 固件→上位机 |
| DIAGNOSTIC | 28 B | 固件→上位机 |
| IMU_STATUS | 99 B | 固件→上位机 |

v3 不传输 encoder-valid、周期 delta、wheel generation 或完整业务结果，这些属于潜在 v4 输入。
STATUS 的 `encoder_count` 保持原始累计语义。精确 offset、范围、无效值和 owner 只以
`docs/protocol/upper-v3.schema.json` 为准。

Host/ESP 各自拥有 session、sequence、duplicate payload 和 ACK 状态。Command refresh token
还绑定 source、slot generation、command id、规范化 payload、mode 和 revoke generation，不能
续活超时或被撤销的旧命令。

## 7. 参数与持久化

`ParameterManagement` 是运行时参数和 identity CRC 的唯一 owner：

- App 注入 factory defaults，再按配置加载 Flash；
- RAM 更新只递增 generation，只有显式保存才写 Flash；
- Flash schema 4 使用 A/B 双槽、CRC、sequence 和最后提交 marker；
- schema 1–3 显式迁移到 schema 4；
- HELLO 报告 effective parameter CRC，诊断同时保留 persisted CRC 和 mismatch；
- 参数保存通过 App 维护工作流证明 PWM=0 和轮速静止。

模式、Safety permit、PS2 takeover 等 RC2 产品策略是非持久化 App 配置，不占用 schema 4
reserved 字段。

## 8. 调试、诊断与分析工具

USART1 提供 version/status/rtos/resettrace、运动与维护命令、参数导入导出、IMU/ADC/电流、
ESP 烧录桥、巡线和 CSV/JSON 遥测。完整命令见 [调试命令台](debug-console.md)。

主要只读/离线工具：

| 工具 | 用途 |
| --- | --- |
| `scripts/hil_smoke.py` | USART1 启动与只读诊断冒烟 |
| `scripts/hil_upper_v3.py` | Host UART v3 HIL |
| `scripts/hil_esp_v3.py` | ESP v3 HIL |
| `scripts/hil_dual_link.py` | 双链路 HIL |
| `scripts/analyze_control.py` | 上升时间、稳态误差、超调、饱和和左右轮差 |
| `scripts/analyze_straight_hil.py` | 直行四工况分析与 RAM-only 调参建议 |
| `scripts/analyze_imu.py` | IMU/OADEV/温漂/六面证据 |
| `scripts/package_firmware_release.py` | clean tag 对应的发布包、manifest 和 SHA256 |

## 9. 测试与 CI

当前 fresh CTest 共 62 项，以 `tests/host/CMakeLists.txt` 和 `ctest -N` 为唯一数量真源：

| 覆盖组 | 代表性内容 |
| --- | --- |
| 安全与模式 | watchdog completion、Safety permit/capability、五模式、rearm、POST、CurrentGuard |
| 控制与电机 | 差速、timing、直行、PID、换向、布局、反馈丢失、维护和诊断租约 |
| 通信 | v3 codec/schema/golden、session、dispatcher、mailbox、Host/ESP transport、publish model |
| 传感器 | ADC/电池、编码器质量、BMI270、IMU pipeline/quality、巡线、PS2 |
| 参数与 UI | schema 迁移、effective CRC、App config、OLED/self-check、Debug console |
| 工具 | Roadmap/IMU/直行/控制分析、架构/命名/所有权和发布元数据纯函数 |

软件门禁状态：Host 62/62；ASan/UBSan 61/61；Debug `text=189212,data=532,bss=90264`；
Release `text=168876,data=528,bss=90248`；cppcheck、clang-format、架构、命名、Service ownership、
CubeMX 安全和 ESP8266 固定工具链检查均通过。

这些结果不代表真实 UART HIL、故障注入、标定或实车验收完成。RC2 可作为
`Pre-release / HIL_PENDING` 发布；`v1.0.0` 仍由 [Roadmap](ROADMAP.md) 的硬件门阻塞。
