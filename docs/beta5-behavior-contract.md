# Beta5 行为兼容契约

Beta5 是架构整改版本，不是业务功能版本。任何目录、接口、状态所有权或并发实现变化，都必须保持本文冻结的外部行为。

## 1. 基线

- 架构实施起点：`v1.0.0-beta4`（`a4ae10d`）。
- 功能真值：Beta1/Beta2 逐项仲裁后的结果。
- 当前本地 Beta5 分支只作为实现素材，不作为功能真值。
- 用户后续提供的实板日志必须在基础 Service 迁移前登记为阻断验收项。

## 2. Beta1/Beta2 仲裁结果

| ID | 场景 | 固定行为 |
| --- | --- | --- |
| BC-MOTOR-001 | M2/M3 同时反向 | 各电机独立完成制动和 PH settle；允许在同一控制轮切相并等幅爬升 |
| BC-STRAIGHT-001 | 直行速度超过 0.30 m/s | 不截断基础速度；停用直行补偿，清空航向误差和积分 |
| BC-LINE-001 | PS2 双表面标定完成 | 只更新 RAM；永久保存必须显式执行 `set save` |

以上三项采用 Beta2 行为，不得通过兼容层恢复 Beta1 行为。

## 3. 命令和安全

控制源优先级固定为 Host/USART3、PS2、ESP12F、Line Following、Debug/USART1。超时依次固定为 200、500、500、50、2000 ms。

- 高优先级命令失效后，仍未超时的低优先级命令可以恢复。
- ESTOP、fault-stop 或 maintenance lock 必须清空全部来源并增加撤销 generation。
- 安全状态解除后不得恢复安全转换前的旧命令，必须收到新命令才能运动。
- 安全门关闭期间的新命令必须被拒绝。
- 非有限浮点数、禁用命令和无效运动学参数必须保持 reject-and-stop 语义。
- Host 和 ESP12F 的 CLEAR_FAULT 仅在 ESTOP 未激活时生效。

## 4. 电机和运动控制

- 默认两驱为 M2 左、M3 右；电机、电流、编码器和 nFAULT 映射保持不变。
- 换向必须经过制动、EN=0、PH settle 和重新爬升。
- 多电机换向不得通过共享全局间隔互相串行阻塞。
- `straight_max_speed_mps` 只表示补偿有效范围，不是整车速度上限。
- 0.30 m/s 在补偿范围内；0.31 m/s、0.50 m/s 和 -0.50 m/s 不得被截速。
- 退出补偿范围必须清空积分；重新进入范围必须建立新的航向参考。
- 编码器反馈失效或安全门关闭必须停止正常输出并复位控制器状态。
- 电流软限幅及过流锁存的编译开关、公式、阈值和 dry-run 行为保持 Beta4 当前值。

## 5. PS2、巡线和维护流程

- PS2 周期为 20 ms，空闲释放时间为 2000 ms。
- 摇杆死区、方向键速度、±90°/±360° 航向宏、IMU 新鲜度和质量门禁保持不变。
- 人工输入必须取消正在运行的航向宏。
- 巡线丢失、传感器过期和安全 generation 变化必须撤销 Line 来源。
- 巡线标定开始时检查 ESTOP、fault-stop、maintenance lock、PWM 和轮速静止条件。
- Beta5 不新增“采集全程锁车”；开始检查结束后释放维护锁。
- Flash 提交必须重新取得维护锁，失败和成功路径都必须正确释放。
- 调试电机测试继续使用现有授权窗口、超时和安全门禁；不得与参数/标定 maintenance lock 混为同一状态。

## 6. 状态、任务和并发

任务时序固定为：Safety 20 ms/High、Motor 10 ms/AboveNormal、Host 5 ms/Normal、IMU DRDY 且 10 ms 超时/Normal、Line 5 ms/BelowNormal、ESP12F 5 ms/BelowNormal、PS2 20 ms/Normal、LED 50 ms/Low、OLED 100 ms/Low、Debug/BelowNormal。

- 不合并轮速 10 ms 链和 IMU DRDY 链。
- 不新增 odometry、位置 x/y 或机器人位姿积分。
- IMU 原始量、校正量、姿态、sensor time、质量位和标定状态不得丢失。
- 编码器和 IMU 必须保留独立时间戳、有效位和新鲜度判断。
- 发布状态必须是完整 generation；不得依赖 `volatile` 防止撕裂。

## 7. 配置、持久化和通信

- 参数优先级固定为：出厂默认值 `<` Flash `<` 本次运行 RAM。
- 有效运动学轮距默认值为 0.176 m。
- Flash schema 固定为 4，bundle 固定为 340 字节，image 固定为 364 字节。
- schema 1–3 迁移结果及 IMU calibration 保持兼容。
- Beta5 历史基线为 Upper Protocol v2；Beta6 已成套升级为 v3-only，STATUS/DIAGNOSTIC/IMU_STATUS 固定为 92/28/99 字节，最大 payload 保持 99 字节。
- 帧头、命令 ID、校验方式、字段顺序、缩放、饱和、owner/deadman 和发布周期不得改变。

## 8. 自动化证据映射

| 契约 | 当前 Host Test |
| --- | --- |
| 命令优先级、超时、安全撤销 | `f407_v2_host` |
| 同周期切相和换向爬升 | `motor_driver_gpio` |
| 高速直行不截速 | `straight_controller` |
| 巡线 RAM/Flash 分离 | `line_control_service` |
| PS2 空闲释放和航向宏 | `ps2_control_service` |
| 参数、schema 和字节布局 | `flash_param`、`upper_v3_golden` |
| IMU 质量和标定 | `imu_pipeline` |
| 安全锁存和恢复 | `safety_service` |

架构迁移可以更换被测入口，但不得修改本表对应的输入、输出和期望结果。

## 9. 待补实板证据

进入基础 Service 迁移前，每个新增实板现象必须记录固件版本/SHA、构建类型和 dirty 状态、复现命令、完整输出、预期与实际行为、Host 回归或 HIL 项以及修复后的复验结果。
