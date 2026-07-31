# RC2 审计整改冻结

本轮以 `1667971022d7d32b87cba0020e638c0da8914ef9` 为唯一基线，版本保持
`1.0.0-rc2`。软件整改不改变 `.ioc`、Upper Protocol v3 payload 或 Flash schema 4；
实板验收完成前发布状态保持 `HIL_PENDING`。

## 运行时安全契约

- Motor 与 Safety 只在完整周期结束时原子提交 `generation + completed_at_ms`。
- 看门狗门同时要求 Motor 20 ms、Safety 40 ms 内各有未消费的完成代次；
  `Evaluate` 不修改状态，只有实际喂狗后才 `CommitFeed`。IWDG 名义超时按 800 ms 校验。
- Safety 发布 40 ms 运动许可租约。Motor 每 10 ms 独立检查许可位、年龄和 generation；
  `age == 40 ms` 仍有效，超过边界立即将 PWM 归零并复位控制状态，不等待 IWDG。
- POST 分为 fatal 与 degraded。参数、电流基础、编码器、驱动、TIM Break 和任务健康决定
  `base_motion`；IMU 降级只关闭 heading 能力。rc2 的远程速度 IMU 策略为只读产品配置
  `remote_velocity_requires_imu=0`，不进入持久化参数。

## 控制模式与命令契约

模式固定为 `DISABLED / MANUAL / AUTO / LINE / MAINTENANCE`。App 负责模式工作流；
CommandManagement 独占来源槽、租约、rearm、permitted mask 和最终仲裁。

PS2 摇杆以 0.15 进入阈值、0.10 退出阈值和连续 3 个 20 ms 有效样本确认接管；
D-pad/宏使用边沿。进入 MANUAL 会清空 AUTO/LINE 已应用命令并要求被抢占来源 rearm。
回中 2 s 只恢复目标模式，不恢复速度。AUTO/LINE 必须重新执行 neutral/disable 后再提交
新的 active intent；PS2 在接管期间断联则进入 DISABLED。

MANUAL 中仍更新远程 session 与 `last_received_sequence`，但 active/neutral 均不写执行槽、
不刷新租约且不预先完成 rearm。ACK 为 `RECEIVED + REJECTED`、`APPLIED=0`，原因
`SOURCE_NOT_PERMITTED`。退出后 disable 与 enable 的 sequence 必须大于 MANUAL 期间最新接收值。

Communication 独占 wire session、sequence 和 duplicate payload 比较；CommandManagement 的
refresh token 只绑定 source、slot generation、accepted command id、规范化 payload、mode 和
revoke generation。旧 token、超时槽或超过原始最大寿命的 duplicate 均不能续租。

## 数据与参数契约

- `encoder_count` 是方向校正后的 `uint32_t` 模 2^32 原始累计位模式，codec 逐字节 little-endian
  输出；轮速拒绝 spike 不得改变累计语义。连续 3 个稳定样本只重建 baseline，不补算异常区间。
- 编码器异常使用 Host 发送完成 ACK latch。旧 STATUS completion 不能清除更新 generation 的新异常；
  ESP 可观察但不能清除 Host latch。
- 控制 timing 从 nominal period 派生。EARLY 不推进 accepted baseline；NORMAL/LATE 使用真实 dt；
  MISSED 更新 baseline、输出零并复位 PI 和目标整形。
- 四轮 effective `pid_kd` 固定为零。旧 schema 4 Flash 的非零值在加载后规范化但不自动写回；
  HELLO `parameter_crc32` 始终表示 effective model。诊断同时保留 persisted/effective CRC 和差异标志。

## 所有权数据流

```text
App 收集带时间、generation、validity、quality 的事实 DTO
  -> Safety Evaluate / 发布能力与许可租约
  -> ModeCoordinator 计算来源 mask
  -> CommandManagement 原子应用 mask 并仲裁
  -> MotionControl 消费 DTO、写电机并返回 MotionEvent
  -> App 将事件路由回 Safety
```

Safety 与 Motion 不再主动拉取兄弟 Service 或跨写业务状态。维护锁、故障清除和 BSP 电机事实
由 App composition/adapters 编排；Release 不包含运行时故障注入接口。

## 验证状态

最终本地 `main` 已重新执行 62/62 Host 测试；ASan/UBSan 为 61/61（静态栈预算项在普通 Host
独立通过）；Debug/Release、架构、命名、Service ownership、CubeMX、cppcheck 2.21.0（416 个
C/H 文件）和 clang-format 18.1.8 通过。arduino-cli 1.5.1 使用 ESP8266 Core 3.1.2 与
WebSockets 2.7.2 完成编译和安全检查；Upper 固定提交 `c472bc6` 的联合 fixture 40/40 通过。
Debug size 为 `text=189212, data=532, bss=90264`，Release 为
`text=168876, data=528, bss=90248`。HIL 项始终未据软件测试提前勾选。
