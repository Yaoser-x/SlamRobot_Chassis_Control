# Service 唯一职权与 Upper Protocol v3 边界

rc2 中 `CommandManagement` 独占五来源槽、租约、rearm、permitted mask 和撤销代次；
`SafetyManagement` 独占 ESTOP、fault latch、能力和带租约运动许可；`MotionControl` 独占控制状态和电机输出。
Host/ESP dispatcher 只负责 wire、session、sequence、duplicate payload 和请求校验，业务结果由 App 应用后回填。

所有权表示“唯一保存可写状态并作出决定”，不等于“唯一读取事实”。任何同一状态的第二个可写副本均视为架构缺陷。

| 能力 | 唯一状态/决策所有者 | 明确禁止 |
| --- | --- | --- |
| Wire 编解码 | Communication 纯 codec | Service getter、session 更新、控制调用 |
| USART3 Host | HostCommunication | 调用 Wireless、决定运动许可、读取 Parameter |
| USART2 ESP | WirelessCommunication | 调用 Host、共享 Host session、读取 Parameter |
| Session/sequence/ACK | Communication internal tracker，每链路独立 slot | 租约、优先级、安全和电机调用 |
| 操作请求队列/阶段 | Communication fixed mailbox，每链路独立 | 直接生成业务成功 |
| 命令租约与仲裁 | CommandManagement | wire/session/ACK 语义 |
| ESTOP/fault/能力/运动许可租约 | SafetyManagement | 协议解析、来源优先级和 TX |
| 五模式工作流 | App `ControlModeCoordinator` | Service 直接写模式或保留第二份业务真值 |
| 参数与 identity CRC | ParameterManagement | Communication 依赖和 HELLO 生成 |
| 构建身份 | App identity provider | 运行时业务状态和直接 TX |
| 全车快照收集 | App collector | Set/Clear/Enable 等决策操作 |
| 快照发布 | SystemPublishSnapshot | 主动读取 Service 或重新聚合业务 |
| 遥测编码 | TelemetryEncoder | 编码过程中调用 getter |
| 电机输出 | MotionControl | Communication、Command 或普通 App workflow 直接操作电机 |

## 固定依赖

```text
BSP transport
    ↑
Communication ──→ CommandManagement ──→ ParameterManagement
      │
      └── operation mailbox ──→ App composition ──→ Safety / Line

App composition ──读取 Service 快照──→ immutable publish model
                                         ↓
                              Host / Wireless encoder
```

Communication 不得包含 ParameterManagement 头；effective parameter CRC 只能由 App collector 写入发布 DTO。
Command、Safety、Parameter 不得反向依赖 Communication。Safety/Motion 不得主动拉取兄弟 Service 或跨写业务状态。
Host 与 Wireless 只复用纯 codec/session tracker，互不调用。

session tracker 只判断 velocity wire 合法性；只有 CommandManagement 的执行结果可将序列标记为 APPLIED。ESTOP、CLEAR_FAULT 和 LINE_CTRL 由 Communication 入队，App 在对应 5 ms 链路任务中调用业务 Service。SafetyManagement/LineFollowing 生成业务结果；App 不复制 session、命令或安全状态成为第二个 owner。

## 控制源契约

| 来源 | 优先级 | 超时 |
| --- | ---: | ---: |
| Host | 1 | 200 ms |
| PS2 | 2 | 500 ms |
| ESP12F | 3 | 500 ms |
| Line | 4 | 50 ms |
| Debug | 5 | 2000 ms |

duplicate 只能在 Communication 确认相同 session/sequence/payload 后，使用 CommandManagement
签发的 slot token 刷新未过期租约；token 必须匹配 command id、mode 和 revoke generation，且不能超过
原始最大寿命。关闭 Safety gate 或模式撤销会清空对应来源并要求重新 neutral/disable；MANUAL 期间的
远程帧只更新接收序号，不得写执行槽、刷新租约或完成 rearm。

Safety 每周期发布的能力 mask 必须先与当前 mode mask 求交，再由 CommandManagement 原子应用，之后才能选择
active source。能力下降会立即撤销并清空不再允许的来源；能力恢复只重新允许来源，不自动清除 rearm。

## TX 与中断边界

RX parser/ISR 只能产生 frame/action 事件，不得启动 TX。GET_INFO action 由接收链路设置自己的 pending HELLO。Host 优先级为 STATUS > HELLO > DIAGNOSTIC > IMU，队列满只淘汰最旧 IMU且最多一个待发 IMU；Wireless 为 STATUS > HELLO > DIAGNOSTIC。
