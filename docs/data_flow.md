# Beta5 数据流与并发契约

## 控制流

```text
Host / PS2 / ESP12F / Line / Debug
                 |
                 v
        Command Management <----- Safety gate
                 |                    ^
                 v                    |
          Motion Control <--- State / Power / Parameter
                 |
                 v
             Motor BSP
```

Safety 消费 Power、State、System 和外部安全命令，发布运动许可。安全转换先关闭 gate 并清空所有命令，再发布新的 safety generation。Motion 只消费已经通过 gate 的活动命令。

## 状态流

- Encoder BSP 原始计数进入 State 的 10 ms wheel 链。
- IMU BSP 的 DRDY/FIFO 事实进入 State 的事件驱动 IMU 链；10 ms 超时入口继续保留。
- ADC BSP 原始值进入 Power；Power 发布电压、电流、有效性、零点和阈值状态。
- Parameter、wheel、IMU、Power 和 System 均以完整 DTO 发布单调 generation；wheel 与 IMU generation 相互独立。
- App 在短临界区读取各 Service 完整状态，组装 Communication/OLED/Debug 所需读模型。
- Communication 只接收自己的发布 DTO；链路状态和发送事实由 App 回注 System。

Beta5 不创建 odometry、x/y、pose 或位姿融合数据流。

## 发布规则

1. 写端在私有工作区构造完整 `next`。
2. 小 DTO 在短临界区内整体赋值并递增 generation。
3. 大 DTO 使用双缓冲；临界区只切换活动索引并发布 generation。
4. 读端在同一临界区复制完整 DTO 或锁定双缓冲索引。
5. generation 必须单调，时间戳和有效/质量位与 payload 同代发布。
6. 临界区内禁止调用其他 Service、BSP、Platform 外部操作或可能阻塞的函数。
7. `volatile` 只表达可见性，不作为快照同步机制。

轮速和 IMU 使用独立时间戳、有效位、质量位与 generation，不以“统一状态”为由合并调度入口。

迁移期间旧 `ParamService`、`EncoderService`、`ImuService`、`CurrentSensorService`、`TaskHealthService` 和 `ResetReasonService` 仅转发新 owner，不再保存状态。IMU 设备采集管线和 ADC/Encoder 设备工作区仍属于 BSP 实现细节，Service 对外状态不再泄漏 BSP 头类型。
