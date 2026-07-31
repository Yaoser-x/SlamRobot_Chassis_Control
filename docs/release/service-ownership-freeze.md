# Service 所有权冻结

- Communication：字节、帧、双链路 session/sequence/ACK，以及固定容量 operation mailbox；不生成业务成功。
- CommandManagement：Host、PS2、ESP、Line、Debug 五来源的唯一命令所有者和最终优先级仲裁者。
- SafetyManagement：ESTOP、fault-stop、能力矩阵、带租约运动许可和 CLEAR_FAULT 业务结果。
- MotionControl：目标整形、差速目标、闭环控制和电机输出。
- StateEstimation/Measurement：编码器、IMU、质量与有效性事实。
- ParameterManagement：运行参数、schema 和 identity CRC。
- LineFollowing：巡线启停业务结果和巡线目标。
- App：启动、五模式工作流、维护锁流程、事实 DTO 收集和跨 Service 事件路由；不复制 Service 状态，也不伪造业务成功。

优先级固定为 `Host > PS2 > ESP > Line > Debug`，但只在当前 mode permitted mask 内仲裁。
Safety gate 关闭时 CommandManagement 在同一临界区清空五来源并令 Host/ESP 必须重新 disable；
MANUAL 恢复只恢复模式，不恢复命令。Motion 的清目标、整形器、PID、反馈保护和直行修正复位链路保持单写者边界。

Safety 与 Motion 的公开入口只消费 App 构造的 DTO；Safety 不写 Command gate，Motion 不写 Safety fault。
维护锁和 CLEAR_FAULT 的物理事实由 App adapter 收集并按固定顺序提交。

操作请求阶段固定为 FRAME_ACCEPTED、REQUEST_DISPATCHED、BUSINESS_APPLIED、BUSINESS_REJECTED、CONDITION_NOT_CLEARED、TIMEOUT。Communication 入队，App 调用业务 Service，最终结果只由业务 Service 的返回值和状态产生。
