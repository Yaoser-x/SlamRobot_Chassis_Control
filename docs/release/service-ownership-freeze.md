# Service 所有权冻结

- Communication：字节、帧、双链路 session/sequence/ACK，以及固定容量 operation mailbox；不生成业务成功。
- CommandManagement：Host、PS2、ESP、Line、Debug 五来源的唯一命令所有者和最终优先级仲裁者。
- SafetyManagement：ESTOP、fault-stop、维护锁、运动许可和 CLEAR_FAULT 业务结果。
- MotionControl：目标整形、差速目标、闭环控制和电机输出。
- StateEstimation/Measurement：编码器、IMU、质量与有效性事实。
- ParameterManagement：运行参数、schema 和 identity CRC。
- LineFollowing：巡线启停业务结果和巡线目标。
- App：启动和跨 Service 编排；不复制 Service 状态，也不伪造业务成功。

优先级固定为 `Host > PS2 > ESP > Line > Debug`。Safety gate 关闭时 CommandManagement 在同一临界区清空五来源并令 Host/ESP 必须重新 disable；Motion 的清目标、整形器、PID、反馈保护和直行修正复位链路保持既有单写者边界。

操作请求阶段固定为 FRAME_ACCEPTED、REQUEST_DISPATCHED、BUSINESS_APPLIED、BUSINESS_REJECTED、CONDITION_NOT_CLEARED、TIMEOUT。Communication 入队，App 调用业务 Service，最终结果只由业务 Service 的返回值和状态产生。
