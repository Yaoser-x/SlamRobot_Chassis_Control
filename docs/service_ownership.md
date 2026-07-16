# Beta5 Service 能力所有权

所有权表示“唯一保存状态并作出决定”，不等于“唯一读取事实”。BSP 发布硬件事实，Algorithm 计算纯结果，App 只编排和组装读模型。

| 能力 | 唯一所有权 | 允许的 Service 依赖 |
| --- | --- | --- |
| Motion Control | 轮速闭环、目标规划、直行补偿、电流限幅、反馈丢失、调试测试、运行时电机输出 | Command、Safety、State、Power、Parameter |
| State Estimation | 编码器计数/轮速、IMU 采样映射/滤波/姿态、质量与新鲜度 | Parameter |
| Power Management | 电压、电流、ADC 有效性、电流零点和原始阈值事实 | State、Parameter |
| Safety Management | ESTOP、fault-stop、maintenance lock、故障锁存/恢复和运动许可 | Power、State、System、Command |
| Command Management | 五来源命令、优先级、超时、活动来源、撤销 generation 和受控 gate | Parameter |
| Teleoperation | PS2 映射、航向宏和人工命令 | State、Line、Command、Parameter |
| Line Following | 巡线解释/控制/丢线和标定流程 | Safety、Command、Parameter |
| Communication | Host/ESP 帧、协议、链路事实和命令路由 | Command、Safety、Line、Parameter |
| Parameter Management | 出厂默认值、Flash/RAM 合并、校验和 generation | 无其他 Service 依赖 |
| System Monitoring | 任务健康、复位、POST 和模块健康事实 | 无其他 Service 读取依赖；事实由 App 注入 |

Algorithm 和 BSP 依赖不计入 Service DAG，但仍受层级边界约束。

## 决策边界

- Safety 决定运动许可，再调用 Command 的 gate 接口；Command 只执行拒绝、清空和 generation 更新。
- 普通控制源抢占可恢复未超时命令；ESTOP、fault-stop、maintenance lock 清空全部命令且解除后不恢复旧命令。
- Motion 是唯一运行时电机输出调用者。硬件 Break 和 FatalStop 在输出链之外直接关断；Motion 初始化只执行安全停车。
- 调试电机测试属于 Motion 内部模式；`maintenance_lock` 只用于标定/保存等维护事务。
- System 只拥有系统健康，不能重新聚合全车业务快照。
- Communication 定义自己的发布 DTO，不能依赖 App 类型；App 组装同一 generation 的 DTO 后注入。

## 固定控制源契约

| 来源 | 优先级 | 超时 |
| --- | ---: | ---: |
| Host | 1 | 200 ms |
| PS2 | 2 | 500 ms |
| ESP12F | 3 | 500 ms |
| Line | 4 | 50 ms |
| Debug | 5 | 2000 ms |

数字越小优先级越高。任何调整都属于行为变更，不能混入 Beta5 架构迁移。
