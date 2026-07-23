# Upper Protocol v3 状态位

STATUS flags：`ESTOP=0x01`、`FAULT_STOP=0x02`、`LINE_ENABLED=0x04`、`SPEED_VALID_ALL=0x08`。未定义 bit 必须忽略，不能承载隐式业务结果。

`SPEED_VALID_ALL` 表示所有启用轮满足当前速度有效契约；上位机仍须同时消费 `motor_enabled_mask` 和 `motor_speed_valid_mask`。禁用轮不得置 speed-valid，也不得触发无反馈故障。单侧至少一个启用且有效的轮，才可形成该侧运动观测。

同一 coherent snapshot 的 STATUS 重发不得推进 `status_sequence` 或 `timestamp_ms`；链路 ACK 可按发送链路注入，但不得改变采样身份。
