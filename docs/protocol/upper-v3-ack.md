# Upper Protocol v3 ACK

ACK 位：`SESSION_VALID=0x01`、`RECEIVED=0x02`、`APPLIED=0x04`、`DUPLICATE=0x08`、`REJECTED=0x10`。STATUS 的 session/sequence/ACK 只描述当前发送链路。

`RECEIVED` 不等于 `APPLIED`。SET_VELOCITY 的 APPLIED 表示 CommandManagement 接受目标或接受 rearm disable。
安全恢复后未先 rearm 的 enable 必须为 `REJECTED`。Safety gate 关闭时原因是 `FAULT`；当前模式不允许来源、
包括 MANUAL 中收到的 Host/ESP active 或 disable，原因为 `SOURCE_NOT_PERMITTED`。

MANUAL 中的新 sequence 仍推进 `last_received_sequence`，ACK 固定为 `RECEIVED=1`、`APPLIED=0`、
`REJECTED=1`、`DUPLICATE=0`。相同 sequence 的旧 keepalive 不刷新执行槽、不完成 rearm，继续报告来源不允许。

CLEAR_FAULT 与 LINE_CTRL 在 v3 没有业务结果字段：收帧只可向调用方报告 `queued`。CLEAR_FAULT 入队不代表锁存已清除，LINE_CTRL 入队不代表巡线已进入目标状态；不得用未定义 bit 或状态延迟推测成功。完整操作结果进入 v4。
