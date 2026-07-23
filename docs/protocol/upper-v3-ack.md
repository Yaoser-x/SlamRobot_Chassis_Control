# Upper Protocol v3 ACK

ACK 位：`SESSION_VALID=0x01`、`RECEIVED=0x02`、`APPLIED=0x04`、`DUPLICATE=0x08`、`REJECTED=0x10`。STATUS 的 session/sequence/ACK 只描述当前发送链路。

`RECEIVED` 不等于 `APPLIED`。SET_VELOCITY 的 APPLIED 表示 CommandManagement 接受目标或接受 rearm disable。安全恢复后未先 rearm 的 enable 必须为 `REJECTED`，gate 关闭时原因是 `FAULT`。

CLEAR_FAULT 与 LINE_CTRL 在 v3 没有业务结果字段：收帧只可向调用方报告 `queued`。CLEAR_FAULT 入队不代表锁存已清除，LINE_CTRL 入队不代表巡线已进入目标状态；不得用未定义 bit 或状态延迟推测成功。完整操作结果进入 v4。
