# Upper Protocol v3 拒绝码

| 值 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | NONE | 无拒绝 |
| 1 | MALFORMED | 长度或字段格式错误 |
| 2 | UNSUPPORTED_VERSION | 版本不是 v3 |
| 3 | NONFINITE | 速度目标为 NaN/Infinity |
| 4 | INVALID_MODE | enable 或 mode 非法 |
| 5 | STALE_SESSION | retired 或过期 session |
| 6 | OUT_OF_ORDER | sequence 倒退、冲突或非前进 |
| 7 | FAULT | Safety gate 关闭 |
| 8 | SOURCE_NOT_PERMITTED | 来源未 rearm 或命令所有者拒绝 |

业务操作的内部阶段固定为 `FRAME_ACCEPTED`、`REQUEST_DISPATCHED`、`BUSINESS_APPLIED`、`BUSINESS_REJECTED`、`CONDITION_NOT_CLEARED`、`TIMEOUT`，但这些阶段不编码进 v3 payload。
