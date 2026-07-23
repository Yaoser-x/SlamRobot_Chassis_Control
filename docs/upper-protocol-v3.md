# Upper Protocol v3

本页仅作为入口，不再重复定义 wire 事实。机器可读事实源是 [`protocol/upper-v3.schema.json`](protocol/upper-v3.schema.json)。

- [帧格式](protocol/upper-v3-frame.md)
- [消息布局](protocol/upper-v3-message-layout.md)
- [session 与 sequence](protocol/upper-v3-session.md)
- [ACK 语义](protocol/upper-v3-ack.md)
- [状态位](protocol/upper-v3-status-flags.md)
- [错误码](protocol/upper-v3-error-codes.md)

v3 payload 已冻结。任何新增业务结果、单调 session、encoder-valid、周期 delta 或 wheel generation 均进入 v4，不得占用 v3 未定义位。
