# Upper Protocol v3 帧格式

帧为 `A5 5A | LEN | CMD | PAYLOAD | CRC8/MAXIM`。`LEN` 等于一字节 CMD 加 payload 长度；CRC 覆盖 LEN、CMD 和 PAYLOAD。多字节字段均为 little-endian，最大 payload 99 B。禁止 packed struct、wire 指针强转和依赖 ABI padding。

帧被 CRC、长度和版本校验接受，只表示 `FRAME_ACCEPTED`，不表示对应业务 Service 已应用请求。完整字段以 [`upper-v3.schema.json`](upper-v3.schema.json) 为准。
