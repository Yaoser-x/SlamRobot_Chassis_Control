# Upper Protocol v3 下位机契约

本固件的 USART3 Host 与 USART2 ESP12F 链路均为 v3-only。配对关系固定为 F407 `1.0.0-beta6`、Upper Protocol v3、Ros2_Slam `v0.4.0`。

## Wire 契约

帧格式为 `A5 5A | LEN | CMD | PAYLOAD | CRC8/MAXIM`。所有多字节值显式 little-endian；禁止 packed struct、wire 指针强转和内存 padding。payload 上限保持 99 B。

| CMD | 名称 | payload |
| ---: | --- | ---: |
| `0x01` | SET_VELOCITY | 23 B |
| `0x02` | ESTOP | 2 B |
| `0x03` | LINE_CTRL | 2 B |
| `0x04` | CLEAR_FAULT | 1 B |
| `0x05` | GET_INFO | 1 B |
| `0x80` | HELLO | 34 B |
| `0x81` | STATUS | 92 B |
| `0x82` | DIAGNOSTIC/schema 1 | 28 B |
| `0x83` | IMU_STATUS | 99 B |

所有 payload 的 offset 0 为版本 3。SET_VELOCITY 依次包含 f32 linear/angular、enable、mode、u64 session 和 u32 sequence。HELLO 包含 capability、原始 20 B Git commit、`0x00020000` hardware revision 和参数 identity CRC32。

## 双链路确认

HOST 与 ESP 各自保存 active/retired session、received/applied sequence、reject 和 ACK；不得共享或互相覆盖。新 session 只能由 disable 建立。相同 sequence 与相同目标是 keepalive；目标不同、倒退或 retired session 均拒绝。回绕按 `0 < new-old < 2^31` 判断。

STATUS 前 65 B 保持 LF/LR/RF/RR 布局，65..91 B 为 snapshot generation、timestamp 和发送链路自己的 session ACK。防重放范围为单次 MCU 运行周期。

## 身份与 fail-closed

clean Git 构建发布 capability `0x1F` 和真实 commit；dirty/无 Git 构建提交全零并清除 BUILD_IDENTITY。ESP 与 Ros2_Slam 在 capability、HELLO、STATUS、release ACK 未全部满足前不得启用运动。

机器金样为 [`tests/fixtures/upper_v3_golden.json`](../tests/fixtures/upper_v3_golden.json)。

## 回滚

回滚必须成套恢复 STM32 beta5.3、ESP v2 与 Ros2_Slam v0.3.0，禁止只回退任一端。
