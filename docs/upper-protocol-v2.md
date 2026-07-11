# Upper Protocol v2 契约

USART3 和 ESP12F 共用帧格式：`A5 5A LEN CMD PAYLOAD CRC8`。`LEN` 等于
`1 + payload_length`，CRC 覆盖 `LEN`、`CMD` 和完整 payload。协议版本保持
`2`。

## 远程安全命令

- `0x02 ESTOP` payload 长度仍为 1 B。USART3 和 ESP12F 远程入口只接受非零值作为“置位急停”；0 不解除急停。解除仅能通过本地 USART1 `estop 0`。
- `0x04 CLEAR_FAULT` payload 长度为 0 B。USART3 上位机与 ESP owner 均可申请清除普通锁存故障；STM32 仅在硬件故障条件消失且输出安全时执行。该命令不能解除 ESTOP，结果通过后续 STATUS 的 `latched_error_flags` 确认。
- `STATUS` payload 保持 65 B、整帧 70 B。`ENCODER_FEEDBACK_LOST` bit17 和 `BATTERY_CRITICAL` bit18 复用既有 32-bit `error_flags/latched_error_flags`，不改变帧长。

常用错误位：bit5 ESTOP、bit6 派生 fault-stop、bit8 DRV nFAULT、bit9 TIM1 BKIN、bit17 编码器反馈丢失、bit18 电池严重欠压。客户端应优先展示锁存根因，不应把 bit6 当成独立硬件故障。

## 周期上报

| 命令 | Payload | 整帧 | 周期 | 说明 |
|---|---:|---:|---:|---|
| `0x81 STATUS` | 65 B | 70 B | 50 ms | 控制、安全、电机、编码器和通信健康 |
| `0x83 IMU_STATUS` | 99 B | 104 B | 20 ms | IMU 原始工程量、姿态和质量计数 |

USART3 发送由单一四槽异步队列串行化。STATUS 优先于 IMU；队列满时优先
丢弃 IMU 遥测。周期时间戳只在成功入队后推进，发送完成计数只在 UART TX
完成回调中更新。

## IMU payload 尾部

| Payload 偏移 | 长度 | 类型 | 字段 |
|---:|---:|---|---|
| 0 | 1 | `uint8` | protocol version |
| 1 | 12 | `float32[3]` | acceleration, g |
| 13 | 12 | `float32[3]` | corrected gyro, dps |
| 25 | 12 | `float32[3]` | Euler angle, degree |
| 37 | 16 | `float32[4]` | quaternion w/x/y/z |
| 53 | 16 | `uint32[4]` | timestamp, sensor time, sample count, quality flags |
| 69 | 28 | `uint32[7]` | quality counters |
| 97 | 1 | `uint8` | status flags |
| 98 | 1 | `int8` | actual temperature, degree Celsius |

温度是直接的有符号摄氏度，不使用 `-40` 或其他隐式 offset。BMI270 原始温度
按 `23 + raw / 512` 换算；`0x8000` 表示无效，不更新有效温度。
标定参数中的 `temperature_offset_c` 是温漂参考温度；温度有效时，每轴陀螺
偏置增加 `temperature_gyro_slope_dps_per_c × (T - reference)`。

## 黄金向量

机器可读真源为 [`tests/fixtures/upper_v2_golden.json`](../tests/fixtures/upper_v2_golden.json)，
由固件 `UpperProtocol_BuildStatusPayload()`、`UpperProtocol_BuildImuStatusPayload()`
和 `UpperProtocol_BuildFrame()` 直接生成。Host test 会比较重新生成结果，覆盖
STATUS 以及 23°C、-41°C、87°C 三条 IMU 帧。
