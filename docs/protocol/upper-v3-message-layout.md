# Upper Protocol v3 消息布局

| CMD | 名称 | Payload | 方向 | 业务所有者 |
| ---: | --- | ---: | --- | --- |
| `0x01` | SET_VELOCITY | 23 B | 上位机→固件 | CommandManagement |
| `0x02` | ESTOP | 2 B | 上位机→固件 | SafetyManagement |
| `0x03` | LINE_CTRL | 2 B | 上位机→固件 | LineFollowing |
| `0x04` | CLEAR_FAULT | 1 B | 上位机→固件 | SafetyManagement |
| `0x05` | GET_INFO | 1 B | 上位机→固件 | Communication |
| `0x80` | HELLO | 34 B | 固件→上位机 | Communication/identity providers |
| `0x81` | STATUS | 92 B | 固件→上位机 | App coherent snapshot |
| `0x82` | DIAGNOSTIC schema 1 | 28 B | 固件→上位机 | 各事实 Service |
| `0x83` | IMU_STATUS | 99 B | 固件→上位机 | StateEstimation |

STATUS 轮索引固定为 `0=LF=M1, 1=LR=M2, 2=RF=M3, 3=RR=M4`。默认布局 mask `0x06`，即 M2 左轮、M3 右轮。禁用轮表示未参与布局，不是有效零速轮；`motor_speed_valid_mask` 必须是 enabled mask 的子集。方向由固件布局和运行参数统一修正，上位机不得再次翻转。

STATUS 仍严格为 92 B。encoder-valid、周期 delta、wheel generation 不存在于 v3；逐字段 offset、宽度、单位、范围和失效值见 [`upper-v3.schema.json`](upper-v3.schema.json)。
