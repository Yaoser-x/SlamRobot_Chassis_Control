# Lab 10：控制模式、来源仲裁与安全撤销

## 实验目标

1. 理解 RC2 的两级决策：App 先选择产品模式，Command Management 再在该模式允许的来源中仲裁。
2. 验证命令租约、PS2 接管、rearm 和 Safety motion gate 的 fail-closed 行为。
3. 能从 `status`、Upper v3 ACK/DIAGNOSTIC 和 Host fixture 记录中还原一次来源切换。

## 当前控制模型

RC2 不再让五个来源无条件同时竞争。`App/composition/control_mode_coordinator.c` 持有模式，
`Service/command_management/command_management_service.c` 持有来源槽和命令生命周期：

| 模式 | 允许来源 | 进入方式 |
| --- | --- | --- |
| `DISABLED` | 无 | 安全降级或 PS2 接管期间断联 |
| `AUTO` | Host、ESP12F | 上电恢复模式；远程控制默认模式 |
| `MANUAL` | PS2 | 摇杆接管确认或 D-pad/定角宏边沿 |
| `LINE` | LINE | PS2 三角键或远程 `LINE_CTRL` 编排 |
| `MAINTENANCE` | DEBUG 诊断 | App 维护编排器持锁期间 |

同一模式内仍按固定数组 `{HOST, PS2, ESP12F, LINE, DEBUG}` 选择第一个有效槽。
实际会出现并行竞争的是 `AUTO` 中的 Host 与 ESP12F，Host 优先。

### 命令租约

| 来源 | 单次租约 |
| --- | ---: |
| Host | 200 ms |
| PS2 | 500 ms |
| ESP12F | 500 ms |
| LINE | 50 ms |
| DEBUG | 2000 ms |

Host/ESP12F duplicate 不能直接延长任意旧命令。只有 session、sequence、payload、slot generation、
mode generation 和 motion-revoke generation 全部匹配时，`CommandManagement_RefreshAccepted()`
才刷新已接受命令；远程槽自首次接受起最多存活 2000 ms。

### rearm 与撤销

以下事件都会清槽或提升 motion-revoke generation：

- Safety motion gate 关闭；
- ESTOP、fault-stop 或维护锁首次生效；
- 产品模式切换；
- capability mask 收紧。

恢复安全条件只会重新开放能力，不会恢复旧运动。每个来源必须先完成自己的 release/neutral
条件，再提交新的 active intent。NaN/Inf、非法来源、旧 generation 和当前模式不允许的来源均被拒绝；
危险输入不会靠静默钳位继续运动。

## 实验设备与边界

| 类别 | 项目 |
| --- | --- |
| 硬件 | F407_V2.0 底盘、12 V 电池、USART1 串口、PS2；Host/ESP12F 至少一个 |
| 软件 | Upper Protocol v3 发送端或匹配 fixture、串口终端 |

首次实验应架空驱动轮，限低速并保留物理断电手段。该实验是 HIL 项，不由 Host 测试替代。

## 实验步骤

### 1. 验证 AUTO 远程来源

1. 上电后用 `status` 确认无 ESTOP/fault-stop，Safety permit 有效。
2. Host 先发送 release/neutral，再发送低速 `SET_VELOCITY`，确认 active source 为 Host。
3. 保持 Host 有效时让 ESP12F 发送同类命令，确认仍选择 Host。
4. 停止 Host 刷新并等待超过 200 ms；若 ESP12F 槽仍有效，应退让到 ESP12F，否则停止。
5. 停止 ESP12F 刷新并等待超过 500 ms，确认 active source 回到 NONE 且 PWM 为零。

记录 ACK outcome、source、slot generation、mode generation 和 motion-revoke generation。

### 2. 验证 PS2 接管与中立恢复

1. 在 AUTO 低速运动时，将 PS2 摇杆推过 `0.15`，保持连续 3 个有效样本。
2. 确认模式进入 MANUAL、远程槽被撤销，active source 变为 PS2。
3. 摇杆回到 `0.10` 以下，并保持无 D-pad/宏输入 2000 ms。
4. 确认模式恢复 AUTO，但旧 Host/ESP 命令没有自动运动。
5. Host 重新执行 release/neutral，再提交新 active 命令，确认运动才恢复。

如果 MANUAL 期间 PS2 断联，预期模式进入 DISABLED，而不是回放接管前命令。

### 3. 验证 LINE 模式边界

1. 通过 PS2 三角键或远程 `LINE_CTRL` 请求 LINE 模式并显式启用巡线。
2. 放置有效黑线，确认 LINE intent 生效。
3. 移走黑线或停止 UART4 有效帧，确认 debounce/50 ms 新鲜度门使 LINE 退让并停车。
4. 用 PS2 触发 MANUAL 接管；中立 2000 ms 后即使恢复 LINE 模式，也必须再次显式 enable，
   不得在同一恢复事件中自动运动。

USART1 的 `line on` 只控制 Line Following enable，不绕过 App mode mask；教学实验应通过上述产品入口切换模式。

### 4. 验证 Safety gate

```text
远程低速运动
  → USART1: estop 1
  → active source = NONE，PWM = 0
  → USART1: estop 0
  → 旧远程命令仍不可恢复
  → 远程重新 release/neutral + active 后才可运动
```

若 `nFAULT` 仍为低或硬件 Break 仍锁存，不应通过重复 `clearfault` 强行继续实验。

## 记录表

| 时间 | 模式 | capability mask | active source | revoke generation | 事件 | 结果 |
| --- | --- | --- | --- | --- | --- | --- |
| | AUTO | | HOST | | Host active | |
| | AUTO | | ESP12F/NONE | | Host timeout | |
| | MANUAL | | PS2 | | takeover | |
| | AUTO | | NONE | | neutral restore | |
| | AUTO | | NONE | | ESTOP clear | |

## 判定标准

- 模式之外的来源不能形成运动输出。
- Host 优先于 ESP12F；各来源在自身租约过期后退让。
- PS2 接管和 Safety 撤销均清除旧来源；恢复后必须 rearm。
- 任意安全门关闭时，同周期命令选择归零，Motor 周期不接受过期 permit。
- 观察到的 source、mode、ACK outcome 与上述状态转换一致。

## 思考题

1. 为什么 RC2 同时保留 mode mask、capability mask 和来源优先级，而不是只保留一个优先级数组？
2. 为什么重复帧刷新还要绑定 slot/mode/revoke generation？
3. PS2 回中后先恢复模式、但不恢复运动，避免了哪类现场风险？
4. 若新增蓝牙来源，应归入现有模式还是新增模式？它的 release 条件和租约应由谁持有？

## 相关代码

- `App/composition/control_mode_coordinator.c`
- `App/composition/chassis_runtime_coordinator.c`
- `Service/command_management/command_management_service.c`
- `Service/communication/internal/remote_command_dispatcher.c`
- `Service/teleoperation/teleoperation_service.c`
- `Service/line_following/line_following_service.c`
