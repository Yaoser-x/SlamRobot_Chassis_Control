# Upper Protocol v3 Session 与 Sequence

Host USART3 与 ESP USART2 各自拥有 active session、sequence、ACK 和最近 8 项 retired session 表，状态互不覆盖。retired 防重放只保证本次 MCU 启动期每链路最近 8 个 session；全历史防重放和单调 session ID 属于 v4。

新 session 只能由 `enable=0` 的 SET_VELOCITY 建立。陌生 session 的 enable 必须拒绝。sequence 的前进定义为 `0 < uint32(new-old) < 2^31`，因此支持 u32 回绕；倒退和半圈歧义拒绝。

相同 sequence 只有在目标、enable 和 mode 完全一致时才是 duplicate。duplicate 不改变目标、generation 或既有应用结果；enable duplicate 只能刷新仍存在且未超时的租约。安全 gate 关闭会清空五来源并令 Host/ESP 重新需要 rearm；故障期间的 disable 不建立资格，恢复后必须收到更新 sequence 的 disable。duplicate disable 不能跨越安全事件 rearm。
