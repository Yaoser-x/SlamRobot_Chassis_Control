# Upper Protocol v3 Session 与 Sequence

Host USART3 与 ESP USART2 各自拥有 active session、sequence、ACK 和最近 8 项 retired session 表，状态互不覆盖。retired 防重放只保证本次 MCU 启动期每链路最近 8 个 session；全历史防重放和单调 session ID 属于 v4。

新 session 只能由 `enable=0` 的 SET_VELOCITY 建立。陌生 session 的 enable 必须拒绝。sequence 的前进定义为 `0 < uint32(new-old) < 2^31`，因此支持 u32 回绕；倒退和半圈歧义拒绝。

相同 sequence 只有在目标、enable 和 mode 完全一致时才是 duplicate。Communication 完成 session、sequence
和 payload 校验后，enable duplicate 只能凭 CommandManagement 签发的 slot token 刷新仍存在、未过期且
未超过原始最大寿命的租约；CommandManagement 不解释 wire session。

安全 gate 或模式撤销会清空来源并推进 revoke/mode generation。故障期间及 MANUAL 中的 disable 不建立资格；
恢复后必须先收到大于最新 received sequence 的 disable，再收到更大的 enable。duplicate disable 不能跨越
安全事件或 MANUAL 接管完成 rearm。
