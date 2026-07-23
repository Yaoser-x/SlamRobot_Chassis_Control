# Upper Protocol v3 冻结声明

v3 的机器可读事实源为 [`../protocol/upper-v3.schema.json`](../protocol/upper-v3.schema.json)，固定长度为 SET_VELOCITY 23 B、ESTOP 2 B、LINE_CTRL 2 B、CLEAR_FAULT 1 B、GET_INFO 1 B、HELLO 34 B、STATUS 92 B、DIAGNOSTIC 28 B、IMU_STATUS 99 B。

RECEIVED 不等于 APPLIED。CLEAR_FAULT 和 LINE_CTRL 在 v3 只能报告 queued，不能宣称业务已应用。duplicate 只刷新仍存在的租约，不改变目标、generation 或既有结果。每链路仅保存本次启动期最近 8 个 retired session。

v3 不再新增字段或复用未定义 bit。完整业务结果、单调 session、全历史防重放、encoder-valid、周期 delta 和 wheel generation 均登记为 Protocol v4 输入。
