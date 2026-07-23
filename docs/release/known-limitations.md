# v1.0.0-rc1 已知限制与未完成门禁

- v3 retired session 仅覆盖每链路本次 MCU 启动期最近 8 项；严格全历史防重放留待 v4。
- v3 不传输 encoder-valid、周期 delta、wheel generation 或完整操作结果。
- CLEAR_FAULT、LINE_CTRL 调用方只能得到 queued；业务结果仅在固件内部 Service/Communication 状态保留。
- 当前兼容候选仅为 Ros2_Slam v0.4.0；Raspberry Pi UART HIL 未通过前不得标记 compatible。
- Host UART、ESP WebSocket、双链路并发、故障注入、示波器证据和实车停车报告仍是硬门槛。
- 发布包、参数 identity CRC、产物哈希和最终 clean HELLO commit 尚未生成。

软件测试通过不等于可发布。以上门禁任一未完成时，不创建 annotated tag，不发布、不推送。
