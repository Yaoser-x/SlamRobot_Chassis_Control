# Changelog

## 1.0.0-rc2 (software candidate, HIL_PENDING)

- 看门狗改为 Motor/Safety 完整周期双 completion、年龄门和 Evaluate/Feed/Commit 两阶段语义。
- Safety 发布能力矩阵和 40 ms permit 租约；能力 mask 在命令选择前与 mode mask 求交，IMU 故障降级 heading，Motor 在 IWDG 前处理过期许可。
- App 统一事实 DTO、模式、维护和安全事件编排，删除 Safety/Motion 跨 Service 拉取与写入入口。
- 增加五模式与 PS2 去抖接管；MANUAL 回中只恢复模式，LINE 不自动重新使能，AUTO/LINE 必须重新 rearm 才能运动。
- wire duplicate 校验留在 Communication，Command refresh 绑定 slot/command/mode/revoke generation。
- 修复 encoder_count 原始累计语义和 Host delivery latch，保持 Upper Protocol v3 字节布局不变。
- 控制 timing 按 nominal period 整数派生；四轮 effective `pid_kd=0`，HELLO 报告 effective parameter CRC。
- 增加 gyro-rate OADEV、ARW/BI/RRW、缺口/抖动检查和确定性噪声 fixture；yaw 结果独立命名。

## 1.0.0-rc1

- 历史候选标签 `v1.0.0-rc1` 指向 `bc472cc`。
