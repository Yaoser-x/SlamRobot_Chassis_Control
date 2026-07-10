# 电流链与 ADC 采样相位 HIL 验收

当前 MCU 电流保护保持观察模式：`MOTOR_CURRENT_LIMIT_A=0.0f`、
`MOTOR_ADC_OVERCURRENT_FAULT_ENABLED=0U`。在完成以下实测前，不修改 ADC
触发相位，也不启用 ADC 软件过流锁停。

## 硬件核对

- 对照原理图和实板确认 DRV8874 的 `VREF`、`RIPROPI`、`IMODE`。
- 以现有 `MOTOR_STALL_CURRENT_A=2.4A` 为目标核算硬件 ITRIP。
- 快速逐周期限流由 DRV8874 完成；MCU 电流只用于 100 ms 级堵转锁停、
  负载估计和功率降额，不实现 20 ms 电流 PI。

## 示波器矩阵

同时采集 TIM1 PWM、DRV8874 IPROPI 和 ADC 外部触发点。每组至少记录稳定区
和换向瞬态：

| 电机 | 方向 | PWM 占空比 |
|---|---|---|
| M2 | 正转、反转 | 10%、50%、90% |
| M3 | 正转、反转 | 10%、50%、90% |

对每个工作点改变或扫过 ADC 采样相位，比较换算电流。若相位间偏差不超过
5%，保留当前触发并记录有效相位；超过 5% 时才允许修改 `.ioc`，将 TIM8
同步到 TIM1，并把完整 ADC 扫描窗口放入实测稳定导通区。

## 启用软件锁停的前置条件

- 已知电流标定点的换算误差不超过 10%。
- 静止零点 256 样本的 span 不超过 20 raw。
- 不同有效采样相位之间的电流偏差不超过 5%。
- BKIN/nFAULT 硬件链路和人工 `clearfault` 恢复路径均已通过。

全部满足并保留示波器截图/原始数据后，才将
`MOTOR_ADC_OVERCURRENT_FAULT_ENABLED` 设为 `1U`。比例软限流继续关闭。
