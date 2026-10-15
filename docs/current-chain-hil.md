# 电流链与 ADC 采样相位 HIL 验收

当前 MCU 电流保护保持观察模式：`MOTOR_CURRENT_LIMIT_A=0.0f`、
`MOTOR_ADC_OVERCURRENT_FAULT_ENABLED=0U`。在完成以下实测前，不修改 ADC
触发相位，也不启用 ADC 软件过流锁停。

## 硬件核对

### 采样有效性与电流幅值

ADC 控制有效性检查缺样、DMA 错误、窗口样本数、零点稳定性，以及已启用电机
通道的满量程饱和。任一有效通道在窗口内出现 `raw >= 4095`，即报告
`invalid=0x00000100`、该通道 `ADCQ q` 的 `0x08` 位，并撤销运动权限；后续
正常窗口恢复有效性后，PS2 仍须回中解锁，旧命令不会自动恢复。

`POWER_ADC_DRIVER_CURRENT_SPIKE_MAX_RAW=512` 保留为幅值观察阈值。相对零点
超过该值只设置通道 `ADCQ q` 的 `0x04` 位，不再设置旧 `invalid=0x80`，也不再
清除 `cvalid` 或 `cmask`。它检查的是电流幅值，而不是采样跳变；不能用它代替
以安培为单位的负载、软限流和过流判定。高幅值的均值、RMS、峰值仍完整发布。

已配置的软件限流/过流开关与硬件 BKIN/nFAULT 停车路径保持原样。此修改不代表
电流比例、采样相位及软件过流锁停已完成实机验收。

### 实板参数

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
# P1 参数化采集

使用 `log json` 或 CSV 留存空载、瞬态、峰值和堵转前数据，再运行：

```bash
python scripts/analyze_roadmap_data.py current current.csv --json current.json --markdown current.md
```

分析结果用于提出 observe/soft/fault 与去抖参数，但不得在没有电源、电机、样本量和原始 CSV 的情况下关闭验收。
