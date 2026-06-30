# Lab 11: ADC 采样与电池监测

## 实验目标

1. **知识**：理解 STM32 ADC1 多通道 DMA 采样、电阻分压网络、分流电阻电流测量、EMA 数字滤波
2. **技能**：能通过 `status` 读取电池电压和电流 raw 值，验证分压公式换算精度，校准电流零点
3. **素养**：理解"模拟信号→数字量→物理量"的完整换算链中每步的误差来源

## 实验原理

### 1. ADC 采样架构

STM32F407 ADC1 由 TIM8 TRGO 以 1kHz 触发，每次扫描 5 个通道，并通过 DMA2 Stream0 循环写入缓冲区：

| 通道 | ADC 引脚 | 信号 | 物理意义 |
|------|---------|------|----------|
| CH10 | PC0 | M1_IPROPI | M1 电机电流 |
| CH11 | PC1 | M2_IPROPI | M2 电机电流 |
| CH12 | PC2 | M3_IPROPI | M3 电机电流 |
| CH13 | PC3 | M4_IPROPI | M4 电机电流 |
| CH14 | PC4 | VBAT_SENSE | 电池电压（经分压） |

> ADC DMA Rank 仍按 `CH10, CH11, CH12, CH13, CH14` 采样；BSP 层把 `PC1/CH11` 归为逻辑 M2，把 `PC2/CH12` 归为逻辑 M3。

参数（`chassis_config.h`）：

| 参数 | 默认值 |
|------|--------|
| ADC 参考电压 | 3.3V |
| ADC 分辨率 | 12-bit (0~4095) |
| ADC 硬件触发周期 | 1ms（TIM8 TRGO） |
| 监控与滤波更新周期 | 20ms（safetyTask） |

### 2. 电池电压换算——电阻分压

电池电压经 47kΩ + 10kΩ 分压后进入 ADC：

```
V_adc = V_bat × R_lower / (R_upper + R_lower)
      = V_bat × 10k / (47k + 10k)
      = V_bat × 10/57
      = V_bat / 5.7

V_bat = V_adc × 5.7
      = (ADC_raw / 4095 × 3.3) × 5.7
      = ADC_raw × (3.3 × 5.7 / 4095)
```

验证：12V 电池 → V_adc = 12/5.7 ≈ 2.105V → ADC_raw ≈ 2.105/3.3 × 4095 ≈ 2612。

### 3. 电机电流换算——分流电阻

DRV8874 IPROPI 输出电压与电机电流成正比，当前代码使用实测初始标定 `MOTOR_CURRENT_VOLTS_PER_AMP = 1.0V/A`：

```
V_propi = I_motor × MOTOR_CURRENT_VOLTS_PER_AMP = I × 1.0

I_motor = V_propi / 1.0

I_mA = abs(ADC_raw - ADC_zero_raw) / 4095 × 3.3 / 1.0 × 1000
```

上电静止阶段会对四路 IPROPI 分别采样 `ADC_MONITOR_CURRENT_ZERO_SAMPLES` 次，平均得到每路 raw 零点。实际运放偏移会被运行时零点抵消；若校准期间电机转动或负载电流不为零，零点会被错误写入运行时状态。

### 4. EMA 数字滤波

```
I_filtered(k) = 0.25 × I_raw(k) + 0.75 × I_filtered(k−1)
```

滤波效果：抑制 ADC 量化噪声（±3 LSB ≈ ±2.4mV ≈ ±24mA）。虽然 ADC 硬件以 1kHz 采样，EMA 在 safetyTask 中以 50Hz 更新；按约 4 次监控更新估算，阶跃响应延迟约为 80ms。

### 5. ADC 校准

`ADC_MONITOR_CALIBRATION_ENABLED = 1U`：启用运行时电流零点校准。F407 当前实现不依赖片上 ADC 自校准；电池电压绝对精度仍需要万用表或可调电源作为外部基准。

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘（架空）、12V 电池、USB-TTL 串口模块、万用表 |
| 软件 | 串口终端 |

## 实验步骤

### 步骤 1：电池电压验证

用万用表测量电池实际电压，与 `status` 显示值对比：

```bash
s
# ADC vbat=11700mV   ← 固件显示值
# 万用表测量电池端电压: ________ V
```

计算误差：

```
误差 = |固件值 − 万用表值| / 万用表值 × 100%
```

如果误差 > 2%，可能原因：
- 分压电阻实际值偏离标称值（47kΩ/10kΩ ±1% 可能导致 ±1.7% 误差）
- ADC 参考电压偏离 3.3V

修正方法：用万用表实测 R_upper 和 R_lower 的实际阻值，修改 `ADC_MONITOR_BATTERY_R_UPPER_OHM` / `_R_LOWER_OHM`。

### 步骤 2：分压公式反向验证

使用可调电源或不同电量的电池，记录多组 ADC raw 值与万用表电压：

| 万用表电压 (V) | ADC raw (vbat) | 固件显示 (mV) | 误差 (%) |
|---------------|----------------|---------------|---------|
| 12.50 |                |               |         |
| 11.50 |                |               |         |
| 10.50 |                |               |         |

```python
import numpy as np
import matplotlib.pyplot as plt

v_actual = np.array([12.50, 11.50, 10.50])
v_firmware = np.array([...])  # 填入固件显示值

plt.figure(figsize=(6, 6))
plt.scatter(v_actual, v_firmware, s=50)
plt.plot([10, 13], [10, 13], 'k--', alpha=0.3)
plt.xlabel('Multimeter (V)')
plt.ylabel('Firmware (mV) / 1000')
plt.title('ADC Battery Voltage Calibration')
plt.grid(alpha=0.3)
plt.savefig('lab11_vbat_calib.png', dpi=150)
```

### 步骤 3：电流零点偏移测量

电机静止（PWM=0），观察四路电流读数：

```bash
s
# ADC vbat=12400mV raw=2700 m1=5mA raw=1250 z=1249 ... cal=256/256 valid=1
```

**预期**：`cal=256/256 valid=1` 后，静止时电流应接近 0mA（±30mA）。若某路偏移明显（如 > 50mA），先确认校准期间电机没有动作，再检查该路 IPROPI 和 `z=` raw 零点。

记录零点偏移：

| 电机 | 显示电流 (mA) | raw ADC |
|------|-------------|---------|
| M1   |             |         |
| M2   |             |         |
| M3   |             |         |
| M4   |             |         |

### 步骤 4：电流精度验证

> 需要万用表（串联测量实际电流）或可调电子负载。

架空驱动单路电机，对比固件电流读数与万用表串联测量值：

```bash
m1 300 0           # M1 300‰ 正转
s                   # 记录固件电流读数
# 同时用万用表串联在 M1 电机回路中测量电流
m1 0 0
```

| 电机 | PWM (‰) | 固件电流 (mA) | 万用表电流 (mA) | 误差 (mA) |
|------|---------|-------------|----------------|----------|
| M1   | 300     |             |                |          |
| M2   | 300     |             |                |          |
| M3   | 300     |             |                |          |
| M4   | 300     |             |                |          |

### 步骤 5：电池放电曲线监测

让底盘持续运行并用 CSV 日志记录电池电压变化：

```bash
log 1 adc
motor 300 300       # 开环持续运行
# 保持 5-10 分钟（注意电池不要过放）
log 0
```

从日志中绘制电池电压-时间曲线，观察电压下降趋势。

## 数据分析

### 电流 EMA 滤波效果

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('lab11_current.csv')
t = df['t_ms'] / 1000.0

fig, ax = plt.subplots(figsize=(12, 5))
ax.plot(t, df['m1_ma'], 'b-', alpha=0.5, label='M1 filtered (EMA α=0.25)')
ax.set_ylabel('Current (mA)')
ax.set_xlabel('Time (s)')
ax.legend()
ax.grid(alpha=0.3)

# 计算噪声水平
noise_std = df['m1_ma'].std()
print(f'M1 current noise (std): {noise_std:.1f} mA')
plt.title(f'Current Signal After EMA Filter (noise σ={noise_std:.1f} mA)')
plt.savefig('lab11_current_ema.png', dpi=150)
```

### 电池放电曲线

```python
df = pd.read_csv('lab11_discharge.csv')
t = df['t_ms'] / 1000.0 / 60.0  # minutes
vbat = df['vbat_mv'] / 1000.0    # volts

plt.figure(figsize=(10, 5))
plt.plot(t, vbat, 'b-', alpha=0.7)
plt.axhline(10.5, color='orange', linestyle='--', label='Low battery warn (10.5V)')
plt.axhline(9.0, color='red', linestyle='--', label='Cutoff (9.0V)')
plt.xlabel('Time (min)')
plt.ylabel('Battery Voltage (V)')
plt.legend()
plt.grid(alpha=0.3)
plt.title('Battery Discharge Curve')
plt.savefig('lab11_discharge.png', dpi=150)
```

## 思考题

1. 电阻分压公式中 `ADC_MONITOR_BATTERY_DIVIDER = (47k+10k)/10k = 5.7`。如果 47kΩ 电阻实际为 46.5kΩ (±1% 误差)，电池电压读数会偏大还是偏小？偏多少？

2. 如果上电零点校准期间某一路实际带有 +10mV 对应的负载电流，这个错误零点对后续电流读数有什么影响？会造成过流保护误触发还是漏触发？

3. ADC 量化台阶 = 3.3V / 4095 ≈ 0.806mV。这意味着电流分辨率 ≈ 0.806mV / 1.0V/A ≈ 0.806mA/LSB。如果要提高电流测量分辨率到 0.1mA，有哪些方法？（提示：过采样平均、外部放大器、更高分辨率 ADC）

4. EMA 滤波的 α 值选择是一个 trade-off。α 越大响应越快但噪声更大。如果电流信号中混入了 50Hz 工频干扰，仅靠 EMA 滤波能否有效去除？如果不能，应该用什么滤波器？

5. 如果电池从 12.6V 放电到 10.5V（低压预警），电机的最大转速会下降多少？（假设 PWM 和负载不变，仅受供电电压影响）

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `vbat` 显示为 0 | 分压电阻开路或 ADC 通道配置错误 | 万用表检查 PC4 电压；确认 `.ioc` ADC 通道配置 |
| 电流始终为 0 | IPROPI 引脚未连接或 ADC 零点未完成 | 检查 PC0/PC1/PC2/PC3 接线和 `cal=` 进度 |
| 电流读数噪声大 | EMA α 过小或 50Hz 工频干扰 | 增大 `MOTOR_CURRENT_FILTER_ALPHA` |
| 电压读数与实际偏差大 | 分压电阻精度不足 | 用万用表实测阻值，更新配置宏 |
