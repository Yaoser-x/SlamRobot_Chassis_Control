
# Lab 04: 电流监测与限流

## 实验目标

1. **知识**：理解 DRV8874 IPROPI 电流镜采样、ADC 采样窗口统计、EMA 低通滤波、过流去抖保护机制
2. **技能**：能通过 `status`/`log 1 adc adcraw` 观察稳定电流和窗口统计，分析空载/带载/堵转三种工况下的电流特征
3. **素养**：理解电机电流与负载的对应关系，建立"电流即力矩"的电机控制直觉

## 实验原理

### 1. IPROPI 电流镜测流

DRV8874 的 IPROPI 引脚输出与电机电流成比例的电压，经 ADC 采样后换算：

```
I_motor = max(V_adc - V_zero, 0) / MOTOR_CURRENT_VOLTS_PER_AMP
         = max(ADC_raw - ADC_zero_raw, 0) / 4095 × 3.3 / 1.0
```

参数（`control_config.h / bsp_config.h`）：

| 参数 | 符号 | 默认值 |
|------|------|--------|
| 上电零点采样 | `ADC_MONITOR_CURRENT_ZERO_SAMPLES` | 256 |
| 电压-电流比 | `MOTOR_CURRENT_VOLTS_PER_AMP` | 1.0 V/A |
| 每路电压-电流比 | `MOTOR_CURRENT_VOLTS_PER_AMP_M1..M4` | 继承全局 |
| 实时电流节流 | `MOTOR_CURRENT_LIMIT_A` | 0.0 A（关闭） |
| 观察模式 | `MOTOR_CURRENT_GUARD_OBSERVE_ONLY` | 1U（只观察） |
| 软限流开关 | `MOTOR_CURRENT_SOFT_LIMIT_ENABLED` | 0U（关闭） |
| ADC 过流锁停 | `MOTOR_ADC_OVERCURRENT_FAULT_ENABLED` | 0U（关闭） |
| 过流去抖次数 | `MOTOR_OVERCURRENT_DEBOUNCE_COUNT` | 5 |
| 额定电流 | `MOTOR_RATED_CURRENT_A` | 0.65 A |
| 堵转电流 | `MOTOR_STALL_CURRENT_A` | 2.4 A |
| EMA 滤波系数 | `MOTOR_CURRENT_FILTER_ALPHA` | 0.25 |

### 2. 窗口统计与 EMA 低通滤波

ADC1 由 TIM8 TRGO 触发约 2kHz 采样；`AdcMonitor_Update()` 每 20ms 取走一批样本并计算：

```
signed_delta = raw - zero
delta        = max(signed_delta, 0)
I_signed     = mean(signed_delta)
I_mean       = mean(delta)
I_rms        = sqrt(mean(delta^2))
I_peak       = max(delta)
I_noise      = stddev(signed_delta)
I_trimmed = 去掉窗口最大/最小后的均值
```

`log 1 adc` 仍输出慢速稳定电流；`log 1 adcraw` 输出窗口 `mean/rms/peak/n`。`status`/`adccal show` 额外输出 `signed_mean/noise/zero_span/quality_flags`。与可调电源电流对比时优先看 `mean` 或 `trimmed`，不要直接拿 `peak` 对比电源平均电流；`signed_mean` 用来观察零点正负漂移，`noise` 用来判断窗口是否可信。

为抑制 ADC 采样噪声，慢速稳定电流继续对 `I_trimmed` 做指数移动平均（EMA）：

```
I_filtered(k) = α × I_raw(k) + (1−α) × I_filtered(k−1)
α = 0.25（新样本权重 25%，旧值权重 75%）
```

α 越大响应越快但噪声越大，α 越小越平滑但滞后越大。

### 3. 过流保护去抖

为避免瞬时噪声触发误保护，采用连续超阈值去抖：

```
if (current_control_valid && MOTOR_ADC_OVERCURRENT_FAULT_ENABLED && I > MOTOR_STALL_CURRENT_A) 连续 MOTOR_OVERCURRENT_DEBOUNCE_COUNT (5) 次:
    → 设置过流标志 + 触发 fault-stop
```

默认配置下系统只累计 `observe/would` 计数，不改变 PWM，也不触发 ADC 软件 fault-stop。`clearfault` 可清除锁存故障，但前提是电流已回落至阈值以下。

### 4. 电流-负载-速度关系

```
空载: I ≈ 0.05~0.15 A  (仅克服内部摩擦)
带载: I ≈ 0.2~0.5 A    (正常行驶)
爬坡/加速: I 随负载上升，默认只记录 dry-run 观察计数
堵转: I → MOTOR_STALL_CURRENT_A (2.4 A, 当前仅记录诊断；默认不触发 ADC 软件锁停)
```

> **核心直觉**：电机电流 ≈ 输出力矩。电流越大表示电机越"吃力"。

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘（架空→落地）、12V 电池、USB-TTL 串口模块 |
| 软件 | 串口终端 |

## 实验步骤

### 步骤 1：空载电流基线

底盘架空（车轮无负载）：

```bash
status                           # 确认 cal/valid/cvalid/ADCQ
adccal show                      # 查看零点跨度、噪声和 dry-run 计数
adccal zero                      # 仅在所有启用电机 effective_pwm=0 时执行
log 1 adc adcraw                  # 稳定电流 + 窗口统计
motor 200 200                     # 低速空载
# 保持 5 秒
motor 400 400                     # 中速
# 保持 5 秒
motor 200 200
# 保持 5 秒
motor 0 0
log 0
```

`status` 实时观察 ADC 行：

```
ADC vbat=11700mV raw=2545 m1=120mA raw=1452 z=1437 m2=110mA raw=1334 z=1320 m3=130mA raw=1580 z=1564 m4=105mA raw=1280 z=1267 cal=256/256 valid=1
```

字段含义：`vbat` 为滤波后的电池电压，紧随其后的 `raw` 为 VBAT ADC 原始值；每路电机的 `raw` 是最近窗口最后一帧 IPROPI 原始值，`z` 是上电静止校准得到的零点；`cal=256/256 valid=1` 表示电流零点完成且当前电流读数可显示。`ADCCAL cvalid=1` 表示可用于保护/控制；`ADCQ signed/noise/span/q` 分别表示带符号均值、窗口噪声、零点 raw 跨度和质量标志。`ADCWIN` 行中的 `mean/rms/pk/n` 分别表示最近窗口的均值、RMS、峰值和样本数。

**记录四路空载电流（mA）**：

| 电机 | 200‰ 空载 | 400‰ 空载 | 600‰ 空载 |
|------|----------|----------|----------|
| M1   |          |          |          |
| M2   |          |          |          |
| M3   |          |          |          |
| M4   |          |          |          |

**分析**：
- 空载电流是否 < 200mA？
- 四路电流是否基本一致（差异 < 50mA）？
- 电流随占空比增大是因为转速升高 → 轴承摩擦增大

### 步骤 2：手扶加载测试

底盘架空，用手轻扶车轮增加阻力：

```bash
log 1 motor adc adcraw           # motor + 稳定电流 + 窗口统计
vel 150                          # 低速闭环
# 依次对 M1/M2/M3/M4 车轮手动施加阻力，每个持续 2~3 秒
# 观察对应电流上升
vel 0
log 0
```

**观察重点**：
- 加载瞬间电流上升幅度
- 速度在加载时是否下降（PID 增大 PWM 输出补偿）
- 撤除负载后电流迅速回落的速度（EMA 滤波的滞后特性）

### 步骤 3：DRV 硬件保护观察

> **警告**：此步骤需要人为堵转电机，持续时间 < 2 秒。长时间堵转会造成电机和驱动板过热。

```bash
log 1 motor adc errors          # 加入 errors 字段
vel 200                          # 低速前进
# 手动用力握住 M1 车轮使其停转（堵转）
# 持续约 3 秒后松手
vel 0
log 0
```

**预期**：
1. 电机负载升高时，观察 ADC 电流峰值、均值、噪声和电源平均电流的差异
2. 若 DRV8874 nFAULT 拉低，系统触发 fault-stop
3. `status` 中 `errors` 字段出现对应位（0x00000002）
4. `fault=1` — fault-stop 触发，全部电机停止
5. 输入 `clearfault` 尝试清除

```bash
clearfault                      # 尝试清除过流锁存
s                               # 确认 errors 是否清零
```

**注意**：当前默认关闭 ADC 软件过流锁停；`clearfault` 仍会受 DRV nFAULT 状态约束。

### 步骤 3b：软件限流 dry-run 验收

保持默认配置时，短时带载或短时堵转只应让 `ADCCAL observe/would` 增加，不应改变 `effective_pwm`，也不应触发 ADC 软件 fault-stop。只有在代码中显式关闭 `MOTOR_CURRENT_GUARD_OBSERVE_ONLY` 并打开 `MOTOR_CURRENT_SOFT_LIMIT_ENABLED` 后，才允许做低阈值短测，确认 PWM 被限幅且状态位变化。

### 步骤 4：带载跑行测试

底盘落地，在平坦地面上低速行驶：

```bash
log 1 motor adc
vel 150                          # 0.15 m/s 前进，地面行驶
# 行驶 10 秒
# 尝试轻微加速
vel 250
# 行驶 10 秒
vel 0
log 0
```

对比架空和落地两种工况下的电流差异：

| 工况 | 速度 (mm/s) | M1 电流 (mA) | M2 电流 (mA) | M3 电流 (mA) | M4 电流 (mA) |
|------|------------|-------------|-------------|-------------|-------------|
| 架空 | 150        |             |             |             |             |
| 落地 | 150        |             |             |             |             |
| 差值 | —          |             |             |             |             |

## 数据分析

### 电流-负载时序分析

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('lab04_load.csv')
t = df['t_ms'] / 1000.0

fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# 速度
axes[0].plot(t, df['m1_mms']/1000, 'b-', label='M1 speed', alpha=0.7)
axes[0].set_ylabel('Speed (m/s)')
axes[0].legend(loc='upper left')
axes[0].grid(alpha=0.3)

# 电流
ax2 = axes[1]
for i, col in enumerate(['m1_ma', 'm2_ma', 'm3_ma', 'm4_ma']):
    ax2.plot(t, df[col], label=f'M{i+1} current', alpha=0.7)
ax2.axhline(2400, color='red', linestyle='--', label='stall 2400mA')  # mA
ax2.set_ylabel('Current (mA)')
ax2.set_xlabel('Time (s)')
ax2.legend(loc='upper left')
ax2.grid(alpha=0.3)

plt.suptitle('Lab 04: Current Response Under Load')
plt.tight_layout()
plt.savefig('lab04_current_load.png', dpi=150)
```

### EMA 滤波效果对比

若有 raw ADC 数据和滤波后电流数据，对比滤波前后的噪声水平：

```python
# raw_current 换算后 vs 滤波电流
noise_raw = df['m1_current_raw'].std()
noise_filtered = df['m1_ma'].std()
print(f'Raw noise (std): {noise_raw:.1f} mA')
print(f'Filtered noise (std): {noise_filtered:.1f} mA')
print(f'Noise reduction: {(1 - noise_filtered/noise_raw)*100:.1f}%')
```

## 思考题

1. 为什么堵转电流远大于运行电流？从直流电机等效电路（电阻 + 反电动势）的角度解释。

2. EMA 滤波系数 α=0.25 意味着新样本的权重为 25%。如果改为 α=0.05，电流响应会变得怎样？这对过流保护的响应延迟有什么影响？

3. 过流去抖需要连续 5 次超阈值才触发保护（safetyTask 20ms 周期，即 100ms 去抖窗口）。为什么需要这个去抖机制？如果去掉（1 次即触发），会有什么风险？

4. 当前 `MOTOR_CURRENT_GUARD_OBSERVE_ONLY = 1U`、`MOTOR_CURRENT_SOFT_LIMIT_ENABLED = 0U` 且 `MOTOR_ADC_OVERCURRENT_FAULT_ENABLED = 0U`。若重新启用实时节流或 ADC 过流锁停，需要怎样区分 PWM 相电流峰值和电源平均电流？

5. 如果上电零点校准期间电机没有完全静止，错误的 `ADC_zero_raw` 会导致什么误差？如何用 `adccal zero` 与 `ADCQ span/noise` 验证零点？

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 四路电流读数均为 0 | ADC 零点未完成 | `status` 检查 `cal=256/256 valid=1`，等待上电零点采样完成 |
| 某路电流持续异常 | 电流零点错误或 IPROPI 接线异常 | 停机后执行 `adccal zero`；检查该路 `z=` raw 零点、`ADCQ span/noise` 和 DRV8874 IPROPI 接线 |
| 轻微负载即触发过流 | 限流阈值过低 | 确认 `MOTOR_CURRENT_LIMIT_A` 与实际电机规格匹配 |
| `clearfault` 无效 | 电流仍高于阈值或 nFAULT 仍低 | `status` 确认电流已回落；检查 DRV nFAULT 引脚 |
| 电流读数噪声大 | α 过小、零点不稳或 ADC 参考电压不稳 | 查看 `ADCQ noise/span/q`；增大 `MOTOR_CURRENT_FILTER_ALPHA`；检查 VREF 去耦电容 |
