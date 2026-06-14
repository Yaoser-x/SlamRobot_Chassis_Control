# Lab 04: 电流监测与限流

## 实验目标

1. **知识**：理解分流电阻测流原理、ADC 采样与换算、EMA 低通滤波、过流去抖保护机制
2. **技能**：能通过 `status`/`log 1 adc` 观察实时电流，分析空载/带载/堵转三种工况下的电流特征
3. **素养**：理解电机电流与负载的对应关系，建立"电流即力矩"的电机控制直觉

## 实验原理

### 1. 分流电阻测流

DRV8874 的 IPROPI 引脚输出与电机电流成比例的电压，经 ADC 采样后换算：

```
I_motor = (V_adc − V_zero) / (R_shunt × Gain)
         = (V_adc − MOTOR_CURRENT_ZERO_V) / (0.1Ω × 1.0)
         = (V_adc − 0.0) / 0.1 = V_adc × 10
```

参数（`chassis_config.h`）：

| 参数 | 符号 | 默认值 |
|------|------|--------|
| 分流电阻 | `MOTOR_CURRENT_SHUNT_OHM` | 0.1 Ω |
| 电流零点 | `MOTOR_CURRENT_ZERO_V` | 0.0 V |
| 电压-电流比 | `MOTOR_CURRENT_VOLTS_PER_AMP` | 0.1 V/A |
| 电流限幅 | `MOTOR_CURRENT_LIMIT_A` | 0.8 A |
| 过流去抖次数 | `MOTOR_OVERCURRENT_DEBOUNCE_COUNT` | 5 |
| 额定电流 | `MOTOR_RATED_CURRENT_A` | 0.65 A |
| 堵转电流 | `MOTOR_STALL_CURRENT_A` | 2.4 A |
| EMA 滤波系数 | `MOTOR_CURRENT_FILTER_ALPHA` | 0.25 |

### 2. EMA 低通滤波

为抑制 ADC 采样噪声，对电流做指数移动平均（EMA）：

```
I_filtered(k) = α × I_raw(k) + (1−α) × I_filtered(k−1)
α = 0.25（新样本权重 25%，旧值权重 75%）
```

α 越大响应越快但噪声越大，α 越小越平滑但滞后越大。

### 3. 过流保护去抖

为避免瞬时噪声触发误保护，采用连续超阈值去抖：

```
if (I > MOTOR_CURRENT_LIMIT_A) 连续 MOTOR_OVERCURRENT_DEBOUNCE_COUNT (5) 次:
    → 设置过流标志 + 触发 fault-stop
```

`clearfault` 可清除锁存故障，但前提是电流已回落至阈值以下。

### 4. 电流-负载-速度关系

```
空载: I ≈ 0.05~0.15 A  (仅克服内部摩擦)
带载: I ≈ 0.2~0.5 A    (正常行驶)
爬坡/加速: I ≈ 0.5~0.8 A (接近限流)
堵转: I → MOTOR_STALL_CURRENT_A (2.4 A, 触发保护)
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
log 1 adc                         # CSV 日志仅 adc 字段
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
ADC vbat=11700mV m1=120mA raw=1452 m2=110mA raw=1334 m3=130mA raw=1580 m4=105mA raw=1280 valid=1
         │           │       │        │       │         │       │         │       │         │
         │           │       │        │       │         │       │         │       │         └─ ADC 校准有效
         │           │       │        │       │         │       │         │       └─ M4 电流
         │           │       │        │       │         │       │         └─ M4 raw ADC
         │           │       │        │       │         │       └─ M3 电流
         │           │       │        │       │         └─ M3 raw ADC
         │           │       │        │       └─ M2 电流
         │           │       │        └─ M2 raw ADC
         │           │       └─ M1 电流
         │           └─ M1 raw ADC 值
         └─ 电池电压 (mV)
```

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
log 1 motor adc                  # motor + adc 双字段
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

### 步骤 3：限流触发实验

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
1. M1 电流迅速上升，超过 `MOTOR_CURRENT_LIMIT_A = 0.8A`
2. 连续 5 次超阈值后触发 `SYSTEM_ERROR_M1_OVERCURRENT`
3. `status` 中 `errors` 字段出现对应位（0x00000002）
4. `fault=1` — fault-stop 触发，全部电机停止
5. 输入 `clearfault` 尝试清除

```bash
clearfault                      # 尝试清除过流锁存
s                               # 确认 errors 是否清零
```

**注意**：若堵转后电流仍未回落至 0.8A 以下，`clearfault` 会拒绝清除。

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
ax2.axhline(800, color='red', linestyle='--', label='limit 800mA')  # mA
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

4. `MOTOR_CURRENT_LIMIT_A = 0.8A` 比 `MOTOR_RATED_CURRENT_A = 0.65A` 高约 23%。这个余量是否合理？如果要让底盘具备更强的爬坡能力，提高限流值有什么风险和收益？

5. `MOTOR_CURRENT_ZERO_V = 0.0f` 假设零电流时 ADC 电压为 0V。实际运放输出可能有偏移（如 10mV），这会导致什么误差？如何校准零点？

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| 四路电流读数均为 0 | ADC 校准未完成 | `status` 检查 `valid=1`，等待片上 ADC 校准 |
| 某路电流持续为负 | 电流零点偏移或方向反接 | 检查 DRV8874 IPROPI 接线；调整 `MOTOR_CURRENT_ZERO_V` |
| 轻微负载即触发过流 | 限流阈值过低 | 确认 `MOTOR_CURRENT_LIMIT_A` 与实际电机规格匹配 |
| `clearfault` 无效 | 电流仍高于阈值或 nFAULT 仍低 | `status` 确认电流已回落；检查 DRV nFAULT 引脚 |
| 电流读数噪声大 | α 过小或 ADC 参考电压不稳 | 增大 `MOTOR_CURRENT_FILTER_ALPHA`；检查 VREF 去耦电容 |
