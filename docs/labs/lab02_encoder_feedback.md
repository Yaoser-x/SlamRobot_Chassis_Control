
# Lab 02: 编码器与速度反馈

## 实验目标

1. **知识**：理解增量式正交编码器原理、四倍频计数、M 法测速公式与误差来源
2. **技能**：能通过 `status` 读取编码器原始脉冲和换算速度，验证方向符号与运动一致性
3. **素养**：建立"传感器数据验证闭环"的工程习惯——每次上电先确认编码器数据合理性

## 实验原理

### 1. 正交编码器与四倍频

增量式编码器输出 A/B 两相正交方波，STM32 的 TIM2/3/4/5 编码器模式对两相的上升沿和下降沿均计数，实现 **4 倍频**：

```
          ┌───┐   ┌───┐   ┌───
  A  ─────┘   └───┘   └───┘
        ┌───┐   ┌───┐   ┌───
  B  ───┘   └───┘   └───┘
        ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑
        每周期 4 个计数（A 和 B 的上升/下降沿各一次）
```

V2.0 逻辑电机与编码器定时器映射如下：

| 电机 | 定时器 | 引脚 |
|------|--------|------|
| M1 | TIM2 | PA15 / PB3 |
| M2 | TIM4 | PD12 / PD13 |
| M3 | TIM3 | PB4 / PB5 |
| M4 | TIM5 | PA0 / PA1 |

> CubeMX 生成文件中的 M2/M3 GPIO label 保留旧命名；调试时以 `status` 的逻辑 M1~M4 输出和上表为准。

设编码器基础线数 PPR (Pulses Per Revolution)，经四倍频和减速比后：

```
编码器每转计数 = PPR × 4
电机轴每转计数 = PPR × 4 × 减速比
```

### 2. M 法测速

在周期 Δt 内读取计数增量 ΔN，换算为速度：

```
转速 (RPS) = ΔN / (PPR × 4 × GearRatio × Δt)
线速度 (m/s) = 转速 × 2π × R_wheel
```

本平台参数（`control_config.h / bsp_config.h`）：

| 参数 | 符号 | 默认值 |
|------|------|--------|
| 编码器基础 PPR | `CHASSIS_ENCODER_BASE_PPR` | 11 |
| 四倍频倍率 | `CHASSIS_ENCODER_QUADRATURE_MULT` | 4 |
| 减速比 | `CHASSIS_MOTOR_GEAR_RATIO` | 56 |
| 轮半径 | `CHASSIS_WHEEL_RADIUS_M` | 0.035 m |
| 轮距 | `CHASSIS_TRACK_WIDTH_M` | 0.176 m |（有效轮距，机械测量 181.5mm）|
| 测速周期 | `CHASSIS_ENCODER_PERIOD_MS` | 10 ms |
| 最大 dt | `CHASSIS_MAX_ENCODER_DT_MS` | 100 ms |

计算示例：编码器 Δt = 10ms 内 ΔN = 100，则：

- 电机转速 = 100 / (11 × 4 × 56 × 0.01) = 0.406 rps
- 轮速 = 0.406 × 2π × 0.035 = 0.089 m/s = 89 mm/s

### 3. 方向符号

编码器方向由 `CHASSIS_Mx_ENCODER_DIR` 宏控制（±1）。前进时 `status` 中 `speed_mm/s` 应为正值。若为负值，需翻转该路的方向符号。

`status` 输出解读（ENC 行）：

```
ENC m1=12345 d=100 89mm/s v=1 m2=...
     │      │    │       │
     │      │    │       └─ speed_valid: 1=有效 0=超时失效
     │      │    └─ 换算速度 (mm/s)
     │      └─ 本周期增量 (counts)
     └─ 累计脉冲数 (counts)
```

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘（架空）、12V 电池、USB-TTL 串口模块 |
| 软件 | 串口终端（115200 8N1） |

## 实验步骤

### 步骤 1：静止状态观测

底盘架空，上电后直接 `status`（不驱动任何电机）：

**预期**：
- `count` 静止不增长（或极缓慢漂移 < 5 counts/s）
- `delta` = 0
- `speed_mm/s` = 0（或接近 0，允许 ±2 mm/s 噪声）
- `speed_valid v=1`（编码器定时器正在计数，有效）

```bash
s               # 快速状态快照
s               # 第二次，间隔约 2 秒，观察 count 是否变化
```

**记录**：

| 电机 | count (t=0) | count (t=2s) | delta | speed (mm/s) | valid |
|------|-------------|--------------|-------|-------------|-------|
| M1   |             |              |       |             |       |
| M2   |             |              |       |             |       |
| M3   |             |              |       |             |       |
| M4   |             |              |       |             |       |

### 步骤 2：低速正转——速度符号验证

```bash
motor 200 200       # 低速前进（200‰）
# 观察 3 秒
s                   # 记录四路 speed_mm/s 符号
motor 0 0
```

**判断标准**：前进时四路 `speed_mm/s` 均应为**正值**。若某路为负值，将 `control_config.h / bsp_config.h` 中对应的 `CHASSIS_Mx_ENCODER_DIR` 改为 `-1`。

| 电机 | speed_mm/s (前进时) | 符号正确？ | 如需修正 |
|------|---------------------|-----------|---------|
| M1   |                     |           |         |
| M2   |                     |           |         |
| M3   |                     |           |         |
| M4   |                     |           |         |

> 默认配置：M1/M2 编码器方向 = +1, M3/M4 = -1（因右侧电机物理安装方向与左侧相反）。

### 步骤 3：不同占空比下的速度测量

依次改变占空比，记录稳态速度：

```bash
log 1 motor         # CSV 日志仅记录 motor 字段
motor 150 150       # 150‰
# 等待 5 秒稳定
motor 300 300       # 300‰
# 等待 5 秒
motor 450 450       # 450‰
# 等待 5 秒
motor 600 600       # 600‰
# 等待 5 秒
motor 0 0
log 0
```

将 CSV 数据导入分析工具，提取每档占空比对应的稳态速度。

### 步骤 4：编码器超时行为

当车轮停止足够长时间后（> 100ms），`speed_valid` 应变为 0：

```bash
motor 300 300       # 启动
# 等待 3 秒
motor 0 0           # 急停
s                   # 立即查看：speed 仍有值
# 等待 2 秒
s                   # 再次查看：speed 已回零，valid 可能变为 0
```

**原理**：`CHASSIS_MAX_ENCODER_DT_MS = 100ms`。若编码器在 100ms 内无脉冲增量，判定速度无效（`valid=0`，速度强制归零），避免静止时的噪声被解读为微小速度。

### 步骤 5：编码器累计计数验证

转动某一路车轮一整圈（手动旋转），观察编码器 count 增量：

```bash
s                           # 记录初始 count
# 手动将 M1 车轮旋转一整圈
s                           # 记录最终 count
```

理论增量 = PPR × 4 × GearRatio = 11 × 4 × 56 = **2464 counts/rev**。

计算实际增量：count_final − count_initial。对比理论值和实际值的误差，分析误差来源（手动旋转不精确、齿轮间隙）。

## 数据分析

### 编码器速度-PWM 特性

```python
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv('lab02_encoder.csv')

# 按 PWM 档位分段，取稳态平均
df['pwm_level'] = pd.cut(df['m1_pwm'], bins=[0, 100, 250, 400, 550, 700, 1000],
                         labels=['150', '300', '450', '600'])
speed_cols = ['m1_mms', 'm2_mms', 'm3_mms', 'm4_mms']

fig, axes = plt.subplots(2, 2, figsize=(12, 8))
for i, (ax, col) in enumerate(zip(axes.flat, speed_cols)):
    means = df.groupby('pwm_level')[col].mean() / 1000
    stds = df.groupby('pwm_level')[col].std() / 1000
    ax.bar(range(len(means)), means.values, yerr=stds.values,
           capsize=5, color=['#3498db','#2ecc71','#f39c12','#e74c3c'])
    ax.set_title(f'M{i+1} Speed vs PWM')
    ax.set_xlabel('PWM (permille)')
    ax.set_ylabel('Speed (m/s)')
    ax.set_xticks(range(len(means)))
    ax.set_xticklabels(means.index)
    ax.grid(axis='y', alpha=0.3)
plt.tight_layout()
plt.savefig('lab02_pwm_speed.png', dpi=150)
```

**分析要点**：
- PWM-速度是否近似线性？拟合直线 R² 是多少？
- 四路电机的速度-占空比曲线是否一致？差异可能在什么范围？
- 低占空比（< 150‰）区域是否有明显非线性（死区效应）？

## 思考题

1. 推导本平台的编码器速度换算公式：从 ΔN (10ms 内的 count 增量) 到 mm/s 的完整表达式，代人实际参数计算缩放因子。

2. 为什么 M3/M4 的编码器方向默认为 −1，而 M1/M2 为 +1？从机械安装角度解释这种非对称性。

3. `CHASSIS_MAX_ENCODER_DT_MS = 100ms` 意味着最低可测速度是多少？（提示：需要至少 1 个脉冲才能触发 valid）

4. 如果因为齿轮间隙导致编码器在转向换向时出现短暂的速度尖峰，PID 控制器会如何响应？可以怎样从编码器层面缓解？

5. M 法测速在低速时精度下降的原因是什么？可以列举至少一种改进的测速方法（T 法、M/T 法）及其原理。

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `speed_mm/s` 始终为 0 | 编码器定时器未初始化或接线故障 | `rtos` 确认 motorTask 在运行；检查编码器 A/B 相接线 |
| 前进时某些路速度为负 | 编码器方向符号错误 | 修改对应 `CHASSIS_Mx_ENCODER_DIR = -1` |
| `speed_valid v=0` | 编码器超时——低速或静止时计数值不变 | 确认编码器接线，低于最低可测速度时 v=0 为正常 |
| 四路速度不一致 | 电机参数差异或轮径差异 | 差异 < 15% 正常（PID 闭环可补偿）；> 30% 检查机械或编码器 |  |
| count 增长但速度为零 | `CHASSIS_MAX_ENCODER_DT_MS` 设置过小 | 增大该宏值或增大占空比提高速度 |
