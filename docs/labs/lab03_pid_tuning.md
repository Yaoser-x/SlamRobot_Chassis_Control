# Lab 03: PID 速度环调参

## 实验目标

1. **知识**：理解 PID 控制器三项（比例/积分/微分）的物理意义与离散化实现，掌握阶跃响应评价指标（超调量、稳态误差、调节时间）
2. **技能**：能通过修改配置参数 → 重编译 → 烧录 → CSV 日志观测的方式迭代调参，用 Python 定量评估控制品质
3. **素养**：建立"先 P → 再 I → 后 D"的分步调参方法论，理解盲目乱调参数的危害

## 实验原理

### 1. PID 控制器离散形式

本平台实现的是位置式 PID（`BSP/pid/pid_controller.c`）：

```
e(k) = target(k) − actual(k)
P(k) = Kp × e(k)
I(k) = I(k−1) + Ki × e(k) × Δt
D(k) = Kd × (e(k) − e(k−1)) / Δt
u(k) = P(k) + I(k) + D(k)
```

**三项的物理意义**：

| 项 | 作用 | 过大时表现 | 过小时表现 |
|----|------|-----------|-----------|
| **Kp** | 比例：响应速度 | 超调→振荡→发散 | 响应迟钝，稳态误差大 |
| **Ki** | 积分：消除稳态误差 | 积分饱和、低频振荡、超调加剧 | 稳态误差无法消除 |
| **Kd** | 微分：抑制超调，阻尼 | 放大高频噪声、PWM 抖动 | 超调大、调节时间长 |

### 2. 控制链全貌

```
vel V [W]  ──→  ControlService  ──→  ChassisService_Step (10ms)
                  ├─ 差速模型: linear_x/angular_z → left_mps/right_mps
                  ├─ 速度斜坡: 1.0 m/s² 平滑过渡
                  ├─ PID 速度环: 独立四路 PI(D)
                  ├─ 电流诊断: 默认记录 ADC 电流，软件过流锁停关闭
                  └─ 电机输出: TIM1 EN/PWM + GPIO PH/DIR, permille ±900
```

### 3. 阶跃响应评价指标

```
        超调量 Mp
        ┌──┐
        │  │      ┌───────────────── 稳态值
    目标│  │     ╱│
    ────┼──┼────╱─┼──────────────
        │  │  ╱   │
        │  │╱     │  ← 稳态误差 ess
        │ ╱│      │
        └──┼──────┼──────────────→ t
           ↑      ↑
        上升时间  调节时间 ts (±5%)
         tr
```

### 4. 当前 PID 参数配置

所有 PID 参数为 `chassis_config.h` 中的编译期常量（默认值）：

```c
// 每路电机独立 PID
#define CHASSIS_PID_KP_M1   50.0f
#define CHASSIS_PID_KI_M1   8.0f
#define CHASSIS_PID_KD_M1   0.05f
#define CHASSIS_PID_KP_M2   1000.0f
#define CHASSIS_PID_KI_M2   800.0f
#define CHASSIS_PID_KD_M2   0.15f
#define CHASSIS_PID_KP_M3   1200.0f
#define CHASSIS_PID_KI_M3   1000.0f
#define CHASSIS_PID_KD_M3   0.18f
#define CHASSIS_PID_KP_M4   100.0f
#define CHASSIS_PID_KI_M4   0.0f
#define CHASSIS_PID_KD_M4   0.0f

// 全局 PID 约束
#define CHASSIS_PID_INTEGRAL_LIMIT     60.0f    // 积分限幅
#define CHASSIS_PID_CORRECTION_LIMIT   500.0f   // PID 输出增量限幅
#define CHASSIS_PID_STOP_EPSILON_MPS   0.005f   // 停止判定阈值
```

> **注意**：当前 PID 参数通过修改 `chassis_config.h` → 重编译 → 烧录的方式调整。平台暂未实现运行时 `set` 命令。

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘（**架空**）、12V 电池、USB-TTL 串口模块、ST-Link V2 |
| 软件 | 串口终端 + arm-none-eabi-gcc + CMake 3.22+ + Ninja（构建环境） |
| 数据分析 | Python 3 + pandas + matplotlib |

## 实验步骤

### 步骤 0：确认构建环境

```bash
arm-none-eabi-gcc --version    # 确认工具链可用 (> 10.0)
cmake --version                # 确认 CMake (> 3.22)
```

### 步骤 1：基线响应——默认 Kp

底盘**架空**，测试当前默认 PID 参数下的阶跃响应：

```bash
log 1 motor                     # CSV 日志（仅 motor 字段：速度 + PWM）
vel 100                         # 阶跃目标 100mm/s
# 保持 10 秒观察响应
vel 0                           # 停止
# 保持 5 秒观察制动响应
log 0                           # 停止日志
```

重复多次取平均值。将日志保存为 `lab03_baseline.csv`。

**观察 `status` 中的关键指标**：

```
CHASSIS req=100,0mm/s target=100,0mm/s actual=89,92mm/s pwm=523,518,541,537 out=1
              │              │                │                  │
              │              │                │                  └─ PWM 输出值
              │              │                └─ 实际速度（编码器反馈）
              │              └─ 斜坡后目标速度
              └─ 请求速度（vel 命令原始值）
```

### 步骤 2：P 参数扫描

依次修改 `chassis_config.h` 中的 Kp 值，观察不同 Kp 下的阶跃响应：

| 实验编号 | Kp (M1/M2) | Kp (M3/M4) | 预期 |
|----------|------------|------------|------|
| A | 600 | 700 | 响应慢、稳态误差大 |
| B | 当前默认值 | 当前默认值 | 基线 |
| C | 2500 | 2800 | 响应快、可能超调 |
| D | 5000 | 5500 | 可能振荡 |

每次修改后执行：

```bash
cmake --build --preset Debug          # 重编译
# 通过 ST-Link 烧录 build/Debug/F407_V2.0.elf
# 重新连接串口，重复步骤 1 的阶跃测试
```

**记录模板**：

| 实验 | Kp_M1 | 上升时间(ms) | 超调量(%) | 稳态误差(mm/s) | PWM 抖动(permille) |
|------|-------|-------------|----------|---------------|-------------------|
| A    | 600   |             |          |               |                   |
| B    | 默认值 |             |          |               |                   |
| C    | 2500  |             |          |               |                   |
| D    | 5000  |             |          |               |                   |

### 步骤 3：加入积分项

选定较优的 Kp 值（可从当前默认值开始），在原基础上调整 Ki：

```c
#define CHASSIS_PID_KI_M1   50.0f    // 小幅积分
#define CHASSIS_PID_KI_M2   50.0f
#define CHASSIS_PID_KI_M3   60.0f
#define CHASSIS_PID_KI_M4   60.0f
```

重新构建、烧录、测试。

**观察重点**：
- 稳态误差是否被消除？（steady-state error → 0）
- 是否出现低频振荡？（积分饱和的典型症状）
- PWM 是否有低频漂移？

尝试不同 Ki 值（20 / 50 / 100 / 200），记录积分效果：

| 实验 | Ki_M1 | 稳态误差(mm/s) | 振荡周期(s) | 超调量(%) |
|------|-------|---------------|------------|----------|
| E    | 20    |               |            |          |
| F    | 50    |               |            |          |
| G    | 100   |               |            |          |
| H    | 200   |               |            |          |

### 步骤 4：加入微分项（可选）

若有明显超调，尝试加入 Kd：

```c
#define CHASSIS_PID_KD_M1   10.0f
// ...
```

**注意**：微分对编码器量化噪声敏感。若 PWM 出现高频抖动（>50 permille），Kd 过大，需要减小。

### 步骤 5：负载测试（落地）

> **安全警告**：落地测试前确认 `estop 1` 命令熟练、底盘四周无障碍物。

将底盘放置地面（非架空），重复一组较优参数的阶跃测试。对比空载 vs 带载的响应差异。

## 数据分析

### 阶跃响应指标提取

```python
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('lab03_baseline.csv')
t = df['t_ms'] / 1000.0

# 提取 M1 阶跃响应
target = 0.1  # 100 mm/s = 0.1 m/s
actual = df['m1_mms'] / 1000.0  # 转换为 m/s
pwm = df['m1_pwm']

# 找上升时间（首次到达 90% 目标的时间）
idx_start = np.argmax(actual > 0.005)  # 响应起始点
idx_rise = np.argmax(actual[idx_start:] >= 0.9 * target) + idx_start
rise_time = t[idx_rise] - t[idx_start]

# 找超调量
steady_state = actual[-100:].mean()  # 最后 100 个点的均值
overshoot = (actual.max() - target) / target * 100.0

# 找调节时间（进入 ±5% 范围后不再离开的时间）
band = 0.05 * target
in_band = np.abs(actual - target) <= band
# 从后往前找最后一个离开 band 的点
idx_settle = len(in_band) - np.argmax(in_band[::-1] == False) - 1
settling_time = t[idx_settle] - t[idx_start]

print(f'M1 阶跃响应指标:')
print(f'  上升时间: {rise_time*1000:.1f} ms')
print(f'  超调量:   {overshoot:.1f} %')
print(f'  调节时间: {settling_time*1000:.1f} ms')
print(f'  稳态误差: {(steady_state - target)*1000:.1f} mm/s')

# 绘图
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
ax1.plot(t, actual, 'b-', label='M1 actual', alpha=0.8)
ax1.axhline(target, color='gray', linestyle='--', label='target (0.1 m/s)')
ax1.axhline(target * 1.05, color='red', linestyle=':', alpha=0.5)
ax1.axhline(target * 0.95, color='red', linestyle=':', alpha=0.5)
ax1.set_ylabel('Speed (m/s)')
ax1.legend()
ax1.grid(alpha=0.3)

ax2.plot(t, pwm, 'r-', label='M1 PWM', alpha=0.8)
ax2.set_ylabel('PWM (permille)')
ax2.set_xlabel('Time (s)')
ax2.legend()
ax2.grid(alpha=0.3)

plt.suptitle('Lab 03: PID Step Response Analysis')
plt.tight_layout()
plt.savefig('lab03_step_response.png', dpi=150)
```

### 调参效果对比

将不同 Kp/Ki 组合的响应曲线叠加到同一张图：

```python
files = {'Kp=600': 'lab03_kp600.csv', 'default': 'lab03_baseline.csv',
         'Kp=2500': 'lab03_kp2500.csv', 'Kp+Ki=50': 'lab03_ki50.csv'}

for label, file in files.items():
    df = pd.read_csv(file)
    plt.plot(df['t_ms'] / 1000, df['m1_mms'] / 1000, label=label, alpha=0.8)

plt.axhline(0.1, color='gray', linestyle='--', label='target')
plt.legend()
plt.savefig('lab03_comparison.png', dpi=150)
```

## 思考题

1. 为什么不同电机的默认 PID 参数不一定相同？从机械布局、驱动特性和编码器反馈质量角度分析。

2. 纯 P 控制为什么会产生稳态误差？用终值定理或控制框图推导说明。加入积分项后稳态误差为什么能被消除？

3. 如果增大 Kp 至 5000，系统开始振荡。振荡频率由哪些因素决定（惯性、控制周期、编码器噪声）？

4. `CHASSIS_PID_INTEGRAL_LIMIT = 60.0` 的含义是什么？如果没有积分限幅，在目标速度突变为 0 时可能发生什么（积分饱和/windup）？

5. `CHASSIS_SPEED_RAMP_MPS2 = 1.0` 速度斜坡对阶跃响应有何影响？如果去掉斜坡限制（设为很大的值），PID 输出会有何变化？

6. 在架空和落地两种情况下，同一组 PID 参数的表现可能有何不同？为什么？（提示：负载惯量、摩擦力差异）

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `arm-none-eabi-gcc: command not found` | 工具链不在 PATH | 确认安装路径，加入环境变量 |
| 修改参数后行为没变化 | 未重新编译或烧录了旧 elf | `cmake --build --preset Debug` 确保重新链接 |
| PWM 高频抖动 | Kd 过大或编码器量化噪声 | 减小 Kd；检查编码器 count 是否有异常跳变 |
| 速度始终追不上目标 | Kp 过低、PWM 死区或过流保护触发 | 增大 Kp；`status` 检查 fault 与 ADC 电流 |
| 低频（~1Hz）振荡 | Ki 过大导致积分振荡 | 减半 Ki；增大 `CHASSIS_PID_INTEGRAL_LIMIT` |
