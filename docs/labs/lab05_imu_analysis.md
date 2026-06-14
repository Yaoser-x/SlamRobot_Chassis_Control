# Lab 05: IMU 与姿态估计

## 实验目标

1. **知识**：理解 MEMS 陀螺仪与加速度计的工作原理、互补滤波姿态融合、陀螺零偏校准的必要性
2. **技能**：掌握 IMU 初始化→校准→数据采集的完整流程，能用 Python 分析加速度/陀螺/欧拉角时序数据
3. **素养**：建立传感器噪声与漂移的量化直觉（陀螺积分误差 ∝ √t），理解"没有任何传感器是完美的"

## 实验原理

### 1. BMI270 传感器规格

| 参数 | 配置值 | 说明 |
|------|--------|------|
| 加速度计量程 | ±2g | `GYR_RANGE=0x02` |
| 陀螺仪量程 | ±500 dps | 对应灵敏度 65.6 LSB/dps |
| 输出速率 (ODR) | 100 Hz | acc + gyro 同步采样 |
| 接口 | SPI2 (PB13/PB14/PB15) | Mode 0, 10 MHz |

### 2. 加速度计与倾角

静止时加速度计测量重力矢量在传感器坐标系中的投影，可直接反算 roll 和 pitch：

```
roll  = atan2(ay, az)   — 绕 X 轴旋转
pitch = atan2(-ax, √(ay² + az²))  — 绕 Y 轴旋转
```

但加速度计对振动和运动加速度敏感，动态时倾角估计不可靠。

### 3. 陀螺仪与角速度积分

陀螺仪测量角速度 (dps)，积分得到角度：

```
angle(k) = angle(k−1) + gyro(k) × Δt
```

短期精度高，但零偏误差随时间累积（积分漂移）。即使零偏仅 0.1 dps，10 秒后漂移 = 1°。

### 4. 互补滤波

融合加速度计（长期稳定但短期噪声大）和陀螺仪（短期精确但长期漂移）：

```
roll(k)  = (1−α) × (roll(k−1) + gyro_x × Δt) + α × roll_acc
pitch(k) = (1−α) × (pitch(k−1) + gyro_y × Δt) + α × pitch_acc
α = 0.02（加速度计权重 2%，陀螺仪权重 98%）
```

yaw 无绝对参考（加速度计无法测量绕重力方向的旋转），纯用陀螺积分。

### 5. EMA 低通滤波

对原始 acc/gyro 数据做 EMA 滤波（α=0.20），抑制高频噪声后再送入互补滤波。

### 6. 陀螺零偏校准

静止状态下陀螺仪的输出不为零——存在零偏（bias）。`imucal` 命令：
1. 采样 N 个陀螺值（静止）
2. 计算 per-axis 平均值作为 bias
3. 验证静止条件（span ≤ 5 dps 且每轴 span ≤ 20 dps），超限则拒绝校准
4. 运行时 `gyro_corrected = gyro_raw − bias`

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘（静止放置在水平桌面）、USB-TTL 串口模块 |
| 软件 | 串口终端 + Python 3 + pandas + matplotlib |

## 实验步骤

### 步骤 1：芯片探测与初始化

```bash
imutest             # 探测 BMI270，期望 "bmi270 probe ok"
```

若返回 `probe failed`，执行 `imudiag` 诊断 SPI 通信：

```bash
imudiag             # SPI 硬件诊断
```

输出解读：
```
BMI270 diag hal1 st=0 rx=24,00,00 hal2 st=0 rx=24,00,00
BMI270 diag bitbang rx=24,00,00 miso nopull=0 pullup=0 pulldown=0
```
- `hal st=0` 表示 HAL_SPI 读取成功（HAL_OK）
- `rx=24` 即 chip_id 0x24
- bitbang 回退读取验证硬件

```bash
imuinit             # 加载 Bosch 配置表并初始化，期望 "bmi270 init ok"
```

### 步骤 2：陀螺零偏校准

将底盘**静止**放置在水平桌面上：

```bash
imucal              # 自动采样数校准
```

输出：
```
bmi270 gyro calibration: keep still
bmi270 gyro calibration ok bias_mdps=15,-32,8
```

三个值分别为 X/Y/Z 轴零偏（单位：毫度/秒）。

若校准失败（`calibration failed: keep still and retry`），说明校准时底盘有晃动，重新执行。

```bash
imucal 500          # 手动指定 500 次采样（约 5 秒 @ 100Hz）
imu 1               # 启用 IMU 周期采样
s                   # 查看 BMI270 状态行，确认 gcal=1
```

### 步骤 3：静止漂移观测

校准后保持静止，记录 30 秒数据：

```bash
log 1 imu           # CSV 日志仅 IMU 字段
# 保持底盘完全静止 30 秒
log 0
```

**分析重点**：
- yaw 在 30 秒内漂移了多少度？换算为 °/s
- roll 和 pitch 是否稳定在 ±1° 以内？
- 陀螺三轴 corrected 值是否接近零（±50 mdps）？

### 步骤 4：手动旋转测试

用手缓慢旋转底盘，观察欧拉角变化：

```bash
log 1 imu
# 绕 Z 轴（yaw）旋转 90°——顺时针看 yaw 减小，逆时针增大
# 绕 X 轴（roll）倾斜——左侧抬起 roll 为正
# 绕 Y 轴（pitch）倾斜——前部抬起 pitch 为正
# 恢复水平静止
log 0
```

**验证**：
- 物理旋转 90° 后 yaw 变化量是否接近 90°（±5°）？
- pitch/roll 在纯 yaw 旋转时是否保持稳定（无交叉耦合）？

### 步骤 5：振动分析

```bash
log 1 imu
vel 150             # 低速前进（架空），电机产生振动
# 保持 10 秒
vel 300
# 保持 10 秒
vel 0
log 0
```

**观察**：
- 电机运转时加速度计三轴出现高频分量（mg 级）
- gyro_filtered 比 gyro_corrected 更平滑（EMA 效果）
- 振动频率是否与电机转速相关？

## 数据分析

### 静止漂移与 Allan 方差

```python
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_csv('lab05_static.csv')
t = df['t_ms'] / 1000.0

fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

# 加速度计
axes[0].plot(t, df['imu_acc_x_mg'], label='X', alpha=0.7)
axes[0].plot(t, df['imu_acc_y_mg'], label='Y', alpha=0.7)
axes[0].plot(t, df['imu_acc_z_mg'], label='Z', alpha=0.7)
axes[0].set_ylabel('Accel (mg)')
axes[0].legend()
axes[0].grid(alpha=0.3)

# 陀螺仪（校正后）
axes[1].plot(t, df['imu_gyro_corr_x_mdps']/1000, label='X corr', alpha=0.7)
axes[1].plot(t, df['imu_gyro_corr_y_mdps']/1000, label='Y corr', alpha=0.7)
axes[1].plot(t, df['imu_gyro_corr_z_mdps']/1000, label='Z corr', alpha=0.7)
axes[1].set_ylabel('Gyro (dps)')
axes[1].legend()
axes[1].grid(alpha=0.3)

# 欧拉角
axes[2].plot(t, df['imu_roll_mdeg']/1000, label='Roll', alpha=0.7)
axes[2].plot(t, df['imu_pitch_mdeg']/1000, label='Pitch', alpha=0.7)
axes[2].plot(t, df['imu_yaw_mdeg']/1000, label='Yaw', alpha=0.7)
axes[2].set_ylabel('Angle (deg)')
axes[2].set_xlabel('Time (s)')
axes[2].legend()
axes[2].grid(alpha=0.3)

plt.suptitle('Lab 05: IMU Static Drift Analysis')
plt.tight_layout()
plt.savefig('lab05_imu_static.png', dpi=150)

# 漂移速率估算
yaw_drift_rate = (df['imu_yaw_mdeg'].iloc[-1] - df['imu_yaw_mdeg'].iloc[0]) / (t.iloc[-1] - t.iloc[0])
print(f'Yaw drift rate: {yaw_drift_rate:.2f} mdeg/s ({yaw_drift_rate/1000:.4f} deg/s)')
print(f'Projected drift in 60s: {yaw_drift_rate * 60 / 1000:.2f} deg')
```

### 振动频谱分析

```python
from scipy import signal

# 提取电机运转时的加速度数据
df_motor = df[(df['t_ms'] > motor_start_ms) & (df['t_ms'] < motor_end_ms)]
fs = 100.0  # 采样率 100 Hz

f, Pxx = signal.welch(df_motor['imu_acc_z_mg']/1000, fs, nperseg=256)
plt.figure(figsize=(10, 4))
plt.semilogy(f, Pxx)
plt.xlabel('Frequency (Hz)')
plt.ylabel('PSD (g²/Hz)')
plt.title('Motor Vibration Spectrum')
plt.grid(True, alpha=0.3)
plt.savefig('lab05_vibration_spectrum.png', dpi=150)
```

## 思考题

1. 互补滤波中 α=0.02 意味着加速度计的权重仅 2%。如果 α 增大到 0.10，roll/pitch 估计在静止和运动时分别会有什么变化？

2. yaw 为什么不能像 roll/pitch 那样用互补滤波？要获得 yaw 的绝对参考，需要什么额外传感器？

3. 陀螺校准的静止验证要求 span ≤ 5 dps。如果校准时底盘放在运转的电机旁边（有轻微振动），校准会失败还是会得到错误的 bias？错误的 bias 对后续姿态估计有何影响？

4. 加速度计测倾角的公式假设传感器静止（只受重力）。如果底盘在加速前进，`roll = atan2(ay, az)` 会得到正确值吗？为什么？

5. 如果要从加速度计数据中识别底盘是否在运动（vs 静止），可以用哪些特征？（提示：加速度方差、频谱能量）

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `imutest` 返回 failed | SPI 接线或芯片供电异常 | `imudiag` 诊断；检查 PB13/14/15 接线 |
| `imuinit` 成功但 `status` 中 `online=0` | 运行中通信丢失 | 驱动会自动重连（1s 间隔），等待即可 |
| `imucal` 反复失败 | 底盘未完全静止 | 将底盘放在稳固桌面上，等待 2 秒后重试 |
| yaw 漂移很快（>1°/s） | 陀螺零偏过大或校准不正确 | 重新 `imucal`，确认静止条件 |
| 欧拉角在某角度附近跳变 | 万向节死锁（gimbal lock） | pitch 接近 ±90° 时 roll/yaw 奇异性，属正常现象 |
