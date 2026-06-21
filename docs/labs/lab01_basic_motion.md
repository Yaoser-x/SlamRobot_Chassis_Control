# Lab 01: 基础运动控制

## 实验目标

1. **知识**：理解 H 桥电机驱动原理、PWM 占空比与转速的关系、差速转向运动学模型
2. **技能**：掌握调试台开环运动命令（`motor`/`left`/`right`/`m1`-`m4`），能安全地启停电机
3. **素养**：建立"先架空、后落地"的实验安全意识，养成操作前检查 ESTOP 的习惯

## 实验原理

### 1. H 桥与 PWM 调速

DRV8874 为 H 桥电机驱动器，IN1/IN2 引脚控制桥臂开关状态：

```
IN1 = PWM, IN2 = 0  →  电机正转（Forward）
IN1 = 0,  IN2 = PWM →  电机反转（Reverse）
IN1 = 0,  IN2 = 0   →  惯性滑行（Coast）
IN1 = 1,  IN2 = 1   →  制动（Brake）
```

控制量单位为 **permille（‰）**，范围 ±900‰。正值表示前进方向，负值表示后退方向，绝对值越大转速越高。900‰ 对应约 90% 占空比，保留 10% 余量避免桥臂直通。

### 2. 差速转向原理

四轮差速底盘通过左右侧轮速差实现转向：

```
左轮速度 = V_linear - ω × (W/2)
右轮速度 = V_linear + ω × (W/2)

其中：V_linear = 线速度 (m/s), ω = 角速度 (rad/s), W = 轮距 (m)
```

| 运动模式 | 左侧速度 | 右侧速度 | 效果 |
|----------|---------|---------|------|
| 前进 | +V | +V | 直线前进 |
| 后退 | −V | −V | 直线后退 |
| 原地左转 | −V | +V | 绕中心逆时针旋转 |
| 原地右转 | +V | −V | 绕中心顺时针旋转 |
| 左前转弯 | +V/2 | +V | 向左前方弧线 |

### 3. 电机编号与布局

默认四驱配置（`chassis_config.h`）：

```
        前
        ↑
  [M1]     [M3]
  左侧      右侧
  [M2]     [M4]
        ↓
        后

M1/M2 = 左侧 (MOTOR_SIDE_LEFT)
M3/M4 = 右侧 (MOTOR_SIDE_RIGHT)
```

> V2.0 当前常用两驱默认启用 M1+M3，M2/M4 禁用；因此 `right 300` 在默认配置下只驱动逻辑 M3。实板 M3 的 PWM 为 `PE13/PC8`，nFAULT 为 `PA3`，编码器为 `TIM3 PB4/PB5`，电流采样为 `PC1`。

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘（架空状态）、12V 电池、USB-TTL 串口模块 |
| 软件 | 串口终端（115200 8N1），无需构建环境 |

## 实验步骤

### 步骤 1：启动与状态确认

底盘架空（四轮离地），连接串口终端，上电。终端应显示：

```
F407 V2 chassis firmware
USART1 debug console ready, type help
```

确认系统正常：

```bash
status
```

观察 `CHASSIS` 行：`estop=0 fault=0 out=0`，确认无紧急停止或故障。

输入 `help` 浏览全部命令。

### 步骤 2：单路电机测试

依次测试 M1–M4，每路执行以下序列：

```bash
m1 200 0        # M1 正转（前进方向），200‰ 占空比
m1 0 0          # 停止 M1
m1 0 200        # M1 反转（后退方向），200‰ 占空比
m1 0 0          # 停止 M1
```

重复 `m2`、`m3`、`m4` 命令。

**观察**：正转时车轮应向前进方向旋转；反转时向后倒退方向旋转。若某路方向相反，需修改 `chassis_config.h` 中对应的 `CHASSIS_Mx_MOTOR_DIR = -1` 并重新烧录。

记录每路电机的实际旋转方向：

| 电机 | 正转方向 (m1 200 0) | 反转方向 (m1 0 200) | 是否与预期一致？ |
|------|---------------------|---------------------|-----------------|
| M1   |                     |                     |                 |
| M2   |                     |                     |                 |
| M3   |                     |                     |                 |
| M4   |                     |                     |                 |

### 步骤 3：单侧聚合测试

```bash
left 300        # 左侧两轮 (M1+M2) 同时正转
left 0          # 停止左侧
right 300       # 右侧两轮 (M3+M4) 同时正转
right 0         # 停止右侧
```

**观察**：`left` 命令仅驱动左侧 M1+M2，右侧不动；`right` 命令仅驱动右侧 M3+M4。

> 默认两驱配置下 M2/M4 禁用，所以 `left 300` 只驱动 M1，`right 300` 只驱动 M3。

### 步骤 4：双侧同步——直线运动

```bash
motor 300 300   # 四轮同时正转（前进）
motor 0 0       # 停止
motor -300 -300 # 四轮同时反转（后退）
motor 0 0       # 停止
```

**观察**：前进时四轮同步向前，后退时四轮同步向后。

### 步骤 5：差速转向

```bash
motor 300 -300  # 原地左转（左侧反转、右侧正转）
motor 0 0       # 停止
motor -300 300  # 原地右转（左侧正转、右侧反转）
motor 0 0       # 停止
```

**观察**：原地左转时左侧轮向后转、右侧轮向前转，车体逆时针旋转。

### 步骤 6：单路 raw 控制和四路独立控制

```bash
# 单路 raw 控制（IN1 前进, IN2 后退）
m1 300 0        # M1 300‰ 正转
m1 0 300        # M1 300‰ 反转
m1 300 300      # M1 制动（IN1=IN2=300‰）

# 四路独立 raw 控制
raw 200 200 200 200    # 四路正转
raw 0 0 0 0            # 停止
raw 200 200 -200 -200  # 左侧前进、右侧后退（原地左转）
raw 0 0 0 0            # 停止
```

### 步骤 7：安全命令练习

```bash
estop 1         # 设置紧急停止
motor 300 300   # 尝试驱动 → 被拒绝："motor test rejected: estop/fault active"
estop 0         # 清除紧急停止
motor 300 300   # 恢复驱动
motor 0 0
stop            # 停止全部运动 + 清除测试命令
```

## 数据分析

### 开环 PWM-速度关系

使用 `log 1 motor` 记录不同 PWM 占空比下的电机速度：

```bash
log 1 motor                     # 启动 CSV 日志（仅 motor 字段）
motor 200 200                   # 200‰ 前进
# 等待 5 秒
motor 400 400                   # 400‰
# 等待 5 秒
motor 600 600                   # 600‰
# 等待 5 秒
motor 800 800                   # 800‰
# 等待 5 秒
motor 0 0
log 0                           # 停止日志
```

将串口日志保存为 CSV 文件，用 Python 绘制 PWM-速度曲线：

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('lab01_motor.csv')
plt.figure(figsize=(10, 6))
for motor in ['m1_mms', 'm2_mms', 'm3_mms', 'm4_mms']:
    plt.plot(df['t_ms'] / 1000, df[motor] / 1000, label=motor, alpha=0.7)
plt.xlabel('Time (s)')
plt.ylabel('Speed (m/s)')
plt.legend()
plt.title('Open-loop PWM vs Speed Response')
plt.grid(True)
plt.savefig('lab01_pwm_speed.png', dpi=150)
```

**分析要点**：
- 四路电机在同一 PWM 下的速度是否一致？
- PWM 与速度是否呈近似线性关系？在低占空比区（< 150‰）有没有死区？
- 增减速时是否有明显滞后？

## 思考题

1. 为什么 `motor 300 300` 是直线前进，而 `motor 300 -300` 是原地转向？用差速运动学公式推导两种情况下左右轮的速度大小和方向。

2. 如果将 M3 和 M4 的 `CHASSIS_Mx_SIDE` 从 `MOTOR_SIDE_RIGHT` 改为 `MOTOR_SIDE_LEFT`，`left 300` 命令会驱动哪些电机？为什么这种配置违反了安全规则？

3. 占空比从 200‰ 提升到 400‰ 时，车轮转速是否精确翻倍？如果不是，可能的原因有哪些（摩擦力、电机效率、PWM 死区）？

4. `motor 0 0`、`stop`、`estop 1` 三者有何区别？分别在什么场景下使用？

5. 如果某一路电机的正转方向与预期相反，除了修改软件配置外，硬件上可以怎么处理？

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `motor` 命令无响应 | ESTOP 或 fault 激活 | `status` 检查 `estop`/`fault` 标志，`estop 0` 或 `clearfault` |
| 某路电机不转 | DRV8874 nSLEEP 拉低或接线断开 | `status` 检查 `drv_fault`，万用表检查电机接线 |
| 四路电机都不转 | 电池电压过低或 DRV_SLEEP_ALL 未拉高 | `status` 检查 `vbat`；重新上电 |
| 电机转动方向与预期相反 | `CHASSIS_Mx_MOTOR_DIR` 符号错误 | 修改对应宏为 `-1`，重新构建烧录 |
| 车轮低速时有异响 | PWM 死区设置或机械间隙 | 增大 `MOTOR_DIRECTION_CHANGE_COAST_CYCLES` |
