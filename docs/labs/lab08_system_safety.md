# Lab 08: 系统安全与故障诊断

## 实验目标

1. **知识**：理解嵌入式安全系统的分层设计（硬件 BKIN → 软件 ESTOP → fault-stop → 过流锁存），掌握 error flag 位掩码机制
2. **技能**：能通过 `status`/`errors` 诊断故障来源，理解 `clearfault` 的条件语义（不清除未恢复的故障）
3. **素养**：建立"安全优先于功能"的设计理念——宁可误停机，不可危险运行

## 实验原理

### 1. 安全层次架构

```
┌─────────────────────────────────┐
│ Level 1: 硬件 Break             │  TIM1/8 BKIN (PE15/PA6)
│  低电平 → 硬件切断 PWM (H桥高阻) │  释放后下一更新事件恢复 MOE
├─────────────────────────────────┤
│ Level 2: 软件 ESTOP             │  SYSTEM_ERROR_ESTOP (bit 5)
│  清空全部控制源 → 拒绝所有新命令  │  estop 1/0 命令 + 上位机帧
├─────────────────────────────────┤
│ Level 3: fault-stop             │  SYSTEM_ERROR_FAULT_STOP (bit 6)
│  清空全部命令 + 停PWM + Sleep    │  DRV nFAULT/过流 → clearfault
├─────────────────────────────────┤
│ Level 4: 过流保护 + DRV 故障     │  bits 1-4 (过流) + bit 8 (DRV)
│  逐路监控 + 去抖 + 锁存          │  clearfault（条件满足才清除）
└─────────────────────────────────┘
```

### 2. 错误标志位定义

TIM1/TIM8 均启用 Automatic Output。BKIN 低电平会立即清除对应定时器 MOE；BKIN 恢复后，下一更新事件自动恢复 MOE。该硬件事件当前不会自动设置软件 ESTOP，需通过 `status` 的 `BREAK tim1/tim8 moe/bif/count` 诊断。

`system_monitor.h` 的 9 个 error flag：

| 位 | 宏 | 含义 | 触发条件 | 是否锁存 |
|----|-----|------|----------|---------|
| 0 | `LOW_BATTERY` | 电池低压 | Vbat < 10.5V | 否（编译禁用） |
| 1 | `M1_OVERCURRENT` | M1 过流 | I > 0.8A × 5 次 | **是** |
| 2 | `M2_OVERCURRENT` | M2 过流 | I > 0.8A × 5 次 | **是** |
| 3 | `M3_OVERCURRENT` | M3 过流 | I > 0.8A × 5 次 | **是** |
| 4 | `M4_OVERCURRENT` | M4 过流 | I > 0.8A × 5 次 | **是** |
| 5 | `ESTOP` | 紧急停止 | `estop 1` 命令 | 否 |
| 6 | `FAULT_STOP` | 故障停机 | DRV 故障或过流触发 | **是** |
| 7 | `ENCODER_INVALID` | 编码器无效 | 超时无脉冲 | 否 |
| 8 | `DRV_FAULT` | DRV8874 硬件故障 | nFAULT 引脚拉低 | **是** |

锁存标志（latched）与实时标志（error）分离：锁存标志需 `clearfault` 清除，且前提是故障条件已解除。

### 3. 故障触发链

```
M1 电流 > 0.8A → 去抖 5 次 → SYSTEM_ERROR_M1_OVERCURRENT
    → latched → SYSTEM_ERROR_FAULT_STOP
    → ControlManager_SetFaultStop(1)
    → 清空全部控制源 + 停止 PWM + 拉低 DRV_SLEEP_ALL
    → 全部电机惯性滑行
```

### 4. clearfault 条件清除

`clearfault` 不会无条件清除所有 latched flag：

| 锁存故障 | 清除条件 |
|----------|---------|
| Mx_OVERCURRENT | 对应电机电流 < `MOTOR_CURRENT_LIMIT_A` |
| DRV_FAULT | 对应 nFAULT 引脚恢复高电平 |
| FAULT_STOP | 所有触发源（过流+DRV）均已清除 |

### 5. LED 指示

`BSP/led/led_status.c` 按状态模式闪烁：

| 模式 | 闪烁 | 含义 |
|------|------|------|
| NORMAL | 常亮 | 系统正常 |
| UPPER_LINK | 双闪 | 上位机连接 |
| LOW_BATTERY | 快闪 | 电池低压（< 10.5V） |
| FAULT | 3 短 1 长 | 故障停机 |
| ESTOP | 1 短 1 长 | 紧急停止 |

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘（**架空**）、12V 电池、USB-TTL 串口模块 |
| 软件 | 串口终端 |

## 实验步骤

### 步骤 1：ESTOP 行为验证

```bash
motor 300 300       # 开环前进
s                   # 确认底盘正在运动
estop 1             # 设置紧急停止
s                   # 确认 out=0, estop=1, 电机停止
motor 300 300       # 尝试重新驱动 → "motor test rejected: estop/fault active"
vel 100             # 尝试闭环 → "velocity command rejected"
estop 0             # 清除 ESTOP
motor 300 300       # 恢复驱动
motor 0 0
```

**关键验证**：ESTOP 激活期间，**所有**控制源（DEBUG/PS2/LINE/ESP/上位机）都被拒绝。

### 步骤 1.1：Break 状态观察

```bash
s
# BREAK tim1 moe=1 bif=0 count=0 tim8 moe=1 bif=0 count=0
```

正常情况下两组 `moe=1` 且 `count` 不持续增长。若教师使用安全夹具短暂拉低 BKIN，预期对应 `count` 增加、PWM 立即关闭；释放后 MOE 在下一更新事件恢复。禁止直接用裸导线短接带电引脚。

### 步骤 2：DRV 故障模拟

> **警告**：此步骤需要短暂模拟 nFAULT 低电平，请谨慎操作。

观察正常状态：

```bash
s
# SYS 行: drv_fault=0,0,0,0 （四路均正常）
```

如果存在某路 DRV 硬件故障：
- `drv_fault` 对应电机变为 1
- `errors` 中 bit 8 (`DRV_FAULT`) 置位
- fault-stop 触发（`fault=1`，全部电机停止）

> 实际实验时，可由教师提前在某一电机驱动板上设置故障（如断开电机相线触发 DRV8874 的 OCP 保护）。

### 步骤 3：过流触发与清除

复现 Lab 04 步骤 3 的堵转过流实验：

```bash
log 1 adc errors
vel 200
# 手动堵转 M1 约 3 秒
vel 0
log 0
```

```bash
s
# 观察 errors=0x00000002 (M1_OVERCURRENT bit1)
# latched=0x00000042 (M1_OVERCURRENT + FAULT_STOP)
# fault=1
```

等待电流回落后：

```bash
clearfault          # 清除锁存故障
s                   # 确认 latched 清零，fault=0
```

**如果 `clearfault` 后 latched 未被清除**，说明故障条件尚未解除（电流仍 > 0.8A 或 nFAULT 仍低）。

### 步骤 4：errors 位掩码解析

错误位掩码解读练习。`status` 中 `errors=0x00000042`：

```
0x00000042 = 0b 0100 0010
bit 6 = FAULT_STOP  (0x40)
bit 1 = M1_OVERCURRENT (0x02)
总计: FAULT_STOP + M1_OVERCURRENT
```

练习解析以下 errors 值，写出对应的错误组合：

| errors 值 | latched 值 | 故障组合 |
|-----------|-----------|---------|
| 0x00000000 | 0x00000000 | |
| 0x00000020 | 0x00000020 | |
| 0x00000100 | 0x00000142 | |
| 0x00000020 | 0x00000062 | |

### 步骤 5：LED 状态观察

结合不同故障状态，观察 LED 行为：

| 状态 | 命令 | LED 预期 |
|------|------|----------|
| 正常 | — | 常亮 |
| ESTOP | `estop 1` | 1 短 1 长闪烁 |
| 过流 | 堵转 M1 | 3 短 1 长闪烁 |
| 清除 | `clearfault` | 恢复常亮 |

## 数据分析

### 故障时序分析

从 CSV 日志的 `errors` 字段追踪故障发生和清除的时间线：

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('lab08_fault.csv')
t = df['t_ms'] / 1000.0

# 解析 error bits
error_bits = {
    'LOW_BATTERY': 0, 'M1_OC': 1, 'M2_OC': 2, 'M3_OC': 3, 'M4_OC': 4,
    'ESTOP': 5, 'FAULT_STOP': 6, 'ENC_INV': 7, 'DRV_FAULT': 8
}

fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

# M1 电流 + 阈值
axes[0].plot(t, df['m1_ma'], 'b-', alpha=0.7, label='M1 current')
axes[0].axhline(800, color='red', linestyle='--', label='limit 800mA')
axes[0].set_ylabel('Current (mA)')
axes[0].legend()
axes[0].grid(alpha=0.3)

# Error flags
for name, bit in error_bits.items():
    if (df['errors'] & (1 << bit)).any():
        axes[1].plot(t, (df['errors'] >> bit) & 1, label=name, alpha=0.7)
axes[1].set_ylabel('Error Flag')
axes[1].legend(loc='upper right')
axes[1].grid(alpha=0.3)

# source (观察故障后的回退)
axes[2].plot(t, df['source'], 'g-', alpha=0.7)
axes[2].set_ylabel('Control Source')
axes[2].set_xlabel('Time (s)')
axes[2].grid(alpha=0.3)

plt.suptitle('Lab 08: Fault Detection and Recovery Timeline')
plt.tight_layout()
plt.savefig('lab08_fault_timeline.png', dpi=150)
```

## 思考题

1. 硬件 TIM_BKIN 可以独立于软件立即切断 PWM。如果固件死锁（while(1) 不喂狗不退让），ESTOP 命令是否还能生效？硬件 BKIN 和软件 ESTOP 各自的不可绕过性如何？

2. `clearfault` 的条件清除语义（必须故障条件解除后才清除）是"安全"还是"不便"？如果不加条件检查，可能造成什么危害？

3. 过流去抖需要连续 5 次（100ms）才触发。为什么不设计为单次超阈值即触发？请估算：电机在堵转状态下 100ms 内温升大约多少？

4. LED 闪烁模式在嘈杂环境中（阳光下/远处/视觉障碍）可能不够显眼。如果要增加一个蜂鸣器作为声音报警，你会放在安全架构的哪一层？为什么？

5. `BATTERY_LOW_MONITOR_ENABLED` 当前为 `0U`（编译禁用）。如果启用，当电池电压低于 9.0V 时应该如何设计停机策略——立即停机还是先警告再降速？为什么？

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `clearfault` 无效 | 故障条件未解除（电流仍高/nFAULT 仍低） | `status` 确认具体值，修复硬件后再清除 |
| 上电即 fault-stop | 某路 DRV nFAULT 上电即低 | `s` 查看 `drv_fault`，检查对应驱动板 |
| ESTOP 清除后仍不能驱动 | 有其他锁存故障未清除 | `clearfault` + `estop 0` |
| LED 不按预期闪烁 | LED 任务可能饥饿 | `rtos` 检查 ledTask stack 和 missed 数 |
