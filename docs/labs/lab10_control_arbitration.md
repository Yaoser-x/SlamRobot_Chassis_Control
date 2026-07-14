# Lab 10: 控制源仲裁与优先级抢占

## 实验目标

1. **知识**：理解多源控制仲裁的设计模式（优先级链、超时退让、reject-and-stop），掌握 `chassis_cmd_t` 的 timestamp 机制
2. **技能**：能通过 `status` 的 `source` 字段和 CSV 日志追踪控制源切换事件，验证每个源的超时和抢占行为
3. **素养**：理解"多源并存必须仲裁"的架构原则——没有仲裁的多源系统是不安全的

## 实验原理

### 1. 控制源优先级

```
UPPER (USART3/RPI)  >  PS2 手柄  >  ESP12F (WiFi)  >  LINE (巡线)  >  DEBUG (USART1)
   优先级 1                2              3                 5               4
```

> LINE(5) 的枚举值比 DEBUG(4) 大，但优先级在 Code 层面通过遍历数组 `{UPPER, PS2, ESP12F, LINE, DEBUG}` 确定，与枚举值无关。

### 2. 仲裁算法

`ControlService_GetCommand()` 按优先级数组顺序遍历各源命令槽：

```
for each source in [UPPER, PS2, ESP12F, LINE, DEBUG]:
    cmd = slot[source]
    if cmd.enable == 1:
        if (now_ms - cmd.timestamp_ms) <= timeout[source]:
            return cmd                             ← 命中，忽略更低优先级
return empty_cmd                                   ← 全部未命中 → 停止
```

### 3. 超时退让

每个控制源的命令携带 `timestamp_ms`（发送时刻的 tick）。代码按源使用独立超时：UPPER 为 200ms，PS2/ESP12F 为 500ms，LINE 为 50ms，DEBUG 为 2000ms。命令超过对应窗口后自动失效，仲裁跳过此源，不会永久锁死。

```
场景: PS2 手柄突然断开
  PS2 最后一次命令的 timestamp = T_0
  500ms 后: T_now - T_0 > 500ms
  → PS2 命令槽失效 → 仲裁跳过 PS2 → 落到更低优先级源或停止
```

### 4. reject-and-stop 语义

`ControlService_SetCommand()` 在以下情况拒绝命令：

| 条件 | 返回值 | 行为 |
|------|--------|------|
| ESTOP 激活 | REJECTED | 拒绝 |
| fault-stop 激活 | REJECTED | 拒绝 |
| 源 ID 非法 | REJECTED | 拒绝 |
| 速度值为 NaN/Inf | REJECTED_AND_STOPPED | 拒绝 + **清空该源槽位** |
| enable=0 | (特殊) | 清空该源槽位 |
| 运动学无效（ω≠0 且轮半径/轴距≤0） | ACCEPTED (clamp) | 拒绝角速度分量 |

### 5. 普通命令的持续刷新

`vel` 命令（DEBUG 源）不会自动持续——`debugTask` 每 10ms 周期调用 `ControlService_SetCommand()` 重新刷新 timestamp。这就是为什么输入一次 `vel 100` 后底盘持续运动：debugTask 在不断续期。

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘（**架空**）、12V 电池、USB-TTL 串口模块、PS2 手柄（可选） |
| 软件 | 串口终端 |

## 实验步骤

### 步骤 1：单源命令超时观察

```bash
# 使用 vel 命令创建 DEBUG 源命令
vel 100             # 底盘前进，source=4 (DEBUG)
s                   # 确认 source=4
# 记录当前 tick 和 speed
```

由于 debugTask 每 10ms 自动刷新 DEBUG 命令，`vel` 不会自然超时。要观察超时行为，需用 `stop` 清除自动刷新：

```bash
stop                # 清除 DEBUG 命令槽，停止自动刷新
s                   # source 回退为 NONE(0)，底盘停止
```

### 步骤 2：低优先级被高优先级抢占

需两个控制源同时在线。用串口 `vel` 作为 DEBUG (低优先级) + PS2 手柄 (高优先级)：

```bash
vel 100             # DEBUG 源: 前进 100mm/s
s                   # source=4 (DEBUG)
# 推 PS2 手柄左摇杆 → source 变为 2 (PS2)
s                   # 确认 source=2，速度变为 PS2 的值
# 释放 PS2 摇杆 (归中死区内) → source 回退为 4 (DEBUG)
s                   # 确认 source=4，恢复 vel 100
stop
```

### 步骤 3：巡线与 PS2 的优先级关系

```bash
line on             # 启用巡线
# 将黑线放在传感器下方
s                   # source=5 (LINE)（若有黑线）
# 推 PS2 摇杆 → source=2 (PS2), 巡线被抢占
# 释放摇杆 → source=5 (LINE)（若仍有黑线）
line off
```

**体会**：PS2 优先级高于 LINE → 操作员随时可以用摇杆"接管"巡线，释放后 LINE 自动恢复。这是安全设计：人 > 自动。

### 步骤 4：ESTOP 拒绝所有源

```bash
vel 100             # DEBUG 源
estop 1             # 紧急停止 → 所有源被清除
s                   # source=0, estop=1
# 推 PS2 摇杆 → 被 ESTOP 拒绝，底盘不动
estop 0             # 清除 ESTOP
# 重新推 PS2 摇杆 → 恢复
```

### 步骤 5：控制源切换时序追踪

```bash
log 1 source        # 仅追踪控制源切换
vel 200             # DEBUG
# 等待 5 秒
vel 200 500         # DEBUG + 角速度
# 等待 5 秒
stop                # 清除
log 0
```

从 CSV 中读出 `source` 列的值序列，验证 source 按预期变化。

## 数据分析

### 控制源切换事件检测

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('lab10_source.csv')
t = df['t_ms'] / 1000.0

source_names = {0: 'NONE', 1: 'UPPER', 2: 'PS2', 3: 'ESP12F', 4: 'DEBUG', 5: 'LINE'}
colors = {0: 'gray', 1: 'red', 2: 'blue', 3: 'orange', 4: 'green', 5: 'purple'}

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

# Source timeline
source_color = [colors.get(s, 'black') for s in df['source']]
ax1.scatter(t, df['source'], c=source_color, s=10, alpha=0.7)
ax1.set_yticks(range(6))
ax1.set_yticklabels([source_names[i] for i in range(6)])
ax1.set_ylabel('Control Source')
ax1.grid(alpha=0.3)

# Source switch events
transitions = df['source'].diff().fillna(0) != 0
switch_times = t[transitions]
switch_from = df['source'].shift(1)[transitions].astype(int)
switch_to = df['source'][transitions].astype(int)

for i, (st, fr, to) in enumerate(zip(switch_times, switch_from, switch_to)):
    ax2.annotate(f'{source_names[fr]}→{source_names[to]}',
                 (st, 1), rotation=45, fontsize=8, ha='left')
ax2.set_ylim(0, 2)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Switch Event')
ax2.grid(alpha=0.3)

plt.suptitle('Lab 10: Control Source Arbitration Timeline')
plt.tight_layout()
plt.savefig('lab10_arbitration.png', dpi=150)

# Statistics
print(f'Total source switches: {transitions.sum()}')
for src in range(6):
    total_ms = (df['source'] == src).sum() * 0.5  # 500ms log period
    print(f'{source_names[src]}: {total_ms:.1f}s ({total_ms/(len(df)*0.5)*100:.1f}%)')
```

## 思考题

1. 为什么通信控制源使用 500ms、LINE 使用 50ms、DEBUG 使用 2000ms？如果所有源统一设为 50ms 或 5s，分别会出现什么问题？

2. 上位机 (UPPER) 优先级最高。假设上位机崩溃并持续发送 `vel 500 0`，操作员能否用手柄或 `estop` 命令阻止底盘？如果可以，是通过什么机制？

3. `reject-and-stop` 在检测到 NaN/Inf 时不仅拒绝还清空该源槽位。为什么不能只拒绝不清空？（提示：考虑命令槽是持久的——该源下次再 SetCommand 之前，旧命令会残留吗？）

4. 巡线源 `LINE(5)` 的丢线退让（无黑线 → ClearSource）和超时退让（50ms 无有效帧 → ClearSource）是双重安全网。两者是否重叠？是否可能发生一个触发而另一个不触发的情况？

5. 如果未来要加入"语音控制源"（通过 Bluetooth 传输），你会把它放在优先级链的什么位置？需要什么额外的安全机制？

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `vel` 命令后底盘立刻停止 | DEBUG 源被更高优先级源抢占 | `s` 查看 source 是否为更高优先级 |
| PS2 摇杆释放后底盘不停 | 另一源持续发送命令 | `s` 确认 source；`stop` 清除所有 |
| `source` 在 2 个值之间快速切换 | 两源交替刷新 | 检查高频源（RPI 50ms/ESP 100ms）的命令间隔 |
