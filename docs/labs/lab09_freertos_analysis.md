# Lab 09: FreeRTOS 实时性能分析

## 实验目标

1. **知识**：理解 RTOS 任务调度（优先级抢占、时间片轮转）、栈内存模型、`osDelayUntil` 精确周期调度
2. **技能**：能通过 `rtos` 命令诊断任务健康度（stack_free、missed、heap_min），评估系统实时性
3. **素养**：建立"嵌入式系统的瓶颈在运行时"意识——内存和 CPU 时间都是有限资源，需要量化分析

## 实验原理

### 1. 任务模型

本固件共有 11 个任务（`freertos.c` + `chassis_tasks.h`）：

| 任务 | 优先级 | 周期 | 调度方式 | 栈 (W) | 核心职责 |
|------|--------|------|----------|--------|----------|
| safetyTask | High | 20ms | osDelayUntil | 状态聚合、故障检测 |
| motorTask | AboveNormal | 10ms | osDelayUntil | 编码器+PID+PWM |
| rpiCommTask | Normal | 5ms | osDelayUntil | 上位机 USART3 |
| imuTask | Normal | 10ms | osDelayUntil | BMI270 100Hz 采样 |
| ps2Task | Normal | 20ms | osDelayUntil | PS2 手柄 |
| lineTask | BelowNormal | 5ms | osDelayUntil | UART4 巡线帧 |
| espTask | BelowNormal | 5ms | osDelayUntil | ESP12F 协议 |
| debugTask | BelowNormal | 10ms | osDelay | 调试台 |
| oledTask | Low | 100ms | osDelayUntil | OLED 显示 |
| ledTask | Low | 50ms | osDelayUntil | 状态 LED |
| defaultTask | Low | 1ms | osDelay | 空闲保活 |

### 2. 优先级与抢占

```
High > AboveNormal > Normal > BelowNormal > Low > Idle

  safetyTask (High, 20ms)
    ↓ 抢占
  motorTask (AboveNormal, 10ms)
    ↓ 抢占
  rpiCommTask / imuTask / ps2Task (Normal)
    ↓ 抢占
  lineTask / espTask / debugTask (BelowNormal)
    ↓ 抢占
  oledTask / ledTask / defaultTask (Low)
```

高优先级任务就绪时立即抢占低优先级任务的 CPU。同一优先级任务按时间片轮转。

### 3. osDelayUntil —— 精确周期

```c
uint32_t wake_time = osKernelGetTickCount();
for (;;) {
    DoWork();                                    // 实际工作
    wake_time += PERIOD_TICKS;
    osDelayUntil(wake_time);                     // 等到下一周期
    missed = (osKernelGetTickCount() > wake_time); // 检测是否错过
}
```

若 `DoWork()` 耗时超过周期 → `missed` 递增 → 实时性恶化。

### 4. 栈内存模型

每个任务有独立的栈空间（`osThreadAttr_t.stack_size × 4 字节`）。`osThreadGetStackSpace()` 返回剩余栈空间。

```
┌────────────────────┐  stack top (SP 初始值)
│  剩余空间           │  ← stack_free
│  (未使用)          │
├────────────────────┤  SP 当前位置
│  已用栈            │
│  (局部变量+调用链)  │
├────────────────────┤  stack bottom
```

`stack_free < 256B` → 有溢出风险，需增大栈配置。

### 5. Heap 内存

`xPortGetFreeHeapSize()` 返回 FreeRTOS 堆剩余空间。`xPortGetMinimumEverFreeHeapSize()` 返回历史最低水位——这是判断是否存在内存泄漏的关键指标。

```
heap_min 持续下降 → 内存泄漏
heap_min 稳定在 > 10KB → 正常
```

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘、12V 电池、USB-TTL 串口模块 |
| 软件 | 串口终端 |

## 实验步骤

### 步骤 1：正常状态基线

上电后等待 30 秒（让系统充分初始化），记录 `rtos` 输出：

```bash
rtos
```

输出示例：
```
RTOS heap_free=85304B heap_min=85120B tick=15234
RTOS default   state=1 stack_free=124B missed=0
RTOS safety    state=2 stack_free=856B missed=0
RTOS motor     state=2 stack_free=1024B missed=0
RTOS rpi       state=2 stack_free=768B missed=0
RTOS imu       state=2 stack_free=640B missed=0
RTOS line      state=2 stack_free=480B missed=0
RTOS esp       state=2 stack_free=512B missed=0
RTOS debug     state=2 stack_free=896B missed=0
RTOS ps2       state=2 stack_free=380B missed=0
RTOS led       state=2 stack_free=200B missed=0
RTOS oled      state=2 stack_free=420B missed=0
RTOS comm upper_tx=0 upper_drop=0 esp_tx=0 esp_drop=0
```

**记录基线**：

| 任务 | 状态 | stack_free (B) | missed |
|------|------|----------------|--------|
| safety | | | |
| motor | | | |
| rpi | | | |
| imu | | | |
| line | | | |
| esp | | | |
| debug | | | |
| ps2 | | | |
| led | | | |
| oled | | | |
| default | | | |

### 步骤 2：负载加压——观察 missed

同时启动多种操作，增加 CPU 负载：

```bash
log 1 motor adc imu errors source ps2 line esp    # 全字段日志（最大负载）
vel 300 500         # 闭环运动
# 保持 30 秒
rtos                # 观察哪些任务出现 missed
vel 0
log 0
```

**预期**：高负载下可能 low-priority 任务（ledTask、oledTask）出现少量 missed，核心任务（safety/motor）应保持 `missed=0`。

### 步骤 3：栈消耗测试

不同操作对应不同调用深度，栈消耗不同：

```bash
rtos                # 基线 stack_free

# 执行一条调用链较深的命令（如 status）
s                   # 触发所有子系统 GetState() 调用
rtos                # 对比 debugTask stack_free 变化

# 启动日志流
log 1 motor adc imu errors source ps2 line esp
# 等待 10 秒
rtos                # 再次对比
log 0
```

**观察**：`debugTask` 的 `stack_free` 在打印全字段日志时是否明显减少（`snprintf` 需要较大的临时缓冲区：`DEBUG_CONSOLE_TX_LINE_SIZE = 1536B`）。

### 步骤 4：Heap 水位监测

长时间运行（> 10 分钟）观察 heap_min 变化：

```bash
rtos                # 记录 heap_min 初始值
# ... 等待 10 分钟，期间可以随意操作
rtos                # 记录 heap_min 最终值
```

**判断**：
- heap_min 基本不变 → 无内存泄漏
- heap_min 持续下降 → 存在未释放的内存分配（检查 `pvPortMalloc` 调用是否都有对应的 `vPortFree`）

### 步骤 5：任务状态观察

`osThreadGetState()` 返回值含义：

| state | 含义 |
|-------|------|
| 1 | Ready（就绪，等待 CPU） |
| 2 | Blocked（阻塞，等待延时/信号量/队列） |
| 3 | Suspended（挂起） |
| 4 | Terminated（终止） |

```bash
rtos
# 大多数任务 state=2 (Blocked)，因为它们在 osDelayUntil 中等待
# defaultTask state=1 (Ready)，因为它是 Low 优先级且 1ms 周期
```

## 数据分析

### 任务时序可视化

从 CSV 日志中提取 `source` 变化和 `missed` 信息，可视化任务切换：

```python
# 注意：CSV 日志不直接输出 missed 数
# 可以通过周期性执行 rtos 命令并手动记录来构建时序数据

import pandas as pd
import matplotlib.pyplot as plt

# 手动记录: timestamp, safety_missed, motor_missed, ...
df = pd.read_csv('lab09_missed_log.csv')

fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
axes[0].plot(df['time_s'], df['safety_missed'], 'r-o', label='safety (High)', markersize=4)
axes[0].plot(df['time_s'], df['motor_missed'], 'b-o', label='motor (AboveNormal)', markersize=4)
axes[0].set_ylabel('Missed Count')
axes[0].legend()
axes[0].grid(alpha=0.3)

axes[1].bar(df['time_s'], df['heap_free']/1024, width=5, alpha=0.7)
axes[1].set_ylabel('Heap Free (KB)')
axes[1].set_xlabel('Time (s)')
axes[1].grid(alpha=0.3)

plt.suptitle('Lab 09: RTOS Real-time Performance')
plt.tight_layout()
plt.savefig('lab09_rtos_perf.png', dpi=150)
```

## 思考题

1. `safetyTask` 优先级为 High，`motorTask` 为 AboveNormal。如果 `safetyTask` 的执行时间偶尔超过 10ms，`motorTask` 会怎样？（提示：考虑优先级抢占和 `osDelayUntil` 的 missed 检测）

2. `debugTask` 使用 `osDelay`（相对延时）而不是 `osDelayUntil`（绝对延时）。两者的区别是什么？为什么 debugTask 用相对延时就足够了？

3. 假设 `motorTask` 栈深度在最坏情况下为 896B，当前 `stack_free=1024B`。余量是否足够？如果不够，增大栈的开销是什么？

4. `defaultTask` 优先级最低且周期仅 1ms。为什么设计一个如此高频的低优先级任务？（提示：参考 STM32 CubeMX FreeRTOS 默认任务的设计意图）

5. 如果要在不修改固件的情况下测量每个任务的 CPU 使用率，FreeRTOS 提供了哪些 hook 函数？`vApplicationTickHook` 和 `Tracealyzer` 的测量原理是什么？

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `heap_free` 极低 (< 5KB) | `configTOTAL_HEAP_SIZE` 不足或内存泄漏 | 增大 FreeRTOS heap；排查 malloc-free 配对 |
| 高优先级任务 `missed` > 0 | 任务执行时间超过周期 | 优化任务逻辑；增大周期；提高 CPU 频率 |
| 某任务 `stack_free` 持续下降 | 递归调用过深或局部变量过大 | 增大对应任务栈（`freertos.c` 中 `stack_size`，+128W） |
| `defaultTask` 显示 `missing` | 任务创建失败 | 检查 heap 是否耗尽 |
