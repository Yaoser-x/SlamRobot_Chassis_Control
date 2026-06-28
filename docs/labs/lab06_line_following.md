# Lab 06: 巡线传感器与跟踪控制

## 实验目标

1. **知识**：理解红外反射式巡线原理、HiWonder 八路传感器协议、加权平均线位计算、P 控制跟踪算法
2. **技能**：能解读 `line` 命令输出的传感器原始数据，调节阈值/速度/Kp 优化跟踪效果
3. **素养**：建立"传感器→位置估计→控制输出"的感知-控制闭环思维

## 实验原理

### 1. HiWonder 八路巡线传感器

传感器通过 UART4（PC10/PC11, 115200 bps）以约 200 Hz 频率输出每一帧数据：

```
帧格式: 0x55 0xAA [cmd] [data 16B] [checksum] → 21 字节
cmd=0x02 (模拟量采样)
data: CH1_analog_H CH1_analog_L ... CH8_analog_H CH8_analog_L
checksum: 前 20 字节 XOR
```

8 路红外传感器按横向排列，黑线吸收红外光 → 对应通道模拟量降低。

### 2. 黑线检测

模拟量阈值判定（`LINE_ANALOG_THRESHOLD = 500`）：

```
analog < 500  →  传感器在该位置检测到黑线 (state=1)
analog ≥ 500  →  无黑线 (state=0)
```

### 3. 加权平均线位计算

8 路传感器的检测结果合并为单一位置估计：

```
position = Σ(i × state[i]) / Σ(state[i])    (i=0~7)
error = position − 3.5
```

其中 3.5 是中心位置（传感器 CH4 和 CH5 之间）。

| 线位置 | position | error | 含义 |
|--------|----------|-------|------|
| 偏左 | 0~3 | −3.5~−0.5 | 需右转纠偏 |
| 居中 | 3~4 | −0.5~+0.5 | 基本直行 |
| 偏右 | 4~7 | +0.5~+3.5 | 需左转纠偏 |

### 4. P 控制

```
angular_z = LINE_KP × error
angular_z = clamp(angular_z, ±LINE_ANGULAR_MAX_RPS)

控制命令: linear_x = LINE_SPEED_MPS, angular_z = P(error)
```

参数（`chassis_config.h`）：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `LINE_SPEED_MPS` | 0.15 m/s | 巡线前进速度 |
| `LINE_KP` | 2.5 | P 控制增益 |
| `LINE_ANGULAR_MAX_RPS` | 2.0 rad/s | 角速度上限 |
| `LINE_DETECT_THRESHOLD_COUNT` | 1 | 最少检测通道数 |
| `LINE_SENSOR_TIMEOUT_MS` | 50 ms | 传感器超时 |

### 5. 安全行为

- **默认关闭**：上电后巡线不启用，需手动 `line on`
- **丢线退让**：检测到的黑线通道数 < 1 → 清除 LINE 控制源 → 底盘停止
- **超时退让**：传感器 50ms 内无有效帧 → 清除 LINE 控制源
- **可被覆盖**：PS2 摇杆操作时 LINE 被优先级仲裁跳过

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘（**架空**→落地）、12V 电池、USB-TTL 串口模块、黑色胶带（制作测试赛道） |
| 软件 | 串口终端 |

## 实验步骤

### 步骤 1：通信验证

底盘架空，确认巡线传感器已连接 UART4：

```bash
line                # 查看传感器状态
```

输出示例：
```
LINE enabled=0 active=0 pos=0.00 err=0.00 lx=0.000 az=0.000 det=0
LINE st=00000000 an=782,810,790,805,795,788,801,775
LINE rx_bytes=21840 frames=1040 proto_err=0 ovf=0
```

**确认**：
- `rx_bytes` 持续增长（~4.2 KB/s @ 200Hz × 21B）
- `frames` 每秒约增加 200
- `proto_err` 保持 0（偶有 1-2 个噪声误码可接受）
- `ovf` 保持 0（UART DMA 无溢出）

连续输入 `line` 多次，观察 `rx_bytes` 和 `frames` 是否持续递增。

### 步骤 2：黑白表面识别

手动将传感器分别对准黑色和白色表面：

```bash
line                # 对准白色表面
line                # 对准黑色胶带（CH4 位置）
```

**观察 `st` 行**：对准黑色表面时，对应通道的 st 变为 1；对准白色表面时恢复 0。

**观察 `an` 行**：黑色表面模拟量 < 500，白色表面 > 500。

记录：

| 表面 | CH1 an | CH2 an | CH3 an | CH4 an | CH5 an | CH6 an | CH7 an | CH8 an |
|------|--------|--------|--------|--------|--------|--------|--------|--------|
| 白色 |        |        |        |        |        |        |        |        |
| 黑色 |        |        |        |        |        |        |        |        |
| 差值 |        |        |        |        |        |        |        |        |

### 步骤 3：架空巡线模拟

底盘**架空**，手动在传感器下方移动黑色胶带：

```bash
line on             # 启用巡线，期望 "line tracking enabled"
s                   # 确认 source=5 (LINE)
line                # 观察 pos/err/det
```

**四项场景验证**：

| 场景 | 黑线位置 | 预期 position | 预期 angular_z | 底盘行为 |
|------|----------|---------------|----------------|----------|
| 居中 | CH4 下方 | 3.5~4.0 | ~0 | 直行或微小转向 |
| 偏左 | CH1 下方 | ~1.0 | 负值 | 右侧车轮快于左侧（右转） |
| 偏右 | CH7 下方 | ~6.0 | 正值 | 左侧车轮快于右侧（左转） |
| 丢线 | 无黑线 | — | — | 底盘停止，source 回退 |

```bash
line off            # 关闭巡线
```

### 步骤 4：巡线参数调优

落地前先调整 P 控制参数。修改 `chassis_config.h`：

```c
#define LINE_KP             2.5f    // 增大 → 转弯更猛，减小 → 跟踪更平滑
#define LINE_SPEED_MPS      0.15f   // 增大 → 更快，但可能冲出赛道
#define LINE_ANGULAR_MAX_RPS 2.0f   // 角速度上限（防止急转弯翻车）
```

| 实验 | LINE_KP | LINE_SPEED_MPS | 预期 |
|------|---------|---------------|------|
| A | 1.5 | 0.10 | 转弯温和，适合大半径弯道 |
| B | 2.5 | 0.15 | 默认参数 |
| C | 4.0 | 0.15 | 转弯快速，可能震荡 |
| D | 2.5 | 0.25 | 高速巡线，需大 Kp 配合 |

每次修改后重编译烧录。

### 步骤 5：实际赛道巡线

> **安全警告**：落地测试前确保 `line off` 和 `stop` 命令熟练，底盘周围无障碍。

准备测试赛道（浅色地面 + 黑色胶带，胶带宽约 15mm），包含：
- 直线段（验证直行稳定性）
- 大半径弯道（验证渐进转弯）
- 急弯（验证最大角速度限制）

```bash
line on             # 启用巡线
# 将底盘放在黑线起始位置，黑线位于传感器中央
# 底盘自动开始巡线
# 观察底盘沿黑线行驶
line off            # 需要停止时关闭巡线
```

### 步骤 6：丢线恢复测试

在赛道中制作一段黑线中断（约 5cm 无胶带）：

**预期**：
1. 底盘经过中断处 → 传感器全部检测到白色
2. `detected_count = 0` → `ClearSource(LINE)`
3. 底盘停止（无其他控制源时）
4. PH/EN 模式下底盘进入低侧慢衰减制动；若需继续巡线，重新对准黑线后执行 `line on`

## 数据分析

### 线位跟踪误差分析

```python
import pandas as pd
import matplotlib.pyplot as plt

# 从 status 或 log 中提取巡线数据
df = pd.read_csv('lab06_line.csv')
t = df['t_ms'] / 1000.0

fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)

axes[0].plot(t, df['line_position'], 'b-', alpha=0.7)
axes[0].axhline(3.5, color='gray', linestyle='--', label='center')
axes[0].set_ylabel('Line Position (0-7)')
axes[0].legend()
axes[0].grid(alpha=0.3)

axes[1].plot(t, df['line_error'], 'r-', alpha=0.7)
axes[1].axhline(0, color='gray', linestyle='--')
axes[1].set_ylabel('Error')
axes[1].grid(alpha=0.3)

axes[2].plot(t, df['angular_z'], 'g-', alpha=0.7)
axes[2].set_ylabel('Angular Z (rad/s)')
axes[2].set_xlabel('Time (s)')
axes[2].grid(alpha=0.3)

plt.suptitle('Lab 06: Line Following Tracking Performance')
plt.tight_layout()
plt.savefig('lab06_line_tracking.png', dpi=150)
```

## 思考题

1. 加权平均计算线位时，为什么用 `position = Σ(i × state[i]) / Σ(state[i])` 而不是取 state=1 的最大/最小索引？加权平均的优势是什么？

2. 若黑线恰好位于 CH3 和 CH4 的中间，两个传感器可能都检测到黑线。此时 position ≈ 3.5，error ≈ 0。但如果阈值设置不当（偏高），会发生什么？

3. P 控制存在一个根本矛盾：`LINE_SPEED_MPS` 越高，同样 Kp 下的纠偏能力越弱（因为纠偏角速度对线速度的相对影响变小）。从运动学的角度解释这个现象，并说明为什么增大 `LINE_KP` 不能完全补偿高速的影响。

4. 丢线后底盘应如何处理？当前策略是清源停止。也可以设计为"丢线后保持最后角速度一秒"或"丢线后减速直行搜索"。分析这两种策略的优缺点。

5. 如果赛道上有十字交叉口（两条黑线交叉），8 路传感器会检测到多个黑线区域。`detected_count` 会突然增大，position 会跳到中间。这可能造成什么控制问题？如何改进算法？

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `line` 命令 `rx_bytes=0` | UART4 接线错误或传感器未供电 | 检查 PC10/PC11 接线和传感器供电 |
| `proto_err` 持续增长 | 传感器通信噪声或波特率不匹配 | 检查接线质量；确认传感器波特率为 115200 |
| 巡线时底盘剧烈摇摆 | Kp 过大或速度过快 | 减半 `LINE_KP`，降速 `LINE_SPEED_MPS` |
| 弯道冲出赛道 | Kp 过小或角速度上限太低 | 增大 `LINE_KP` 和 `LINE_ANGULAR_MAX_RPS` |
| 丢线过于频繁 | 阈值不当或黑线反光不足 | 微调 `LINE_ANALOG_THRESHOLD`；增加黑线宽度 |
