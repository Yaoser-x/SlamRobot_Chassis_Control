# Lab 12: OLED 显示与 I2C 自检

## 实验目标

1. **知识**：理解 I2C 总线通信协议（起始/停止/ACK/7-bit 地址）、SSD1306 OLED 控制器驱动原理、嵌入式开机自检（POST）设计模式
2. **技能**：能通过 `i2cscan` 扫描总线器件，解读 OLED 三阶段 UI（欢迎→自检→运行），理解自检项的 PASS/FAIL 含义
3. **素养**：理解"上电即反馈"的人机交互设计原则——启动过程中通过显示提供状态透明度

## 实验原理

### 1. I2C 总线基础

I2C1 总线（PB8 SCL / PB9 SDA）以 100 kHz 标准模式运行：

```
     SCL ──┐     ┌──┐  ┌──┐     ┌──
           └─────┘  └──┘  └─────┘
     SDA ────┐       ┌───────────
             └───────┘
           ST  Addr  ACK  Data  ACK  SP
```

每个从器件有 7-bit 地址。SSD1306 OLED 默认地址为 `0x3C`（7-bit），HAL 左移一位变成 `0x78`。

### 2. SSD1306 OLED 驱动

| 参数 | 值 |
|------|-----|
| 分辨率 | 128 × 64 像素 |
| 接口 | I2C1 (PB8/PB9) |
| 地址 | 0x3C (7-bit) |
| 刷新周期 | 100ms (oledTask, Low 优先级) |
| 驱动实现 | `BSP/oled/ssd1306.c`（页寻址模式，8 页 × 128 列） |

显存组织（页寻址，Page Addressing Mode）：

```
Column 0 ─────────────────────→ 127
Page 0: [bit0...bit7] × 128    ← 顶部 8 行
Page 1: [bit0...bit7] × 128
...
Page 7: [bit0...bit7] × 128    ← 底部 8 行
```

### 3. 三阶段 UI 流程

```
上电 ──→ Phase 0: WELCOME (5s)
        │ 显示: "F407 V2.0" + 固件版本
        │
        └──→ Phase 1: SELFCHECK (~4.8s, 8项 × 600ms)
             每项依次显示:
             [1] I2C Bus      → PASS/FAIL
             [2] IMU Probe    → PASS/FAIL
             [3] ADC Calib    → PASS/FAIL
             [4] Motor DRV    → PASS/FAIL
             [5] Encoder      → PASS/FAIL
             [6] UART3 (RPI)  → PASS/FAIL
             [7] UART4 (LINE) → PASS/FAIL
             [8] ESP12F       → PASS/FAIL
             │
             └──→ Phase 2: NORMAL (循环刷新)
                  显示:
                  - 运行时间 (s)
                  - 电池电压 (V)
                  - 8 个模块在线状态 (●/○)
                  - 当前控制源
```

### 4. 模块在线状态

正常运行时 OLED 用 ●/○ 指示 8 个模块状态：

| 模块 | 判定依据 | 超时 |
|------|---------|------|
| IMU | `imu_state.online` | — |
| ADC | `adc_state.current_valid` | — |
| Motor | `!motor_state.fault_active[i]` | — |
| Encoder | `encoder_state.speed_valid_all` | — |
| RPI | 上位机帧时间戳 | 500ms |
| LINE | 巡线帧时间戳 | 50ms |
| ESP | ESP12F 帧时间戳 | — |
| PS2 | `ps2_state.online` | — |

### 5. 自检阶段实现

自检阶段（`OLED_PHASE_SELFCHECK`）每 600ms 检测一项，8 项共约 4.8 秒：

```c
// OLED_SC_ERROR_* bits 9-16 (与 motor/battery bits 0-8 无冲突)
OLED_SC_ERROR_I2C         (1UL << 9)
OLED_SC_ERROR_IMU         (1UL << 10)
OLED_SC_ERROR_ADC         (1UL << 11)
OLED_SC_ERROR_MOTOR       (1UL << 12)
OLED_SC_ERROR_ENCODER     (1UL << 13)
OLED_SC_ERROR_UART3_RPI   (1UL << 14)
OLED_SC_ERROR_UART4_LINE  (1UL << 15)
OLED_SC_ERROR_ESP12F      (1UL << 16)
```

## 实验设备

| 类别 | 项目 |
|------|------|
| 硬件 | F407_V2.0 底盘（包含 SSD1306 OLED 模块）、12V 电池、USB-TTL 串口模块 |
| 软件 | 串口终端 |

## 实验步骤

### 步骤 1：I2C 总线扫描

上电后立即执行 I2C 扫描：

```bash
i2cscan
```

输出示例：
```
I2C1 scan:
  0x3C (7-bit)  ACK
```

**解读**：总线上在地址 0x3C 处检测到 ACK 响应，即 SSD1306 OLED。如果无设备响应：
```
I2C1 scan:
  no device found
```
检查 PB8 (SCL) / PB9 (SDA) 接线和 OLED 供电。

### 步骤 2：观察三阶段 UI 过渡

上电后立即注视 OLED 屏幕（不要操作串口），观察三个阶段：

| 时间 | 阶段 | 屏幕显示 |
|------|------|----------|
| 0–5s | WELCOME | "F407 V2.0" 大标题 |
| 5–10s | SELFCHECK | 8 项依次检测，每项约 0.6s |
| 10s+ | NORMAL | 运行时间、电压、模块状态 ●/○ |

记录自检阶段每项的 PASS/FAIL 结果：

| 序号 | 检测项 | 结果 | 若 FAIL，可能原因 |
|------|--------|------|------------------|
| 1 | I2C Bus | | |
| 2 | IMU Probe | | |
| 3 | ADC Calib | | |
| 4 | Motor DRV | | |
| 5 | Encoder | | |
| 6 | UART3 (RPI) | | |
| 7 | UART4 (LINE) | | |
| 8 | ESP12F | | |

正常运行时所有 8 项均应为 **PASS**（硬件完好的情况下）。

### 步骤 3：NORMAL 阶段信息解读

运行阶段 OLED 显示的信息：

```
运行时间: 123s
电池电压: 11.70V
模块状态:
● IMU  ● ADC  ● Motor  ● Encoder
○ RPI  ● LINE ○ ESP   ● PS2
控制源: DEBUG
```

- **●** = 在线/正常
- **○** = 离线/超时

对比 `status` 命令输出验证 OLED 上的模块状态是否正确。

### 步骤 4：模块离线检测

人为制造模块离线场景，观察 OLED 状态变化：

**场景 A — 巡线传感器断开**：
```bash
line                # 确认传感器在线
# 物理断开 UART4 连接线
# 等待 50ms（LINE 超时）
# OLED 上 LINE 状态: ● → ○
# 重新连接后 50ms 内: ○ → ●
```

**场景 B — IMU 禁用**：
```bash
imu 0               # 禁用 IMU
# OLED 上 IMU 状态: ● → ○
imu 1               # 重新启用
# OLED 上 IMU 状态: ○ → ●
```

### 步骤 5：自检错误与 DEBUG 台对比

若某项自检 FAIL，可通过串口进一步诊断：

| 自检 FAIL | 串口诊断命令 |
|-----------|------------|
| I2C | `i2cscan` |
| IMU | `imutest` + `imudiag` |
| ADC | `s` 检查 `valid=1` 和电流读数 |
| Motor DRV | `s` 检查 `drv_fault` |
| Encoder | `s` 检查 `speed_valid` |
| UART3/4 | `s` 检查对应 `rx_bytes` |
| ESP12F | `espflash status` |

## 数据分析

### OLED 刷新率测量

oledTask 周期为 100ms（10 Hz 刷新率）。通过录屏或高速摄像，验证实际刷新率：

```python
# 分析录屏中 OLED 屏幕内容变化的时间间隔
import numpy as np

frame_intervals = np.diff(timestamps_of_change)
print(f'OLED refresh interval: {np.mean(frame_intervals):.1f} ms '
      f'(expected: 100 ms)')
```

### 模块在线率统计

长时间运行（> 1 小时）记录各模块的在线率：

```python
# module_name: total_online_ms / total_observed_ms
df = pd.read_csv('lab12_module_uptime.csv')
for module in ['imu', 'adc', 'motor', 'encoder', 'rpi', 'line', 'esp', 'ps2']:
    uptime = (df[f'{module}_online'] == 1).sum() / len(df) * 100
    print(f'{module}: {uptime:.1f}% online')
```

## 思考题

1. SSD1306 页寻址模式下，一页（Page）对应 8 行像素。为什么选择 8 行为一页，而不是逐像素寻址？从 I2C 带宽和显存利用率的角度分析。

2. 自检阶段每项检测 600ms，8 项共约 4.8 秒。如果要求自检在 2 秒内完成，可以从哪些方面压缩时间？（提示：减少检查项、并行检查、减少间隔）

3. OLED 的 I2C 地址固定在 0x3C。如果总线上需要挂载第二个 OLED 或另一个 I2C 器件，SSD1306 的地址可以改变吗？查阅 SSD1306 数据手册确认。

4. 自检的错误码使用 bits 9-16 以区别于 motor/battery bits 0-8。为什么要这样设计？如果所有错误混在一起（bits 0-8），会有什么问题？

5. 当前的 OLED UI 只显示 8 个模块的 binary 状态（●/○）。如果要显示更多信息（如具体电流值、IMU 倾角），在 128×64 像素的限制下可以怎样优化布局？

## 常见问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| OLED 全黑不显示 | I2C 连接或供电异常 | `i2cscan` 确认 0x3C 有 ACK；检查 PB8/PB9 |
| 自检某项持续 FAIL | 对应硬件故障或未连接 | 用对应串口命令诊断（见表） |
| NORMAL 阶段模块全 ○ | 传感器超时或未初始化 | `status` 检查各传感器实际状态 |
| OLED 显示花屏/残影 | I2C 通信干扰或 SSD1306 显存未清 | 重新上电复位；检查 I2C 线缆屏蔽 |
