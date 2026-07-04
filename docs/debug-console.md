# USART1 调试命令台

## 1. 概述

USART1（PB6 TX / PB7 RX），`115200 8N1`。由 `debugTask`（osPriorityBelowNormal, 10ms 周期, 2048W 栈）驱动，通过中断式环形缓冲区接收命令行，`\r` 或 `\n` 为行结束符。

**安全约束**：ESP12F flash bridge active 期间调试台暂停命令解析（避免二进制烧录流被误当作文本命令）。

---

## 2. 全部命令

| 命令 | 参数 | 说明 |
| --- | --- | --- |
| `help` / `h` | — | 打印命令列表 |
| `status` / `s` | — | 打印所有子系统单次快照（编码器/底盘/ADC/IMU/系统/巡线/ResetTrace） |
| `rtos` | — | FreeRTOS heap、各任务栈余量、missed-period、通信统计 |
| `header` | — | 打印全字段 CSV 日志标题行 |
| **`log 0`** | — | 停止 CSV 日志输出 |
| **`log 1`** | `[field...]` | 启动 2 Hz CSV 日志，可选字段过滤（见第 3 节） |
| **`line`** | — | 打印巡线传感器原始数据与控制状态 |
| **`line on`** | — | 启用巡线控制 |
| **`line off`** | — | 禁用巡线控制 |
| `motor` | `<L> <R>` | 左右侧开环 permille（范围 -900…900） |
| `left` / `right` | `<P>` | 单侧开环快捷命令 |
| `m1`…`m4` | `<F> <R>` | 单路 raw EN/PH 测试；输出按 `F-R` 解析为有符号 PWM |
| `raw` | `<LF> <LR> <RF> <RR>` | 左右侧 raw EN/PH 输入；每侧按 `F-R` 解析 |
| `vel` | `<mm/s> [mrad/s]` | 闭环速度控制（提交至 `CONTROL_SOURCE_DEBUG`） |
| `stop` | — | 清除所有测试命令、清空 open-loop 和 `vel` 指令 |
| `estop` | `<0\|1>` | 清除/设置紧急停止 |
| `clearfault` | — | 清除锁存过流/DRV 故障标志 |
| `imutest` | — | 探测 BMI270（读取 CHIP_ID 期望 `0x24`） |
| `imudiag` | — | SPI 硬件诊断（寄存器回读 + bit-bang + MISO 上拉/下拉/悬空） |
| `imuinit` | — | 加载配置表并初始化 BMI270 |
| `imucal` | `[n]` | 陀螺零偏校准（默认自动采样数，可选指定） |
| `imucalclear` | — | 清除陀螺校准数据 |
| `imu` | `<0\|1>` | 启用/禁用 IMU 周期采样 |
| `espreset` | — | 复位 ESP12F（`ESP_RST` 脉冲） |
| `espboot` | `<0\|1>` | 设置 ESP12F 正常启动 / 下载模式（`ESP_IO0`） |
| `espflash on` | — | 启用 USART1↔USART2 透明烧录桥（下载模式：IO0=0 + 复位） |
| `espflash off` | — | 关闭烧录桥，ESP12F 正常启动 |
| `espflash status` | — | 查看桥接统计（收发字节/溢出/错误/自动退出次数） |
| **`espat on`** | — | 启用 USART1↔USART2 AT 透传桥（正常模式：IO0=1 + 复位） |
| **`espat off`** | — | 关闭 AT 透传桥，ESP12F 正常启动 |
| **`i2cscan`** | — | 扫描 I2C1 总线（地址 1–127），列出所有 ACK 响应的器件 |

---

## 3. 周期 CSV 日志 (`log`)

### 3.1 基本用法

日志以 500ms 间隔（2 Hz）向 USART1 输出 CSV 行。**全字段模式**（`log 1` 无参数）输出全部 36 列，兼容历史数据解析脚本。**过滤模式**仅输出指定字段，减少串口带宽占用：

```
log 0                          停止日志
log 1                          全字段（追加 IMU SensorTime/quaternion/quality）
log 1 imu                      仅时间戳 + IMU 20 列
log 1 motor imu                时间戳 + motor(8 列) + imu(20 列)，按输入顺序
log 1 motor adc line           时间戳 + motor + adc + line，按输入顺序
log 1 motor adc adcraw         电机速度/PWM + 稳定电流 + ADC 窗口统计
```

### 3.2 可用字段

| 字段名 | 输出列 | 列数 | 数据来源 |
| --- | --- | --- | --- |
| `motor` | `m1_mms, m2_mms, m3_mms, m4_mms, m1_pwm, m2_pwm, m3_pwm, m4_pwm` | 8 | 相邻日志帧累计计数计算的约 500ms 平均轮速 + `ChassisControl_GetState` |
| `adc` | `vbat_mv, m1_ma, m2_ma, m3_ma, m4_ma` | 5 | `AdcMonitor_GetState` 的慢速稳定电流，兼容旧 CSV |
| `adcraw` | `m*_mean_ma, m*_rms_ma, m*_pk_ma, m*_n` | 16 | 最近 ADC 窗口的均值/RMS/峰值/样本数，用于电流校准 |
| `imu` | `imu_online, imu_chip, imu_acc_x/y/z_mg, imu_gyro_corr_x/y/z_mdps, imu_gyro_filt_x/y/z_mdps, imu_roll/pitch/yaw_mdeg, imu_stime, imu_q*_milli, imu_quality` | 20 | `ImuBmi270_GetState`（车体坐标系） |
| `errors` | `errors` | 1 | `SystemMonitor_GetState` |
| `source` | `source` | 1 | `SystemMonitor_GetState` |
| `ps2` | `ps2_ok, ps2_fail` | 2 | `Ps2Control_GetState` |
| `line` | `line_bytes, line_frames` | 2 | `LineUart_GetState` |
| `esp` | `esp_rx, esp_tx` | 2 | `Esp12fComm_GetState` |

**输出格式**：第一列为 `t_ms`（`osKernelGetTickCount()`），后续按用户输入顺序排列字段列。所有浮点值缩放为毫/微单位整数（×1000），避免 `printf` 浮点开销。

**性能**：过滤模式下惰性获取状态快照——仅获取选中字段对应的子系统数据，其余不调用 `GetState`。日志行通过 `HAL_UART_Transmit` 同步输出（100ms 超时），USART TX 繁忙时阻塞等待。

---

## 4. 巡线调试命令 (`line`)

### 4.1 输出格式

```
> line
LINE enabled=0 active=0 pos=3.50 err=0.00 lx=0.000 az=0.000 det=3
LINE st=00110000 an=234,198,45,67,812,723,1023,998
LINE rx_bytes=14280 frames=680 proto_err=2 ovf=0
```

### 4.2 字段说明

**第一行 — 控制状态**：

| 字段 | 类型 | 说明 |
| --- | --- | --- |
| `enabled` | `0`/`1` | 全局巡线开关（`g_line_enabled`），通过 `line on`/`line off` 或 PS2 三角键或 ESP12F 控制 |
| `active` | `0`/`1` | 当前是否正在跟踪黑线（有控制指令输出） |
| `pos` | float | 加权平均线位置。0.0 = CH1（最左），7.0 = CH8（最右），中心 3.5 |
| `err` | float | 距中心偏差 = pos − 3.5。正值=线偏右→应右转，负值=线偏左→应左转 |
| `lx` | float | 当前输出的 `linear_x` (m/s)，正常为 `LINE_SPEED_MPS` |
| `az` | float | 当前输出的 `angular_z` (rad/s)，由 P 控制器计算 |
| `det` | uint | 当前检测到黑线的通道数（0–8）。0 表示丢线 |

**第二行 — 传感器原始数据**：

| 字段 | 格式 | 说明 |
| --- | --- | --- |
| `st` | 8 位 `0`/`1` | 8 通道状态位（`state[0]`–`state[7]`）。`1` = 检测到黑线（模拟量 < `LINE_ANALOG_THRESHOLD`） |
| `an` | 8 个 uint | 8 通道原始模拟量 ADC 值（0–4095） |

**第三行 — UART 收帧统计**：

| 字段 | 说明 |
| --- | --- |
| `rx_bytes` | UART4 DMA 累计接收字节数（含所有原始字节） |
| `frames` | 成功解析的有效协议帧数（`0x55 0xAA` 帧头 + 校验和通过） |
| `proto_err` | 协议错误数（校验和失败或非法帧长） |
| `ovf` | DMA 循环缓冲溢出次数 |

### 4.3 子命令

| 命令 | 说明 |
| --- | --- |
| `line` | 打印当前传感器数据与控制状态（单次快照） |
| `line on` | 启用巡线控制（设置 `g_line_enabled = 1`） |
| `line off` | 禁用巡线控制（设置 `g_line_enabled = 0`，并 `ClearSource(LINE)`） |

---

## 5. 其他命令速览

### 5.1 状态查询

- **`status` / `s`**：单次输出所有子系统快照，含编码器速度与计数、底盘目标/实际速度与 PWM、TIM1/TIM8 BREAK 的 MOE/BIF/累计观测次数、ADC 电压电流（含零点校准进度）、IMU profile/init/SensorTime/quaternion/quality、系统错误标志/复位标志/控制源、通信统计、ResetTrace 崩溃记录

`ENC` 行末的 `hw=a,b,c,d` 是按逻辑 M1~M4 排列的原始定时器计数，用于区分“定时器未收到脉冲”和“逻辑计数未更新”。V2.0 映射为 M1=TIM2、M2=TIM4、M3=TIM3、M4=TIM5；CubeMX 中 M2/M3 的旧 label 不代表运行时逻辑顺序。

`BREAK tim1 moe=... bif=... count=... tim8 ...` 中，`moe` 表示当前主输出是否启用，`bif` 表示本次采集是否观察到 Break 标志，`count` 是启动以来累计观察次数。BIF 在采集后清除，因此瞬态故障应结合 `count` 判断。
- **`rtos`**：FreeRTOS 运行时状态，除 heap/栈外还包括 USART3 TX drop 和 ESP12F TX drop 计数
- **`header`**：打印全字段 CSV 标题行（调试用，正常由 `log 1` 自动输出）

### 5.2 电机调试

- 所有 motor/raw 命令受 ESTOP 和 fault-stop 保护，激活时拒绝执行并提示 `"rejected: estop/fault active"`
- permille 参数自动钳位至 `±CHASSIS_PWM_MAX_PERMILLE`（默认 900‰）
- `vel` 通过 `ControlManager` 提交至 `CONTROL_SOURCE_DEBUG`（最低优先级），每 10ms 自动刷新时间戳避免超时

### 5.3 IMU 操作

- `imutest` 执行单次 SPI 读取 chip-id，不修改 IMU 运行状态
- `imucal` 要求保持静止，校准成功后自动写入零偏估计值，`imucalclear` 清除
- `imu 0` 不会关闭 SPI 外设，仅停止 `imuTask` 采样；正常采样由 BMI270 INT1 唤醒，10ms 超时后轮询降级

### 5.4 ESP12F 管理

- `espreset` 通过拉低 `ESP_RST` 复位 ESP12F
- `espisolate` 彻底断电 ESP12F（拉低 RST + EN），仅可通过整板复位恢复。用于确认 ESP12F 异常时不影响主控
- `espboot 1` 设置下载模式（`ESP_IO0=0` + 复位），`espboot 0` 恢复正常启动（`ESP_IO0=1`）
- `espflash on` 进入烧录桥（下载模式：IO0=0, RST 脉冲），用于 esptool.py 或 Arduino 烧录
- `espat on` 进入 AT 透传桥（正常模式：IO0=1, RST 脉冲），用于手动发 AT 指令测试连通性
- `espat on` / `espflash on` 操作均不可逆——需要等待 30s 自动退出或复位整板才能恢复调试台
- `espflash off` / `espat off` 用于桥未激活时手动恢复（或复位整板强制退出）

### 5.5 复位诊断（Reset Trace）

上电时自动打印复位源标志和崩溃追踪记录。`status` 命令也会输出这些信息。

**启动输出示例**：

```
RESET csr=0x0C000000 bor=0 por=0 pin=0 sftr=0 iwdg=0 wwdg=0 lpwr=0
RESETTRACE valid=1 kind=2 reason=0 task=2 line=0 cfsr=0x00000082 hfsr=0x40000000 bfar=0x00000000 mmfar=0x00000000 pc=0x0800ABCD lr=0x08001234 xpsr=0x61000000 exc=0xFFFFFFEC sp=0x2001FF00 msp=0x2001FF00 psp=0x2001F000 ctrl=0x00000002 fpccr=0x00000000 dma_lisr=0x00000000 dma_cr=0x00000000 dma_ndtr=5 dma_fcr=0x00000021 adc_sr=0x00000000 adc_cr2=0x16000303 d0=0 d1=0 d2=0 d3=0 safety=12345 motor=12340 ps2=12300 esp=12200 debug=12100 source=2 estop=0 fault=0
```

**复位源字段**（`RESET` 行）：

| 字段 | 说明 |
| --- | --- |
| `csr` | RCC->CSR 寄存器原始值 |
| `bor` | 欠压复位 (Brown-out Reset) |
| `por` | 上电复位 (Power-on Reset) |
| `pin` | NRST 引脚复位 |
| `sftr` | 软件复位 (Software Reset) |
| `iwdg` | 独立看门狗复位 |
| `wwdg` | 窗口看门狗复位 |
| `lpwr` | 低功耗复位 |

**ResetTrace 字段**（`RESETTRACE` 行）：

| 字段 | 说明 |
| --- | --- |
| `valid` | 记录是否有效（magic + checksum 通过） |
| `kind` | 崩溃类型：0=NONE, 1=NMI, 2=HardFault, 3=MemManage, 4=BusFault, 5=UsageFault, 6=Error_Handler, 7=FreeRTOS, 8=DMA_GUARD |
| `task` | 最后活跃任务：0=NONE, 1=Safety, 2=Motor, 3=PS2, 4=ESP, 5=Debug |
| `cfsr` | Configurable Fault Status Register（含 MMARVALID/BFARVALID/各种 fault 原因位） |
| `hfsr` | HardFault Status Register |
| `bfar` | Bus Fault Address Register |
| `mmfar` | MemManage Fault Address Register |
| `pc/lr/xpsr` | 崩溃时堆栈上的 PC/LR/XPSR（HardFault 专用） |
| `exc` | EXC_RETURN 值 |
| `sp` | 崩溃时的栈指针 |
| `msp/psp` | 崩溃时主栈和进程栈指针 |
| `ctrl/fpccr` | CONTROL 与 FPU 上下文控制寄存器 |
| `dma_lisr/dma_cr/dma_ndtr/dma_fcr` | DMA2 Stream0 状态和剩余传输计数 |
| `adc_sr/adc_cr2` | ADC1 状态和控制寄存器 |
| `safety/motor/ps2/esp/debug` | 各任务最后一次心跳 tick（ms） |
| `source/estop/fault` | 崩溃时的控制源/ESTOP/fault-stop 状态 |

**原理**：`reset_trace_record_t` 存放在 `.noinit` RAM 段（链接脚本 `STM32F407XX_FLASH.ld` 定义），启动代码不清零，因此记录可跨软件复位、看门狗复位和故障复位保留，但断电后不保证保留。首次读取时固件将记录复制为本次启动快照并清空 live 区，供下一次故障记录使用。

### 5.6 I2C 扫描 (`i2cscan`)

扫描 I2C1 总线地址 1–127，通过 `HAL_I2C_IsDeviceReady` 逐一探测，列出所有 ACK 响应的器件 7-bit 地址。

**输出示例**：

```
I2C1 scan:
  0x3C (7-bit)  ACK
```

- 7-bit 地址 `0x3C` 对应 SSD1306 OLED（HAL 左移 1 位后 `0x78`）
- 无器件响应时输出 `"no device found"`
- 用于验证 I2C 总线连接和器件地址，排查 OLED 不显示问题
