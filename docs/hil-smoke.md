# HIL 冒烟测试

HIL v1 只验证启动和只读诊断链路，不自动驱动电机，不切换继电器，不修改 Flash 参数。

## 前置条件

- 已烧录最新 STM32 主控固件。
- PC 能访问 USART1 调试串口。
- 车体架空或电机电源断开，避免误动作。

## 命令

```bash
python scripts/hil_smoke.py --port COMx
```

可选参数：

```bash
python scripts/hil_smoke.py --port COMx --baud 115200 --timeout 20
```

在底盘完全静止且电机电源安全的条件下，可单独验证 500 样本 IMU 标定期间
低优先级任务没有新增 timeout：

```bash
python scripts/hil_imu_calibration.py --port COMx --baud 115200
```

该脚本比较标定前后 line、ESP、debug、LED、OLED 的 RTOS timeout 计数；它会
启动一次运行时标定，因此不属于默认只读 smoke。

## 验收项

脚本会等待启动输出并依次发送：

- `status`
- `i2cscan`
- `imutest`
- `espflash status`

通过条件：

- 串口能打开。
- 启动输出或命令响应中能看到 `POST:`。
- `status` 有 `POST` 和 `PARAM` 行。
- 每条命令都有响应，脚本未超时。

## 边界

- 该脚本不证明电机闭环、巡线、PS2、ESP 网页控制都已通过实车验收。
- `imutest` 失败时脚本会失败；若本次硬件没有安装 BMI270，应跳过 HIL 或调整验收策略。
- `espflash status` 只读取桥状态，不进入烧录模式。

电流链、DRV8874 硬件限流和 ADC 采样相位不属于本只读 smoke 的覆盖范围，
按 [电流链与 ADC 采样相位 HIL 验收](current-chain-hil.md) 单独留存示波器证据。

## P0 专项 HIL（需人工执行）

- GPIO 包围 Encoder 最终发布临界区，确认 <20µs。
- raw/open-loop 单次发送后断开 USART1，500ms 内 PWM 归零；编码器断开后 200ms 内整车停车并需 `clearfault`。
- 上位机持续发命令时执行 `set save`，Flash 写入全程 PWM 必须为 0。
- PS2 L1/R1 ±90° 误差≤5°，L2/R2 ±360° 误差≤10°；人工介入、手柄失联、IMU 质量异常和安全故障立即取消并零速。
- 验证 10.5/11.0V 告警滞回、9.0V-500ms 锁停和 9.6V-2s 自动解除；解除后 source 仍为 NONE。
- 同一 nFAULT 注入应同时留下 TIM1/TIM8 count/last；TIM1 持续锁存至安全 `clearfault`。
- 冷启动和热复位各 20 次，正常板不得出现 `0x240`；异常时记录 `status` 的 BREAK `origin/startup/pre_bif/bkin/nfault`。启动阶段允许短暂唤醒瞬态，但必须连续稳定 5ms 才武装输出，20ms 未稳定则锁存。
- ESP 擦 EEPROM 首配、8/63 字节边界、两客户端 owner/只读、heartbeat/断连停车和远程 ESTOP 只置位。Arduino IDE 记录 Core 3.1.2 与 WebSockets 2.7.2 编译结果。
- ESP owner 可申请清除普通 fault，观察者不可；故障条件仍存在时主控拒绝，任意远程客户端均不能解除 ESTOP。
