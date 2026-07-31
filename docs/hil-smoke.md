# HIL 冒烟测试

rc2 软件状态为 `HIL_PENDING`。创建 `v1.0.0-rc2` annotated tag 前必须归档冷/热启动各 20 次、
五来源 rearm、PS2 接管/断联、断链停车、故障清除不恢复旧命令、编码器尖峰/冻结、UART 压力、
Motor/Safety task stall、Safety permit 先停车后 IWDG 复位，以及 IMU OADEV/温漂/六面数据报告。
HIL 未完成不阻止本地软件 squash，但禁止创建 rc2 标签。

HIL v1 只验证启动和只读诊断链路，不自动驱动电机，不切换继电器，不修改 Flash 参数。

## 前置条件

- 已烧录最新 STM32 主控固件。
- PC 能访问 USART1 调试串口。
- 车体架空或电机电源断开，避免误动作。

## 命令

```bash
python scripts/hil_smoke.py --port COMx --json-out hil-smoke.json --transcript-out hil-smoke.txt
```

可选参数：

```bash
python scripts/hil_smoke.py --port COMx --baud 115200 --timeout 20 --json-out hil-smoke.json
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
- `version`
- `i2cscan`
- `imutest`
- `espflash status`

通过条件：

- 串口能打开。
- `status` 有 `POST done=` 和 `PARAM` 行；连接到已经运行的固件时不要求重新出现启动横幅。
- 每条命令都有响应，脚本未超时。
- JSON 中保存 ISO 8601 UTC 时间、设备/端口、固件身份、命令摘要、逐项断言和最终 pass/fail；文本 transcript 保留供人工审计。

原始 CSV 可用 `scripts/analyze_roadmap_data.py` 的 `current|encoder|line|geometry` 子命令分析；
IMU 姿态稳定性与 gyro OADEV 使用 `scripts/analyze_imu.py`。分析器不会自动回写阈值，也不会把软件完成标成 HIL 通过。

```powershell
python scripts/analyze_imu.py imu.csv --output-dir allan-out `
  --firmware-sha <final-squash-sha> --parameter-crc <effective-crc> `
  --imu-odr 100Hz --imu-bandwidth-profile normal
```

必须归档 `allan_result.json`、`allan_curve.csv` 和 `summary.md`。默认输入是
`imu_gyro_corr_*_dps`，质量缺口不得拼接；具体算法、单位与 BI plateau 规则见
[RC2 陀螺噪声分析方法](release/gyro-allan-method.md)。

双向直行使用监督式分析器（脚本不连接车辆）：

```bash
python scripts/analyze_straight_hil.py straight.jsonl measurements.csv \
  --baseline-csv baseline.csv --json straight-report.json --markdown straight-report.md
```

人工 CSV 列固定为 `run_id,direction,speed_mps,caster_transition,distance_m,lateral_error_m,yaw_error_deg,firmware_sha,battery_v`。换向后前 0.30m 必须标记 `caster_transition=1` 并单独保留；随后的 2m 稳态段标记为 0。单个 telemetry 文件对应唯一 run 时，脚本可从人工 CSV 推断 `run_id`；合并多个 run 前必须由采集流程为每行注入 `run_id`。脚本只接受 `forward/reverse` 与 `0.15/0.30m/s`，次数按唯一 `run_id` 统计。它只输出 RAM `set` 建议，不包含 `set save`；四格各 5 次、每行固件 SHA 非空且与 baseline 唯一一致，并同时满足横向、航向和相对基线改善率后才允许人工保存。

## 边界

- 该脚本不证明电机闭环、巡线、PS2、ESP 网页控制都已通过实车验收。
- `imutest` 失败时脚本会失败；若本次硬件没有安装 BMI270，应跳过 HIL 或调整验收策略。
- `espflash status` 只读取桥状态，不进入烧录模式。

电流链、DRV8874 硬件限流和 ADC 采样相位不属于本只读 smoke 的覆盖范围，
按 [电流链与 ADC 采样相位 HIL 验收](current-chain-hil.md) 单独留存示波器证据。

## P0 专项 HIL（需人工执行）

任务 stall 不使用 Release 运行时命令。Host Test 使用 fake scheduler；实板使用调试器/J-Link
挂起指定 RTOS 任务，或使用不进入发布产物的 HIL-only 构建目标。每次记录：Release/HIL 构建类型、
宏、注入方法和时刻、最后喂狗时刻、PWM 归零时刻、IWDG 复位时刻、复位原因和电机输出。

- 挂起 Safety task：要求 permit 超过 40 ms 后 PWM 先归零，随后 IWDG 才复位。
- 分别挂起 Motor 和 Safety task：确认任一 completion 不前进都停止喂狗，旧 generation 不能复用。
- 启动后首次合法喂狗必须在 100 ms 内；实测仍须满足 `Motor timeout < Safety timeout < IWDG timeout`。

- GPIO 包围 Encoder 最终发布临界区，确认 <20µs。
- raw/open-loop 单次发送后断开 USART1，500ms 内 PWM 归零；编码器断开后 200ms 内整车停车并需 `clearfault`。
- 上位机持续发命令时执行 `set save`，Flash 写入全程 PWM 必须为 0。
- PS2 L1/R1 ±90° 误差≤5°，L2/R2 ±360° 误差≤10°；人工介入、手柄失联、IMU 质量异常和安全故障立即取消并零速。
- 验证 10.5/11.0V 告警滞回、9.0V-500ms 锁停和 9.6V-2s 自动解除；解除后 source 仍为 NONE。
- 同一 nFAULT 注入应同时留下 TIM1/TIM8 count/last；TIM1 持续锁存至安全 `clearfault`。
- 冷启动和热复位各 20 次，正常板不得出现 `0x240`；异常时记录 `status` 的 BREAK `origin/startup/pre_bif/bkin/nfault`。启动阶段允许短暂唤醒瞬态，但必须连续稳定 5ms 才武装输出，20ms 未稳定则锁存。
- ESP 擦 EEPROM 首配、8/63 字节边界、两客户端 owner/只读、heartbeat/断连停车和远程 ESTOP 只置位。Arduino IDE 记录 Core 3.1.2 与 WebSockets 2.7.2 编译结果。
- ESP owner 可申请清除普通 fault，观察者不可；故障条件仍存在时主控拒绝，任意远程客户端均不能解除 ESTOP。
