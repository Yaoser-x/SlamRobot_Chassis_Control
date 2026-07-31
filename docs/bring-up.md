# Bring-up 验收指南

按以下 12 个步骤逐步验收，每步确认通过后再进入下一步。**不要在未确认底层硬件的情况下直接跑闭环高速**。

---

## 步骤 1：RTOS 状态检查

**命令**：`rtos`

**确认项**：
- 十个业务任务均已创建（`safetyTask`–`oledTask`，无 `missing`）
- `heap_free` > 10KB，`heap_min` 不异常小
- 各任务 `stack_free` > 256B（尤其 `debugTask` 需 > 1KB，`motorTask` > 512B）
- 核心任务（`safety`/`motor`/`rpi`/`imu`/`oled`）`missed=0`

---

## 步骤 1.5：IWDG 看门狗验证

**无需命令**，通过以下方式间接验证：

- `rtos` 输出中任务持续运行。IWDG 名义门限约 800ms；复位后用 `resettrace`/复位原因确认，不以 OLED 现象单独下结论
- 喂狗同时要求 Motor 20ms、Safety 40ms 内各有新的完整周期 completion；任务入口 heartbeat 不等于完成
- Safety permit 超过 40ms 时 Motor 应先归零 PWM，随后才因 completion 不推进而等待 IWDG 复位
- 验证方式：长时间运行系统（> 10 分钟），确认无异常复位（`rtos` 的 runtime 计数不归零）

---

## 步骤 2：全系统状态快照

**命令**：`status` / `s`

默认两驱配置仅启用 M2（左侧）和 M3（右侧），M1/M4 的速度、电流和故障状态会按禁用策略归零。V2.0 实板 M2/M3 的最终映射为：

| 逻辑电机 | EN/IN1 PWM + PH/IN2 DIR | nFAULT | 编码器 | IPROPI |
| --- | --- | --- | --- | --- |
| M2 | PE11 / PC7 | PD14 | TIM4 PD12/PD13 | PC1 |
| M3 | PE13 / PC8 | PA3 | TIM3 PB4/PB5 | PC2 |

> CubeMX 生成文件保留旧 M2/M3 label；验收时以 BSP 逻辑映射和 `status` 的 M1~M4 顺序为准。

**确认项**：

- **编码器**：已启用电机的 `count` 和 `delta` 正常显示（架空轮时可为零）；`speed_valid` 状态正常。`hw=` 依次为逻辑 M1~M4 的原始定时器计数
- **底盘**：`req`/`target`/`actual` 均为零（无运动命令），`out=0`，`estop=0 fault=0`
- **Break**：正常空闲时 `tim1/tim8 moe=1`；`bif` 通常为 0，`count` 不应持续增长。若 `moe=0` 或计数增长，检查 PE15/PA6 和对应 DRV nFAULT/BKIN 路径
- **ADC**：`vbat` 显示电池电压（典型 11–12.6V）；四路电流接近零（架空轮无负载时）
- **IMU**（若已初始化）：`enabled`/`online` 状态正确；`chip=0x24`；加速度/陀螺/欧拉角有合理数值
- **系统**：`source=0`，`errors=0x00000000`，`drv_fault=0,0,0,0`
- **通信**：`line` 帧计数、`esp` 收发帧、`ps2` 连接状态均有显示
- **OLED**：屏幕亮起，依次显示欢迎屏（5s）→ 自检进度（8 项）→ 运行状态（时间/电压/模块在线/控制源）。自检阶段每项显示 PASS/FAIL，正常运行时 8 个模块在线状态用 ●/○ 指示

---

## 步骤 3：单轮电机方向验收

**前置条件**：架空轮或移除负载，确保旋转件不会造成伤害。

**命令**：
```
m1 200 0     # M1 以 200‰ 占空比正转（前进方向）
m1 0 0       # 停止 M1
m1 0 200     # M1 以 200‰ 占空比反转（后退方向）
m1 0 0
```

依次测试需要启用的电机；默认两驱重点测试 M2、M3。**确认项**：
- 每路电机正转/反转方向与预期一致
- 若某路方向相反，修改 `CHASSIS_Mx_MOTOR_DIR = -1`
- 若某路不转，检查 DRV8874 `nSLEEP`/`nFAULT` 和接线
- `status` 中对应 `drv_fault` 保持 0

---

## 步骤 4：左右侧聚合测试

**命令**：
```
left 200      # 左侧两轮（M1+M2）同时正转
left 0
right 200     # 右侧两轮（M3+M4）同时正转
right 0
motor 200 200 # 四轮同时正转（前进）
motor 0 0
motor 200 -200 # 原地左转（左侧正转、右侧反转）
motor 0 0
```

**确认项**：`CHASSIS_Mx_SIDE` 侧别配置与实际轮组对应，聚合命令下所有已启用目标电机同步动作。默认两驱下 `left 200` 仅驱动 M2（左侧），`right 200` 仅驱动 M3（右侧）。

---

## 步骤 5：编码器方向与速度验证

**前置条件**：车轮落地或架空。

**命令**：`motor 200 200`（低速前进），然后 `status`

**确认项**：
- 已启用电机的编码器 `speed_mm/s` 均为正值（前进时）
- 若某路为负值，修改 `CHASSIS_Mx_ENCODER_DIR = -1`
- `count` 值随轮转动递增
- 停止后 `speed_mm/s` 回零（在 `CHASSIS_MAX_ENCODER_DT_MS` 内）

---

## 步骤 6：ADC 校准

**命令**：`status`

ADC1 由 TIM8 TRGO 以 2kHz 触发五通道 DMA 扫描（20kHz PWM / 10），safetyTask 每 20ms 读取最新快照并更新滤波、零点和保护状态。零点累计 256 次约需 2.56 秒，完成前电流保护尚未生效，禁止执行运动测试。

**确认项**：
- `vbat` 与万用表测量值一致，偏差 < 0.2V；`raw` 为 VBAT ADC 原始值
- `cal=256/256 valid=1` 后，四路电流在静止时接近零（±50mA）
- `ADCCAL cvalid=1` 且 `ADCQ span` 未超过门限；否则不要启用任何电流保护/限流配置
- 若电池电压偏差明显，调整 `POWER_ADC_DRIVER_BATTERY_R_UPPER_OHM` / `_R_LOWER_OHM`
- 若电流零点偏移，先确认上电校准期间电机未转动，再在全部电机停止后执行 `adccal zero`，查看各路 `z=` raw 零点和 `span`
- 三阶段验收顺序固定为：静止零点 (`status`/`adccal show`) → 空载低 PWM (`log 1 adc adcraw`) → 带载或短时堵转；默认 dry-run 只报警计数，不应改变 PWM

---

## 步骤 7：闭环低速验证

**前置条件**：步骤 3–6 全部确认通过，ESTOP 未激活。

**命令**：
```
vel 50        # 以 50mm/s 直线前进
vel 0         # 停止
vel 50 100    # 前进 50mm/s + 角速度 100mrad/s
vel 0
```

**确认项**：
- 底盘实际运动与指令一致（直线/转弯方向正确）
- `status` 中 `actual_mm/s` 跟踪 `target_mm/s`（稳态偏差 < 20%）
- `missed` 不增长
- `fault` 保持 0
- 不宜一开始就跑高速（>300mm/s），先确认低速闭环稳定性

---

## 步骤 8：BMI270 IMU 验收

**命令**：
```
imutest       # 探测芯片
imudiag       # SPI 硬件诊断
imuinit       # 加载配置表并初始化
imucal        # 静止校准陀螺零偏（保持静止 1–2s）
status        # 查看 IMU 数据
```

**确认项**：
- `imutest` 输出 `"bmi270 probe ok"`
- `imuinit` 输出 `"bmi270 init ok"`
- `imudiag` 三路 HAL 读取和 bit-bang 读取均正常（无 `0x00`/`0xFF` 死值）
- `imucal` 输出 `"calibration ok"`，gyro bias 在 ±50mdps 以内（静止时）
- `status` 中 `roll/pitch/yaw` 有效，静止时 yaw 漂移 < 1°/s
- 手动旋转 IMU 时欧拉角变化符合直觉

---

## 步骤 9：ESP12F 烧录通道验证

**命令**：
```
espflash on
```

然后立即关闭串口终端，执行：
```bash
esptool.py --chip esp8266 --port COMx --baud 115200 chip_id
```

**确认项**：
- `chip_id` 返回非零值（例如 `0x00d4f2a8`）
- 若超时，检查 `ESP_IO0` 电平、USART2 接线、ESP12F 供电

---

## 步骤 10：ESP12F 网页控制验证

**前置条件**：步骤 9 通过，ESP12F 已烧录网页固件。

**确认项**：
- 复位整板后 ESP12F 正常启动（`espflash status` 显示 inactive）
- 擦除 EEPROM 后仅出现开放 `F407_Chassis_Setup`；7/64 字节密码拒绝，8/63 字节接受并重启。重启后出现受密码保护的 `F407_Chassis`。
- 配置完成前 WebSocket 不启动，无法发送运动。
- 两个网页客户端中首个 claimant 为 owner，另一个只读但仍收到遥测。非 owner 运动/巡线/普通停止被拒绝。
- owner 停 heartbeat 或断开后 500ms 内停车并释放控制权。任意客户端可设置 ESTOP，但网页/固件不能解除，须本地 `estop 0`。
- 通过 owner 网页发送运动命令，`status` 中 `source` 显示 `ESP12F(3)`。

---

## 步骤 11：巡线传感器验收

**前置条件**：UART4 巡线传感器已正确接线（PC10→传感器 RX，PC11→传感器 TX）。

**命令**：
```
line          # 查看传感器状态
log 1 line    # 可选：持续观察收帧统计
```

**阶段 A — 通信验证**：
- 连续多次输入 `line`，`rx_bytes` 每周期增长约 21 字节（200Hz 查询 × 21 字节 = 4.2KB/s）
- `frames` 逐秒递增（每秒约 200 帧）
- `proto_err` 和 `ovf` 保持 0 或极低值（偶尔噪声导致可接受）

**阶段 B — 数据验证**：
- 将传感器依次对准黑色和白色表面
- `st` 状态位相应变化（黑线上对应通道为 1，白色为 0）
- `an` 模拟量在黑线上 < `LINE_ANALOG_THRESHOLD`（默认 500），白色上 > 阈值

**阶段 C — 控制验证**：
- 架空轮，输入 `line on` 启用巡线
- 手动在传感器下方移动黑线
- `status` 中 `source` 显示 `LINE(5)`，底盘随黑线位置转向
- 黑线居中时底盘直行，黑线偏左时左转，偏右时右转
- 移除黑线（丢线），底盘停止（`ClearSource`），`source` 回退为 `NONE` 或更低优先级源
- `line off` 或 PS2 三角键关闭巡线

**阶段 D — 实际巡线**：
- 将底盘放在铺有黑线的浅色地面上
- `line on` 启用巡线，默认速度 `LINE_SPEED_MPS = 0.15 m/s`
- 底盘应沿黑线前进，转弯平滑
- 若转弯过猛，减小 `LINE_KP`；若跟踪慢/脱线，增大 `LINE_KP`
- 若前进过快，减小 `LINE_SPEED_MPS`
