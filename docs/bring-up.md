# Bring-up 验收指南

按以下 11 个步骤逐步验收，每步确认通过后再进入下一步。**不要在未确认底层硬件的情况下直接跑闭环高速**。

---

## 步骤 1：RTOS 状态检查

**命令**：`rtos`

**确认项**：
- 所有 11 个任务均已创建（`defaultTask`–`oledTask`，无 `missing`）
- `heap_free` > 10KB，`heap_min` 不异常小
- 各任务 `stack_free` > 256B（尤其 `debugTask` 需 > 1KB，`motorTask` > 512B）
- 核心任务（`safety`/`motor`/`rpi`/`imu`/`oled`）`missed=0`

---

## 步骤 2：全系统状态快照

**命令**：`status` / `s`

**确认项**：

- **编码器**：四路 `count` 和 `delta` 正常显示（架空轮时可为零）；`speed_valid` 状态正常
- **底盘**：`req`/`target`/`actual` 均为零（无运动命令），`out=0`，`estop=0 fault=0`
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

依次测试 M1–M4。**确认项**：
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

**确认项**：`CHASSIS_Mx_SIDE` 侧别配置与实际轮组对应，聚合命令下所有目标电机同步动作。

---

## 步骤 5：编码器方向与速度验证

**前置条件**：车轮落地或架空。

**命令**：`motor 200 200`（低速前进），然后 `status`

**确认项**：
- 四路编码器 `speed_mm/s` 均为正值（前进时）
- 若某路为负值，修改 `CHASSIS_Mx_ENCODER_DIR = -1`
- `count` 值随轮转动递增
- 停止后 `speed_mm/s` 回零（在 `CHASSIS_MAX_ENCODER_DT_MS` 内）

---

## 步骤 6：ADC 校准

**命令**：`status`

**确认项**：
- `vbat` 与万用表测量值一致，偏差 < 0.2V；`raw` 为 VBAT ADC 原始值
- `cal=256/256 valid=1` 后，四路电流在静止时接近零（±50mA）
- 若电池电压偏差明显，调整 `ADC_MONITOR_BATTERY_R_UPPER_OHM` / `_R_LOWER_OHM`
- 若电流零点偏移，先确认上电校准期间电机未转动，再查看各路 `z=` raw 零点

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
- WiFi AP 出现并可连接
- 网页控制界面可正常访问
- 通过网页发送运动命令，`status` 中 `source` 显示为 `UPPER(1)` 或 `ESP12F(3)`

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
