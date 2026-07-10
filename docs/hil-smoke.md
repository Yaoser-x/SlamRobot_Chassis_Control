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
