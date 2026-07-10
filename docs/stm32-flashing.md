# STM32 主控固件烧录指南

本文档覆盖 STM32F407VET6 主控固件烧录。ESP12F 模组烧录见 [`esp12f-flashing.md`](esp12f-flashing.md)。

## 1. 生成固件

```bash
cmake --preset Debug
cmake --build --preset Debug
```

构建后会在 `build/Debug/` 生成：

- `F407_V2.0.elf`
- `F407_V2.0.hex`
- `F407_V2.0.bin`
- `F407_V2.0.map`

Release 版本对应 `build/Release/`。

## 2. STM32CubeProgrammer GUI

1. 用 ST-Link 连接 SWDIO、SWCLK、GND 和 3.3V 参考电压。
2. 打开 STM32CubeProgrammer，接口选择 `ST-LINK`，Port 选择 `SWD`。
3. 点击 `Connect`，确认芯片识别为 STM32F407。
4. 选择 `Erasing & Programming`。
5. File path 选择 `build/Debug/F407_V2.0.hex`。
6. 勾选 `Verify programming` 和 `Run after programming`。
7. 点击 `Start Programming`。

## 3. STM32CubeProgrammer CLI

```bash
STM32_Programmer_CLI -c port=SWD mode=UR reset=HWrst \
  -w build/Debug/F407_V2.0.hex \
  -v \
  -rst
```

若连接失败，可先尝试降低 SWD 频率：

```bash
STM32_Programmer_CLI -c port=SWD freq=1000 mode=UR reset=HWrst
```

## 4. OpenOCD

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/F407_V2.0.elf verify reset exit"
```

## 5. st-flash

```bash
st-flash --reset write build/Debug/F407_V2.0.bin 0x08000000
```

## 6. 烧录后验收

1. 打开 USART1 调试串口。
2. 复位主控，确认启动输出包含 `RESET`、`RESETTRACE` 和 `POST:`。
3. 输入：

```text
status
i2cscan
imutest
espflash status
```

`POST` 或 OLED 自检失败不会阻塞启动，但应进入 `status` 继续排查。

## 7. 常见问题

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| 无法连接 SWD | 线序错误、目标板未供电、SWD 频率过高 | 检查 GND/3.3V 参考电压，降低 SWD 频率 |
| 烧录后行为没变 | 烧录了旧产物或错误 preset | 重新运行 `cmake --build --preset Debug` 并确认文件时间 |
| Verify 失败 | Flash 写入不稳定或供电不足 | 降低 SWD 频率，确认电源和 ST-Link 接地 |
| 程序启动后反复复位 | IWDG、HardFault 或电源问题 | 查看 `RESETTRACE` 和 `status` |
