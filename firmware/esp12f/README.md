# ESP12F 固件编译与烧录指南

## 文件说明

- `F407_ESP12F/F407_ESP12F.ino` — ESP8266 Arduino 固件，实现首次配置 AP + owner 租约网页遥控

## 功能特性

- **首次配置**：无有效 EEPROM 记录时仅开放 `F407_Chassis_Setup`，网页设置 8–63 字节密码；magic/version/length/CRC 回读验证成功后重启。
- **WebSocket owner 租约**：首个 claimant 获得 500ms 控制租约，每 200ms heartbeat；其他客户端只读遥测。owner 断开/超时立即停车释权。
- **upper_protocol 帧协议**：与 STM32F407 USART2 通信，CRC-8/Dallas 校验
- **安全保护**：配置前不启动 WebSocket；500ms 速度 deadman 与 owner 租约独立；任意客户端可设置 ESTOP，但不能远程解除。
- **故障处理**：页面按名称显示活动/锁存原因；仅 owner 可发送 `clearfault`，并以 STM32 后续 STATUS 为准确认成功。ESTOP 始终只能在 STM32 本地解除。
- **重启中和**：ESP 每次启动都主动向 STM32 发送速度零和 `LINE_CTRL=0`，因此模块重启/掉电恢复不会遗留旧巡线模式。

## 编译环境

1. Arduino IDE 安装 **ESP8266 boards**：
   - 文件 → 首选项 → 附加开发板管理器网址：
     `https://arduino.esp8266.com/stable/package_esp8266com_index.json`
   - 工具 → 开发板 → 开发板管理器 → 搜索 `esp8266` → 安装

2. 开发板选择 **Generic ESP8266 Module**；验收默认记录 ESP8266 Core **3.1.2**
3. 安装额外库 `arduinoWebSockets` **2.7.2**

4. 关键参数设置：

| 参数 | 推荐值 |
|------|--------|
| Upload Speed | **115200** |
| Flash Mode | **DOUT** |
| Flash Size | **4MB (FS:none OTA:~1019KB)** |
| Reset Method | **None**（不依赖 DTR/RTS） |
| Crystal Frequency | 26 MHz |

`ESP8266WiFi`、`ESP8266WebServer` 来自 ESP8266 Core；`WebSocketsServer` 来自上述 `arduinoWebSockets` 库。

## 编译

网页源码保留在 `F407_ESP12F.ino` 的 `HTML_PAGE` 块中。修改页面后，先重新生成
gzip 资源并验证：

```powershell
python F407_ESP12F/tools/check_esp_safety_policy.py
python F407_ESP12F/tools/generate_html_gz.py
python F407_ESP12F/tools/check_html_gz.py
```

打开 `F407_ESP12F.ino`，点击 Arduino IDE **✓ 验证**（编译），确认编译成功无错误。

## 烧录（通过 STM32 桥接）

1. STM32 调试台（USART1）输入：

   ```
   espflash on
   ```

   提示 `"esp12f flash bridge on: close this terminal..."`

2. **关闭串口终端**，释放 COM 口。

3. Arduino IDE → 选择 STM32 USART1 对应的 COM 口 → 点击 **→ 上传**

   或使用 esptool.py：

   ```bash
   # 先导出编译好的 .bin（Arduino IDE: 项目 → 导出已编译的二进制文件）
   esptool.py --chip esp8266 --port COMx --baud 115200 write_flash 0x00000 F407_ESP12F.ino.bin
   ```

4. 烧录完成后等待 **30 秒**自动退出 bridge，或复位 STM32。

## 使用

### 首次配置与控制 AP

1. 首次上电连接开放 WiFi **`F407_Chassis_Setup`**，浏览器打开 **`http://192.168.4.1`**。
2. 设置 8–63 字节密码；成功后 ESP 重启。
3. 使用新密码连接 **`F407_Chassis`**，打开同一地址。
4. 页面显示“控制权”后才可操作摇杆；只读客户端仅看遥测，但仍可按急停。

### 控制界面

- **虚拟摇杆**（中间的圆形区域）：触摸拖动控制方向和速度
  - 上下 = 前进/后退（linear_x）
  - 左右 = 旋转（angular_z）
- **STOP**：停车
- **E-STOP**：只能设置紧急停止；锁存后页面显示“请本地解除”，需 STM32 USART1 `estop 0`。
- **巡线**：启用/禁用巡线模式
- **清除故障**：仅控制权 owner 可申请清除普通 fault；若原因仍存在，页面显示主控拒绝
- **Max spd** 滑块：速度倍率 10%~100%
- **累计**：显示 STM32 上报的 `encoder_count` 累计计数，车轮运动时持续递增或递减属于正常行为；瞬时运动状态应查看“速度”

### AT 透传测试（验证 ESP12F 硬件连通性）

STM32 调试台（USART1）输入：

```
espat on
```

然后在终端中直接发 AT 指令（需 ESP12F 已烧录固件）：

```
AT
AT+GMR
```

30 秒无操作自动退出透传。

## 自定义配置

修改 `F407_ESP12F.ino` 开头的配置常量：

```cpp
#define AP_SSID           "F407_Chassis"   // AP 热点名称
#define MAX_LINEAR_MPS    0.5f              // 最大线速度
#define MAX_ANGULAR_RPS   1.0f              // 最大角速度
```

## 故障排查

| 现象 | 原因 | 处理 |
|------|------|------|
| 网页打不开 192.168.4.1 | 没连上 ESP 热点 | 检查 WiFi 是否连接 `F407_Chassis` |
| 网页能打开但摇杆无效 | 当前是只读客户端，或 WebSocket 端口被拦截 | 确认页面显示“控制权”；确认端口 81 |
| 摇杆操作底盘不动 | STM32 端 USART2 接线或协议问题 | 检查 `status` 命令看 `esp_rx` 是否增长 |
| 烧录超时 | ESP12F 未进入下载模式 | 确认 `espflash on` 执行成功 |
| 编译报错缺少库 | 开发板选错 | 确认选了 Generic ESP8266 Module |
