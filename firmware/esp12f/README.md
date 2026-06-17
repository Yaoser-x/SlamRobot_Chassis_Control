# ESP12F 固件编译与烧录指南

## 文件说明

- `F407_ESP12F.ino` — ESP8266 Arduino 固件，实现 WiFi 桥接 + 网页遥控

## 功能特性

- **AP+STA 智能切换**：上电先尝试连接已保存的 WiFi 路由器（12s 超时），失败自动开热点
- **WebSocket 实时遥控**：手机网页虚拟摇杆控制底盘速度，实时遥测回显
- **upper_protocol 帧协议**：与 STM32F407 USART2 通信，CRC-8/Dallas 校验
- **安全保护**：WebSocket 断开 → 自动停车；500ms 无指令 → 速度归零

## 编译环境

1. Arduino IDE 安装 **ESP8266 boards**：
   - 文件 → 首选项 → 附加开发板管理器网址：
     `https://arduino.esp8266.com/stable/package_esp8266com_index.json`
   - 工具 → 开发板 → 开发板管理器 → 搜索 `esp8266` → 安装

2. 开发板选择 **Generic ESP8266 Module**

3. 关键参数设置：

| 参数 | 推荐值 |
|------|--------|
| Upload Speed | **115200** |
| Flash Mode | **DOUT** |
| Flash Size | **4MB (FS:none OTA:~1019KB)** |
| Reset Method | **None**（不依赖 DTR/RTS） |
| Crystal Frequency | 26 MHz |

4. **不需要安装额外库**：`ESP8266WiFi`、`ESP8266WebServer`、`WebSocketsServer`、`EEPROM` 均为 ESP8266 核心自带。

## 编译

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

### AP 模式（出厂默认）

1. ESP12F 上电后未找到已保存 WiFi → 自动开热点
2. 手机/PC 连接 WiFi：**`F407_Chassis`**，密码：**`12345678`**
3. 浏览器打开 **`http://192.168.4.1`**
4. 虚拟摇杆操控底盘

### STA 模式（连接路由器）

1. 首次使用 STA 模式，需在网页控制界面发送配置指令。
   或者修改代码中的默认 SSID/密码（搜索 `sta_configured` 相关逻辑）。
2. 网页中可使用配置功能保存 WiFi 信息到 EEPROM。

### 控制界面

- **虚拟摇杆**（中间的圆形区域）：触摸拖动控制方向和速度
  - 上下 = 前进/后退（linear_x）
  - 左右 = 旋转（angular_z）
- **STOP**：停车
- **E-STOP**：紧急停止（锁存，再按一次解除）
- **巡线**：启用/禁用巡线模式
- **Max spd** 滑块：速度倍率 10%~100%

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
#define AP_PASS           "12345678"        // AP 密码
#define STA_TIMEOUT_MS    12000             // STA 连接超时
#define MAX_LINEAR_MPS    0.5f              // 最大线速度
#define MAX_ANGULAR_RPS   1.0f              // 最大角速度
```

## 故障排查

| 现象 | 原因 | 处理 |
|------|------|------|
| 网页打不开 192.168.4.1 | 没连上 ESP 热点 | 检查 WiFi 是否连接 `F407_Chassis` |
| 网页能打开但摇杆无效 | WebSocket 端口被防火墙拦截 | 确认连的是 ESP 热点；端口 81 |
| 摇杆操作底盘不动 | STM32 端 USART2 接线或协议问题 | 检查 `status` 命令看 `esp_rx` 是否增长 |
| 烧录超时 | ESP12F 未进入下载模式 | 确认 `espflash on` 执行成功 |
| 编译报错缺少库 | 开发板选错 | 确认选了 Generic ESP8266 Module |
