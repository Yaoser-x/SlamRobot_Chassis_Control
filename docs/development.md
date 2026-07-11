# 开发指南

## 1. 构建环境

### 1.1 必备工具

| 工具 | 最低版本 | 验证命令 | 说明 |
| --- | --- | --- | --- |
| **CMake** | 3.22 | `cmake --version` | 构建系统生成器 |
| **Ninja** | 任意 | `ninja --version` | 快速增量构建 |
| **GNU Arm Embedded Toolchain** | 10+ | `arm-none-eabi-gcc --version` | 交叉编译器，需包含 `newlib-nano` |

### 1.2 安装参考

**Windows**：
- 安装 [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html)（含 CMake/Ninja/GCC）或分别安装各组件
- 将 `arm-none-eabi-gcc` 所在目录加入 `PATH`

**Ubuntu**：
```bash
sudo apt install cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi
```

**macOS**：
```bash
brew install cmake ninja arm-none-eabi-gcc
```

---

## 2. 构建命令

### 2.1 固件构建

```bash
# 配置 + 构建 Debug
cmake --preset Debug
cmake --build --preset Debug

# 配置 + 构建 Release
cmake --preset Release
cmake --build --preset Release

# 仅构建（已配置）
cmake --build build/Debug
cmake --build build/Release

# 清理重新构建
cmake --build build/Debug --target clean
cmake --build --preset Debug
```

### 2.2 Host 单元测试

Host 测试在本地机器上运行，**无需交叉编译工具链**，纯 C 逻辑验证。

```bash
# 配置
cmake -S tests/host -B build/host-tests-ninja -G Ninja

# 构建
cmake --build build/host-tests-ninja

# 运行全部测试
ctest --test-dir build/host-tests-ninja --output-on-failure

# 运行单个测试
ctest --test-dir build/host-tests-ninja -R f407_v2_host --output-on-failure
```

### 2.3 构建产物

| 预设 | 产物路径 |
| --- | --- |
| Debug | `build/Debug/F407_V2.0.elf`, `.map`, `.hex`, `.bin` |
| Release | `build/Release/F407_V2.0.elf`, `.map`, `.hex`, `.bin` |
| Host 测试 | 17 个 CTest target，覆盖控制逻辑、ADC/电流保护、UART 协议、OLED/IMU 管线、POST、参数存储与 Flash 参数镜像 |

### 2.4 当前构建验证

```
Debug:  RAM   81.2 KB / 128 KB (61.9%)
        FLASH 180.7 KB / 512 KB (34.5%)
        Host 测试 17/17 通过 (CI host-tests)
```

---

## 3. 持续集成 (CI)

配置文件：`.github/workflows/firmware-build.yml`

**触发条件**：push 到 `main` 分支 / 向 `main` 发起 Pull Request

**CI 流程**：

| 步骤 | 操作 |
| --- | --- |
| 环境 | `ubuntu-24.04` |
| 依赖 | 固件构建: `cmake`、`ninja-build`、`gcc-arm-none-eabi`；Host 测试: `cmake`、`ninja-build`、`gcc`；静态检查: `clang-format`、`cppcheck` |
| 构建 | 矩阵构建 Debug + Release preset |
| 产物 | 上传 `firmware-Debug` 和 `firmware-Release` artifacts（含 `.elf`、`.map`、`.hex`、`.bin`） |
| Host 测试 | 独立 `host-tests` job：配置 → 构建 → `ctest`（17 个 target），通过后输出测试摘要 |
| 格式检查 | `clang-format --dry-run --Werror` 检查本次变更涉及的 App/BSP/tests C/H 文件 |
| 静态分析 | `cppcheck --enable=warning,style --error-exitcode=1` 检查本次变更涉及的 App/BSP/tests C/H 文件 |
| 内存报告 | `arm-none-eabi-size` 输出 Flash/RAM 使用量到 GitHub Step Summary |

---

## 4. 时钟配置

当前使用 **HSI (16 MHz)** 内部时钟（HSE 晶振待修复）。

| 时钟域 | 频率 |
| --- | --- |
| HSI | 16 MHz |
| PLLM / PLLN / PLLP / PLLQ | 16 / 336 / 2 / 7 |
| SYSCLK | 168 MHz |
| HCLK / Cortex FCLK | 168 MHz |
| APB1 (PCLK1) | 42 MHz |
| APB1 Timer Clock | 84 MHz (×2) |
| APB2 (PCLK2) | 84 MHz |
| APB2 Timer Clock | 168 MHz (×2) |
| HAL Timebase | TIM6 / `TIM6_DAC_IRQn` |

> **禁止提交推送与 HSI 相关的临时 `SystemClock_Config` 改动到 `main`。**

---

## 5. Git 策略

### 5.1 行尾策略

- `.gitattributes`：`* text=auto eol=lf` — 强制所有文本文件使用 LF 行尾
- `core.excludesfile` 指向 `.git/info/exclude`（仓库内），不依赖系统级全局 ignore

### 5.2 禁止提交

- `build/` 构建输出目录
- IDE 临时文件：`.idea/`、`.vscode/`、`.vs/`
- 下载产物：`.elf`、`.bin`、`.hex`、`.map`
- 系统噪声文件（`Thumbs.db`、`.DS_Store` 等）

### 5.3 提交格式

```
前缀: 中文标题
```

| 前缀 | 用途 |
| --- | --- |
| `Feat:` | 新功能 |
| `Fix:` | Bug 修复 |
| `Doc:` | 文档变更 |
| `Refactor:` | 代码重构（不改变外部行为） |
| `Test:` | 测试代码 |
| `Chore:` | 构建/工具链/CI/依赖变更 |

提交信息应详细描述改动内容与动机，不可只写标题。

### 5.4 提交前检查清单

```bash
# 1. 检查工作区状态
git status --short --branch

# 2. 检查关键文件是否有意外变更
git diff --check -- .gitignore .gitattributes README.md CMakeLists.txt \
  Core/Src/freertos.c App BSP tests/host

# 3. 固件编译
cmake --build --preset Debug

# 4. Host 测试
ctest --test-dir build/host-tests-ninja --output-on-failure
```

涉及构建配置、链接脚本、启动文件或 FreeRTOS 配置时，**额外执行**：
```bash
cmake --preset Release && cmake --build --preset Release
```

若 `git status` 显示大量 `Drivers/` 无关变更：
```bash
git restore --worktree -- Drivers   # 清除第三方库行尾噪声
```

---

## 6. 开发约束

### 6.1 代码组织

- **不修改硬件引脚定义**：业务代码只消费 `Core/Inc/main.h` 和 `.ioc` 中已有标签
- **不手动修改第三方库**：`Drivers/`（CMSIS + HAL/LL）、`Middlewares/`（FreeRTOS）保持原样
- **配置集中管理**：所有配置常量放入 `App/chassis/chassis_config.h`，修改参数优先动此文件
- **业务代码优先放入 `App/` 和 `BSP/`**：不在 CubeMX 生成区外手动修改 `Core/` 文件

### 6.2 CubeMX 生成区规则

`Core/Src/` 和 `Core/Inc/` 下的文件由 CubeMX 生成，**只能在 `USER CODE BEGIN/END` 区域写入业务代码**。

修改 `.ioc` 后重新生成代码时，必须复查以下文件的差异：

| 文件 | 关注点 |
| --- | --- |
| `CMakeLists.txt` | `App/`、`BSP/` 源文件和 include path 是否仍包含 |
| `Core/Src/freertos.c` | 任务创建代码是否在 USER CODE 区域 |
| `Core/Inc/main.h` | 引脚宏定义是否变化 |
| `Core/Src/gpio.c` | GPIO 初始化是否变化 |
| `cmake/stm32cubemx/CMakeLists.txt` | CubeMX 生成的源文件清单 |

### 6.3 安全约束

- **控制安全改动必须保留 reject-and-stop 语义**：非法命令拒绝 + 清空该源命令槽，不能静默夹紧后继续运行
- **修改时钟、链接脚本、FreeRTOS heap 或 CMake 工具链后**：必须同时验证 Debug 和 Release 构建

### 6.4 模块状态

| 模块 | 状态 | 说明 |
| --- | --- | --- |
| 底盘控制链 | ✅ 完成 | 差速模型 + PID + PWM |
| 控制源仲裁 | ✅ 完成 | 五级优先级 + 超时 + ESTOP/fault-stop |
| 编码器 | ✅ 完成 | 四路计数/差分/速度 |
| ADC/电流/电压 | ✅ 完成 | 五通道 DMA 采样 |
| BMI270 IMU | ✅ 完成 | 配置表/校准/姿态估计/诊断 |
| PS2 手柄 | ✅ 完成 | bit-bang 协议/宏指令/巡线切换 |
| ESP12F 协议 | ✅ 完成 | upper_protocol 帧协议通信 + 烧录透传桥 |
| 巡线传感器 | ✅ 完成 | DMA 帧解析 + P 控制 |
| 调试台 | ✅ 完成 | 命令台/CSV 日志字段过滤 |
| POST/参数持久化 | ✅ 基础完成 | 上电 POST、ParamStore、Flash 镜像、`get/set/set save/set reset` |
| ESP12F 网页固件 | 🟡 代码完成 / HIL 待验 | 首配 EEPROM+CRC、owner/heartbeat 租约、只读遥测广播、远程 ESTOP set-only；待 Arduino IDE 编译与多客户板测 |
| OLED SSD1306 | ✅ 完成 | I2C 驱动 + 三阶段 UI + 真实链路自检/模块在线状态 |
| HIL 冒烟 | ✅ 基础完成 | USART1 只读 smoke 脚本，不自动驱动电机 |
| 上位机 (RPI) 对接 | 🔲 待对接 | 协议已就绪 |

---

## 7. 常见排查

| 问题 | 可能原因 | 处理 |
| --- | --- | --- |
| `arm-none-eabi-gcc: command not found` | 工具链未安装或未加入 PATH | `arm-none-eabi-gcc --version` 验证；确认安装路径在 PATH 中 |
| `undefined reference to __libc_init_array` 或 `_exit` | newlib-nano 链接问题 | Ubuntu 安装 `libnewlib-arm-none-eabi`；确认链接参数含 `--specs=nano.specs` |
| GitHub Actions 无产物 | 构建失败或 artifact 路径错误 | 检查 CI 日志确认构建成功；验证 artifact 路径 `build/<Preset>/F407_V2.0.elf` |
| `git status` 显示大量 `Drivers/` 修改 | Windows `core.autocrlf` 转换行尾 | 确认 `.gitattributes` 存在 → `git restore --worktree -- Drivers` |
| CubeMX 重新生成后构建失败 | CubeMX 生成的文件覆盖了业务配置或源文件清单 | 重点复查 `CMakeLists.txt`（缺失 `App/``BSP/` 源文件）、`freertos.c`（任务创建代码可能被覆盖） |
| `ninja: build stopped: subcommand failed` | 编译错误 | 查看完整编译输出定位具体错误文件和行号 |
| 调试台 `log 1` 输出不完整或乱码 | TX 缓冲区不足或 UART 阻塞 | 减少 log 字段数（使用过滤模式）；检查 `DEBUG_CONSOLE_TX_LINE_SIZE` (768B) |
| 任务栈溢出 | `configASSERT` 触发或 `stack_free` 急剧下降 | 增大对应任务栈（`freertos.c` 中 `osThreadAttr_t.stack_size`），每次 +128W |
| 电机不转但无 fault 报错 | DRV_SLEEP_ALL 被拉低或 PWM 占空比为零 | `status` 检查 `out` 位和 `pwm` 值；`estop`/`fault` 是否激活 |
| OLED 不显示 | I2C 连接异常或器件地址不匹配 | `i2cscan` 确认地址 `0x3C` 有 ACK；检查 PB8/PB9 接线和供电 |
