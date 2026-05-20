# SlamRobot_Chassis_Control

STM32F407 V2.0 底盘控制工程基线，使用 STM32CubeMX 生成外设初始化代码，并通过 CMake + Ninja + GNU Arm Embedded Toolchain 构建固件。

## 工程结构

```text
Core/                  应用入口、外设初始化与中断相关代码
Drivers/               STM32 HAL/LL、CMSIS 驱动与设备头文件
Middlewares/           FreeRTOS 等第三方中间件
cmake/                 交叉编译工具链与 STM32CubeMX CMake 目标
.github/workflows/    GitHub Actions 固件构建流水线
CMakePresets.json      Debug / Release 构建预设
F407_V2.0.ioc          STM32CubeMX 工程配置
STM32F407XX_FLASH.ld   Flash 链接脚本
```

## 环境要求

- CMake 3.22 或更高版本
- Ninja
- GNU Arm Embedded Toolchain，命令需可通过 `arm-none-eabi-gcc` 访问，并包含 newlib / nano specs 支持

本地已验证环境示例：

```bash
cmake --version
ninja --version
arm-none-eabi-gcc --version
```

## 本地构建

Debug 构建：

```bash
cmake --preset Debug
cmake --build --preset Debug
```

Release 构建：

```bash
cmake --preset Release
cmake --build --preset Release
```

构建产物默认输出到：

```text
build/Debug/F407_V2.0.elf
build/Debug/F407_V2.0.map
build/Release/F407_V2.0.elf
build/Release/F407_V2.0.map
```

## GitHub Actions

仓库包含固件构建流水线 `.github/workflows/firmware-build.yml`。当代码推送到 `main` 或向 `main` 发起 Pull Request 时，CI 会执行：

- 安装 CMake、Ninja、GNU Arm Embedded Toolchain
- 配置并构建 Debug preset
- 配置并构建 Release preset
- 上传 Debug / Release 的 `.elf` 与 `.map` 产物

## 开发约束

- CubeMX 生成代码保持小步修改，避免无关批量格式化。
- 构建输出、IDE 临时文件和系统噪声不入库。
- 提交前至少执行一次 Debug 构建。
- 修改构建配置或链接脚本时，同时验证 Debug 与 Release。
