# 下位机五层架构

固件业务代码按 `App → Service → Domain/BSP/Platform` 单向依赖。`Domain` 不依赖硬件、RTOS 或其他业务层；`Service` 通过 BSP 和 Platform 访问设备与系统能力；HAL 回调由 App 分发。

| 层 | 职责 | 禁止事项 |
| --- | --- | --- |
| App | 初始化编排、十个业务任务、ISR 分发、调试与显示适配 | 持有控制/安全状态 |
| Service | 参数 generation、控制源仲裁、底盘编排、安全锁存、传感器快照、通信组帧 | 直接调用 HAL、CMSIS-RTOS 或 Cortex-M intrinsic |
| Domain | 值类型、PID、运动学、布局、IMU/巡线/编码器数学、协议 codec | 引用 App/Service/BSP/Platform、HAL、CMSIS 或 FreeRTOS |
| BSP | ADC、TIM、PWM/GPIO、Flash storage、BMI270、UART transport | 引用 App 或 Service |
| Platform | 临界区、单调时钟、周期延时、任务事件、看门狗、复位与 ResetTrace | 引用业务层 |

## 关键所有权

- `ParamService` 独占运行时参数和 generation；`flash_param` 负责兼容既有双槽、schema、CRC 与迁移，`FlashStorage` 只执行原始 Flash 操作。
- `ControlService` 独占各控制源、ESTOP、fault-stop、maintenance lock 和 motion generation。
- `ChassisService` 是唯一电机目标下发者；参数 generation 变化时同步 PID、斜坡和 BSP 方向配置。
- `SafetyService` 独占故障锁存、清除条件和 fault-stop 决策。
- Upper/ESP Service 只组帧与分发，物理收发经 `BSP/transport`。
- IMU 任务句柄在 `osThreadNew` 成功后通过 `PlatformTaskEvent_Bind` 绑定，ISR 只发布事件。

## 构建边界

CMake 定义 `f407_domain`、`f407_platform`、`f407_bsp`、`f407_service`、`f407_app` 五个静态目标。`scripts/check_architecture_dependencies.py` 解析实际 include，并扫描 Domain/Service 的 HAL、RTOS 和 Cortex-M 直接调用；同时禁止旧模块路径和前缀重新出现。

## CubeMX 边界

`.ioc` 仅删除无业务职责的 `defaultTask`，保留十个业务任务的优先级、栈和周期。重新生成后只接受 FreeRTOS 任务声明、创建和包装入口差异；时钟、外设、引脚与 NVIC 变化必须回退并单独评审。

## 验收

```powershell
cmake --build --preset Debug
cmake --build --preset Release
cmake -S tests/host -B build/host-tests-ninja -G Ninja
cmake --build build/host-tests-ninja
ctest --test-dir build/host-tests-ninja --output-on-failure
python scripts/check_architecture_dependencies.py
git diff --check
```

本地软件验收不替代开发板验证；25 项实机检查仍是合并门禁。
