# Beta5 五层架构

Beta5 最终只保留 `App / Service / BSP / Algorithm / Platform` 五类用户代码。`Core / Drivers / Middlewares` 是生成区或第三方边界，不参与业务所有权。

依赖方向固定为：App 负责编排，Service 负责业务能力，BSP 负责设备事实，Algorithm 负责无状态或显式状态的纯逻辑，Platform 负责 RTOS/Cortex-M 系统能力。

| 层 | 职责 | 禁止事项 |
| --- | --- | --- |
| App | 初始化和任务调度；持有跨 Service 工作流、租约、超时和重试状态；硬件相关 ISR、OLED、Debug 由独立 adapter 构建目标承载 | 复制或拥有 Service 领域状态；绕过 Service 决策；普通 App 目标直连 BSP、Algorithm 或 HAL |
| Service | 十项能力的唯一状态和决策所有者；定义公开 config/status DTO | 公共头泄漏 BSP/HAL/RTOS/Platform/internal 类型；直接调用 HAL/RTOS/intrinsic |
| BSP | 原始设备采样、硬件 ID/状态、寄存器映射、transport、Flash 原始读写、电机硬件输出 | 引用 App 或 Service；拥有业务仲裁、滤波策略或运行参数 |
| Algorithm | PID、运动学、控制器、滤波、标定数学、估计数学和值类型 | 引用 App/Service/BSP/Platform、HAL、RTOS 或全局硬件状态 |
| Platform | 临界区、时间、任务事件、看门狗、复位和 ResetTrace | 引用 App/Service/BSP/Algorithm 业务类型 |

## Service 能力目录

最终 Service 顶层目录固定为：

```text
Service/
  motion_control/
  state_estimation/
  power_management/
  safety_management/
  command_management/
  teleoperation/   PS2 teleoperation
  line_following/
  communication/   host and wireless communication
  parameter_management/
  system_monitoring/
```

一个目录对应一个公开能力节点。协议 codec、Flash schema、frame builder 等实现细节必须放在所属目录的 `internal/`，只允许本能力实现和显式开放的 Host Test 使用。

## 公开契约

- 每个 Service 定义自己的 `*_config_t`、`*_status_t`，不借用 BSP 状态作为公开 DTO。
- 公共头只允许标准库类型、Algorithm 值类型和其他 Service 的公开 DTO。
- Service 公共头禁止 callback/function-pointer ports；跨 Service 事实收集和持久化编排必须由 App 主动完成。
- Service 只能沿声明的无环 DAG 使用兄弟能力；跨能力会话、租约和提交顺序必须由 App 编排。
- `internal/` 头不得被其他 Service、App、BSP 或 Platform 引用。
- 生产代码不保留兼容包装；必要的历史行为夹具只能存在于 `tests/host/compat/`。

## 构建边界

最终 CMake 静态目标固定为 `f407_algorithm / f407_platform / f407_bsp / f407_service / f407_app / f407_app_adapters`。普通 `f407_app` 只链接 Service 与 Platform；`f407_app_adapters` 显式隔离硬件调试、显示、ISR 和传输适配代码。生产树禁止存在 `Domain/`、`Device/`、`Common/`、`Shared/`、`Model/`、`Utils/` 或 `Manager/` 顶层目录。

## Beta5.4 运行时边界

- 巡线 Service 只拥有请求、采样、校验和 RAM apply；App 协调器持有完整维护租约并区分 PS2 自动提交与 Debug 手动提交。
- IMU 标定的 Motion 事实和 Flash 持久化由 App 主动编排；每次开机以有效 Flash bias 为初值重新自动标定，
  State Estimation 只接收事实并推进自身状态机。
- System publish snapshot 由 App 主动 Collect 后交给 System Monitoring 发布，不允许 Service 回调 App 收集兄弟 Service 状态。
- 巡线 Service 按 Parameter generation 同步阈值和极性；RAM apply 在下一帧前生效，只有显式 save 持久化。
- IMU 启停、探测、重初始化、Profile 切换和诊断只通过 State Estimation Service；App/Debug 不直接控制 BMI270 BSP 生命周期。
- IMU 瞬时质量位按周期重算，锁存位保留历史；事件计数只由新设备事件或新样本推进。
- Motion 的停车、命令撤销和安全复位路径同时清空直行控制器及全部直行诊断字段。

## 自动门禁

```powershell
python scripts/check_architecture_dependencies.py --final
python scripts/check_naming_conventions.py --final
python scripts/check_service_ownership.py --final
```

CI 与本地发布验收统一运行 Final 门禁，要求 `Domain/`、混杂配置、公共头泄漏、Service 非法边和旧电机输出所有者全部归零。

## CubeMX 边界

`.ioc`、引脚、外设、NVIC、十个任务的数量/周期/优先级均不在 Beta5 修改范围。CubeMX 重新生成时只接受 USER CODE 区业务入口差异；硬件或调度差异必须回退并单独评审。

本地软件验收不替代最终 25 项 HIL 及 Beta5 追加场景。
