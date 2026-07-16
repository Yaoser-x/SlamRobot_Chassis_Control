# Beta5 五层架构

Beta5 最终只保留 `App / Service / BSP / Algorithm / Platform` 五类用户代码。`Core / Drivers / Middlewares` 是生成区或第三方边界，不参与业务所有权。

依赖方向固定为：App 负责编排，Service 负责业务能力，BSP 负责设备事实，Algorithm 负责无状态或显式状态的纯逻辑，Platform 负责 RTOS/Cortex-M 系统能力。

| 层 | 职责 | 禁止事项 |
| --- | --- | --- |
| App | 初始化、任务调度、ISR adapter、发布读模型、OLED 和 Debug 适配 | 持有业务状态；绕过 Service 决策；新增未列入白名单的 BSP 直连 |
| Service | 十项能力的唯一状态和决策所有者；定义公开 config/status DTO | 公共头泄漏 BSP/HAL/RTOS/Platform/internal 类型；直接调用 HAL/RTOS/intrinsic |
| BSP | 原始设备采样、硬件 ID/状态、寄存器映射、transport、Flash 原始读写、电机硬件输出 | 引用 App 或 Service；拥有业务仲裁、滤波策略或运行参数 |
| Algorithm | PID、运动学、控制器、滤波、标定数学、估计数学和值类型 | 引用 App/Service/BSP/Platform、HAL、RTOS 或全局硬件状态 |
| Platform | 临界区、时间、任务事件、看门狗、复位和 ResetTrace | 引用 App/Service/BSP/Algorithm 业务类型 |

## Service 能力目录

最终 Service 顶层目录固定为：

```text
Service/
  motion/          motion control
  state/           state estimation
  power/           power management
  safety/          safety management
  command/         command management
  teleoperation/   PS2 teleoperation
  line/            line following
  communication/   host and wireless communication
  param/           parameter management
  system/          system monitoring
```

一个目录对应一个公开能力节点。协议 codec、Flash schema、frame builder 等实现细节必须放在所属目录的 `internal/`，只允许本能力实现和显式开放的 Host Test 使用。

## 公开契约

- 每个 Service 定义自己的 `*_config_t`、`*_status_t`，不借用 BSP 状态作为公开 DTO。
- 公共头只允许标准库类型、Algorithm 值类型和其他 Service 的公开 DTO。
- `internal/` 头不得被其他 Service、App、BSP 或 Platform 引用。
- 迁移兼容包装不保存状态：新接口是唯一实现，旧接口只转发；所有消费者迁完后整批删除。

## 构建边界

最终 CMake 静态目标固定为 `f407_algorithm / f407_platform / f407_bsp / f407_service / f407_app`。迁移期间 `f407_domain` 可以继续存在，但不得新增文件、引用或职责；Algorithm 归位完成后一次删除。

## 自动门禁

```powershell
python scripts/check_architecture_dependencies.py
python scripts/check_naming_conventions.py
python scripts/check_service_ownership.py
```

普通模式冻结 Beta4 已登记债务且禁止新增。Beta5 发布前追加 `--final`，要求 `Domain/`、混杂配置、公共头泄漏、Service 非法边和旧电机输出所有者全部归零。

## CubeMX 边界

`.ioc`、引脚、外设、NVIC、十个任务的数量/周期/优先级均不在 Beta5 修改范围。CubeMX 重新生成时只接受 USER CODE 区业务入口差异；硬件或调度差异必须回退并单独评审。

本地软件验收不替代最终 25 项 HIL 及 Beta5 追加场景。
