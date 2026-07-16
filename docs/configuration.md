# Beta5 配置边界

## 运行参数

Parameter Management 是唯一运行时参数所有者，优先级固定为：

```text
factory defaults < compatible Flash image < current RAM override
```

Flash schema 4、340 字节 bundle、364 字节 image、双槽/CRC 和 schema 1–3 迁移保持逐字节兼容。RAM 变更只有显式保存才进入 Flash。

## Service 配置

每个能力公开自己的 `*_config_t`，只包含该能力需要的产品值。`robot_config_t` 位于 App，是这些配置的聚合；`App_Init` 负责按依赖顺序注入。Service 不读取 App 配置，不保存指向临时配置对象的指针。

Parameter、State、Power、System 基础能力已消费 App 注入配置：Parameter 接收完整 factory defaults 和三项持久化策略；State 接收 wheel/IMU 独立新鲜度；Power 接收电流零点静止速度；System 接收 App 根据既有任务周期计算出的九项超时毫秒值。Parameter factory defaults 允许在有效范围内形成产品变体，其余尚未迁移能力仍只接受与 Beta4 逐字段等价的值。

启动参数合并顺序已经由 App 明确执行：先用注入 factory defaults 初始化唯一 RAM owner，再按 `load_flash_on_boot` 加载兼容 Flash bundle，之后运行期整包 RAM 更新只递增 generation，不会隐式保存。`persist_imu_calibration` 和 `persist_current_zero` 分别控制首次 IMU 自动标定保存以及保存 bundle 时是否携带电流零点；Beta5 默认值均为 1，与 Beta2 行为一致。

## 调度配置

任务周期、优先级、栈和 UI 周期只属于 App/Core 生成调度边界。System Monitoring 接收的是 App 派生后的超时毫秒值，不再读取任务周期宏：

| 任务 | 周期/触发 | 优先级 |
| --- | --- | --- |
| Safety | 20 ms | High |
| Motor | 10 ms | AboveNormal |
| Host | 5 ms | Normal |
| IMU | DRDY，10 ms timeout | Normal |
| Line | 5 ms | BelowNormal |
| ESP12F | 5 ms | BelowNormal |
| PS2 | 20 ms | Normal |
| LED | 50 ms | Low |
| OLED | 100 ms | Low |
| Debug | 现有周期 | BelowNormal |

Beta5 不修改这些值，只把业务代码中重复的调度常量收口到 App 配置。

## BSP 配置

GPIO/TIM/UART/ADC 映射、电气换算、硬件 PWM 上限、Break/nFAULT 和设备地址属于 BSP。Service 只能接收换算后的事实或通过 BSP API 操作设备，不能包含 HAL handle、引脚宏或寄存器类型。

迁移期间 `Domain/config/control_config.h` 与 `BSP/bsp_config.h` 是只读兼容映射：现有引用登记在架构检查器中，只允许减少，禁止新增；Beta5 最终模式要求全部删除。
