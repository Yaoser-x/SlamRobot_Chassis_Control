# Beta5 命名规范

- 顶层用户代码目录固定为 `App / Service / BSP / Algorithm / Platform`。
- C/H 文件和子目录统一 `lower_snake_case`。
- Service 顶层目录使用能力名：`motion/state/power/safety/command/teleoperation/line/communication/param/system`。
- 公开 Service 头以 `_service.h`、`_config.h`、`_status.h` 或 `_types.h` 结尾。
- 私有实现位于 `Service/<capability>/internal/`，协议、Flash 布局和 frame builder 不使用公开文件名。
- 公共函数使用能力前缀；配置类型为 `<capability>_config_t`，状态类型为 `<capability>_status_t`。
- BSP 公开 API 使用设备前缀；硬件 ID 和硬件状态类型只在 BSP 边界出现。
- Algorithm 名称描述算法本身，不使用 Service、Task、HAL 或具体外设前缀。
- 兼容包装保留旧名字但不得保存状态或被新代码引用；最终批次一次删除。

`scripts/check_naming_conventions.py` 在迁移模式检查新增文件/目录格式，在 `--final` 模式额外检查能力目录和公共头契约。
