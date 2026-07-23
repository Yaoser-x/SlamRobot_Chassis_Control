# Firmware v1.0.0-rc1 候选基线

版本文件为 `1.0.0-rc1`。本轮起始审计基线是 clean commit `366a0385290d526009e6cd3bbdaa7b74b2fecad6`；该值不是最终标签目标。

| 项目 | 候选值 |
| --- | --- |
| Upper Protocol | v3，固定 payload |
| Hardware revision | `0x00020000` |
| STM32CubeMX | 6.15.0 / DB 6.0.150 |
| FreeRTOS | 10.3.1 |
| ARM GCC | GNU Tools for STM32 13.3.rel1 / GCC 13.3.1 |
| 参数持久化 schema | 4 |
| 默认 enabled mask | `0x06`（M2+M3） |
| Host / ESP 命令租约 | 200 ms / 500 ms |
| v3 golden SHA-256（rc1 候选） | `8406A0F8D0B450D1FC1AF27EF6B2B5C17742285C732EEB8705DA4BD2855E2A0E` |

计划原值“参数 schema 3”已按实际 `FLASH_PARAM_SCHEMA_VERSION=4` 修正。最终 commit、annotated tag 目标、参数 identity CRC 及 ELF/BIN/HEX/MAP SHA-256 不在 Git 跟踪文档中自引用，而由 clean Release 构建后的 `release-manifest.json` 和 `SHA256SUMS` 记录。

dirty 或无法读取 Git 的构建必须保持 fail-closed：HELLO commit 全零并移除 BUILD_IDENTITY capability。clean 构建的 HELLO 20-byte commit 必须与最终标签目标一致。

当前状态：软件冻结实施中；双链路 HIL、实车停车和 release manifest 未完成，因此不得创建 `v1.0.0-rc1` 标签。
