# Firmware v1.0.0-rc2 审计基线

版本文件为 `1.0.0-rc2`。本轮唯一审计基线是 clean commit
`1667971022d7d32b87cba0020e638c0da8914ef9`；最终目标是在该基线上增加一个本地整改提交。

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
| 软件状态 | `HIL_PENDING` |

计划原值“参数 schema 3”已按实际 `FLASH_PARAM_SCHEMA_VERSION=4` 修正。最终 commit、annotated tag 目标、参数 identity CRC 及 ELF/BIN/HEX/MAP SHA-256 不在 Git 跟踪文档中自引用，而由 clean Release 构建后的 `release-manifest.json` 和 `SHA256SUMS` 记录。

dirty 或无法读取 Git 的构建必须保持 fail-closed：HELLO commit 全零并移除 BUILD_IDENTITY capability。clean 构建的 HELLO 20-byte commit 必须与最终标签目标一致。

当前状态：软件整改与本地软件门禁已完成；双链路 HIL、实车停车和 release manifest 未完成，
因此不得创建 `v1.0.0-rc2` 标签。
