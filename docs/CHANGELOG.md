# Changelog

## 1.0.0-rc2 (software candidate, HIL_PENDING)

- 启动和运行期安全事实采用 fail-closed，并发布完整安全运行状态。
- 五来源使用独立 rearm 资格和结构化命令结果，保持 v3 线协议与固定优先级不变。
- App 协调 Motor/Safety 双周期完成语义，看门狗不再以任务入口 heartbeat 作为健康依据。
- 加固控制周期、非有限输入和轮速重获取；PID 公式、`kd` 与线性映射不变。
- Host RX 使用最后字节 100 ms 超时，TX 队列缩短临界区并增加压力指标。
- IMU schema 4 的 legacy `crc` 字段明确为 FNV-1a integrity hash；字段布局不变。

## 1.0.0-rc1

- 历史候选标签 `v1.0.0-rc1` 指向 `bc472cc`。
