# F407 V2.0 发布检查表

本检查表只记录可追溯的验证结果。未附固件 SHA、日志或测量记录的项目不得勾选。

## v1.0.0-rc2 / HIL_PENDING

- [x] `VERSION` 为 `1.0.0-rc2`
- [x] 看门狗双 completion、年龄门和 Evaluate/Feed/Commit Host 覆盖
- [x] 40 ms Safety permit 在 Motor 侧独立过期停机
- [x] 五模式、PS2 去抖接管和 MANUAL 后 neutral→enable rearm
- [x] 控制 timing、非有限数值、轮速重获取和异常 delivery latch Host 覆盖
- [x] effective parameter CRC 与四轮 `pid_kd=0` schema 4 兼容覆盖
- [x] gyro OADEV 的 white/flicker/RRW/constant、缺口和抖动 fixture
- [x] Host 半包按最后字节 100 ms 超时，TX 使用 RESERVED/READY 提交
- [x] Upper Protocol v3 与 Flash schema 4 布局不变
- [x] 最终本地 `main` Host Test 62/62 通过
- [x] 最终本地 `main` 的 Debug/Release、size、架构、所有权、CubeMX 和格式证据
- [x] Host AddressSanitizer / UndefinedBehaviorSanitizer 61/61；插桩放大的静态栈预算项由普通 Host 独立通过
- [x] cppcheck 2.21.0 检查 416 个 C/H 文件通过；clang-format 18.1.8 通过
- [x] arduino-cli 1.5.1、ESP8266 Core 3.1.2、WebSockets 2.7.2 编译与安全策略检查通过
- [x] Lower STATUS → Upper v3 decoder → WheelObservation → formal odometry 联合 fixture 40/40（Upper `c472bc6`）
- [ ] rc2 HIL 项目全部归档
- [ ] 创建本地 annotated tag `v1.0.0-rc2`（HIL 全绿后）

历史 `v1.0.0-rc1` 已存在并指向 `bc472cc`，不得移动或重建。

最终本地软件门禁 size：Debug `text=189212, data=532, bss=90264`；Release
`text=168876, data=528, bss=90248`。以上不代表实板 HIL 已完成。

## v1.0.0-rc1 / Upper Protocol v3

- [x] `VERSION` 为 `1.0.0-rc1`
- [x] Host/ESP v3-only，协议长度与 v3 golden vector 固定
- [x] schema、C encoder、Python codec 和三组布局 golden vector 一致
- [x] Safety 恢复要求 Host/ESP 新 disable rearm
- [x] CLEAR_FAULT/LINE_CTRL 经 mailbox 和 App 分发，业务结果由 Service 生成
- [x] Service DAG、唯一所有权和 codec/encoder 纯度门禁通过
- [x] 当前工作树 Debug 与 Release build 通过
- [ ] 最终 clean commit 的 Debug 与 Release 重建、HELLO identity 校验通过
- [x] 58 项 Host tests 全部通过
- [ ] ESP8266 Core 3.1.2 / WebSockets 2.7.2 编译通过
- [ ] 与 Ros2_Slam v0.4.0 固定 commit 的逐字节交叉金样通过
- [ ] 双链路 HIL、故障注入和连续 30 min 运行通过
- [ ] clean commit 对应 ELF/HEX/BIN、ESP bin 与 HIL 报告归档
- [ ] 创建本地 annotated tag `v1.0.0-rc1`（仅在以上硬门槛全部通过后）

## Beta5.3 静态门禁

- [x] `VERSION` 为 `1.0.0-beta5.3`
- [x] 五层依赖、命名和 Service 所有权门禁通过
- [x] 53/53 Host tests 通过
- [x] Upper Protocol V2 固定金样通过
- [x] Flash Schema 4 编解码、尺寸和迁移测试通过
- [x] Debug clean build 通过
- [x] Release clean build 通过
- [x] `git diff --check` 通过
- [ ] Beta5.3 板级 HIL（进入 `1.0.0-rc1` 前执行）

## Beta2 静态门禁

- [x] `VERSION` 为 `1.0.0-beta2`
- [x] Debug clean build 通过
- [x] Release clean build 通过
- [x] 34/34 Host tests 通过
- [ ] ESP8266 固件编译通过
- [x] clang-format 全树检查通过
- [ ] cppcheck 全树检查通过
- [x] CubeMX 安全配置检查通过
- [x] `git diff --check` 通过
- [ ] Release 固件报告固定 SHA、`dirty=0`

预验收：2026-07-13 在 COM3 上使用 Debug、`dirty=1` 候选固件通过 smoke 和 IMU 静默标定调度检查；
不得据此勾选 Release 固件或正式 HIL 项。

## P0 板级安全

- [ ] 编码器临界区最大值小于 20 us
- [ ] 调试输出 deadman 最迟 500 ms 停车
- [ ] 编码器断线最迟 200 ms 锁停
- [ ] Flash 操作期间 PWM 始终为零
- [ ] ESTOP 来源和解除权限验收通过
- [ ] TIM1 Break、TIM8 诊断和 nFAULT 验收通过
- [ ] 3S 欠压告警、锁停和恢复验收通过
- [ ] PS2 定角精度和安全取消验收通过
- [ ] 冷启动 20 次、热复位 20 次通过
- [ ] ESP owner、observer、heartbeat 和 stale 验收通过

## 参数与集成

- [ ] 编码器、轮径和 PID 参数定型
- [ ] 电流链标定完成
- [ ] IMU 精度测试完成
- [ ] 有效轮距和直行四工况定型
- [ ] 巡线实车测试完成
- [ ] ROS2 协议与树莓派串口联调完成
- [ ] 一小时混合运行通过

## 发布产物

- [ ] ELF、HEX、BIN、MAP 完整
- [ ] ESP12F 固件产物完整
- [ ] HIL JSON、串口 transcript 和 P0 验收表完整
- [ ] 参数、IMU、电流和直行报告绑定固件 SHA
- [ ] CHANGELOG 和已知限制已同步
