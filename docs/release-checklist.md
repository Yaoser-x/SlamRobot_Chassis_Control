# F407 V1.0 发布检查表

本检查表只记录可追溯的验证结果。未附固件 SHA、日志或测量记录的项目不得勾选。

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
