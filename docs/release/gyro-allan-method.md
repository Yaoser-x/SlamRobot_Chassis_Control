# RC2 陀螺噪声分析方法

`scripts/analyze_imu.py` 对校正但未额外滤波的 `imu_gyro_corr_*_dps` 计算 gyro-rate
overlapping Allan deviation（OADEV）。估计器采用
[NIST SP 1065](https://tf.nist.gov/general/pdf/2220.pdf) 的 overlapping Allan variance
定义；陀螺噪声类型、单位和换算规则由本文冻结，不从姿态 yaw 曲线推断。

## 输入与预处理

- 输入单位固定为 `deg/s`，默认 `bias_correction_enabled=true`、`filter_enabled=false`。
- 默认 `detrend_policy=none`。脚本估计并报告角漂移，但不静默扣除；只有显式
  `--detrend linear` 才输出线性去趋势后的独立分析。
- 默认拒绝 `imu_quality & 0x1ff != 0` 的样本。无效样本和时间缺口切断片段，缺口两侧不得拼接。
- 每段至少 64 个样本；采样间隔使用 `median_dt`。非缺口间隔的 jitter 必须满足
  `p95 <= 5%` 且 `max <= 20%`，否则拒绝整组分析。
- tau 只取二次幂 cluster；每个 OADEV 点合计至少 20 个 overlapping terms。多段以有效项数加权。

## 系数提取

每个候选区间使用连续 5 点在 `log10(tau)-log10(ADEV)` 上线性拟合：

| 系数 | 斜率门 | 拟合门 | 输出 |
| --- | --- | --- | --- |
| ARW | `-0.5 ± 0.1` | `R² >= 0.90` | `deg/sqrt(h)`、`rad/sqrt(s)` |
| BI | `0 ± 0.1` | `R² >= 0.90` | plateau 几何中心拟合值 `/0.664`，`deg/h` |
| RRW | `+0.5 ± 0.1` | `R² >= 0.90` | `deg/h/sqrt(h)` 及 SI 时间基准值 |

不存在可信 BI plateau 时必须输出 `null`，禁止用全局 ADEV 最小值替代。常量偏置应被
OADEV 差分消除，不能作为 BI fixture；确定性测试分别使用 white-rate、1/f flicker、
rate-random-walk 和常量偏置。

## 命令与产物

```powershell
python scripts/analyze_imu.py imu.csv --output-dir allan-out `
  --firmware-sha <sha> --parameter-crc <effective-crc> `
  --imu-odr 100Hz --imu-bandwidth-profile normal
```

产物固定为 `allan_result.json`、`allan_curve.csv`、`summary.md`。JSON 记录 firmware SHA、
effective parameter CRC、采样率、样本/片段/拒绝计数、温度范围、tau/ADEV、三轴系数、
输入信号阶段、ODR、带宽 profile、bias/filter 状态、角漂移、去趋势策略和算法版本。
`yaw_output_stability` 只描述姿态输出稳定性，与 gyro OADEV 结论分开。
