#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

/**
 * @file bsp_config.h
 * @brief BSP 层硬件配置常量。
 *
 * 包含电机布局、编码器物理参数、PWM 限制、ADC 标定等 BSP 驱动所需的
 * 编译期常量。App 层通过 chassis_config.h 间接包含此文件。
 */

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_MOTOR_COUNT                 4U

/* 电机布局编译期配置。
 *
 * CHASSIS_Mx_ENABLED:
 *   1U: 启用该路电机；参与目标分配、PID、编码器、电流、故障检测。
 *   0U: 禁用该路电机；目标/PWM/PID/编码器/电流固定为 0，nFAULT/过流/编码器异常不触发整车停机。
 *
 * CHASSIS_Mx_SIDE:
 *   MOTOR_SIDE_LEFT:  接收 left_mps。
 *   MOTOR_SIDE_RIGHT: 接收 right_mps。
 *
 * 默认两驱:
 *   M1 = 左侧，M3 = 右侧，M2/M4 禁用。
 *   V2.0 实板 M3: PWM=PE13/PC8，nFAULT=PA3，ENC=PB4/PB5，IPROPI=PC1。
 *
 * 安全规则:
 *   左右两侧必须各至少启用一路电机；否则拒绝运动输出。
 */
#ifndef CHASSIS_M1_ENABLED
#define CHASSIS_M1_ENABLED                  1U
#endif
#ifndef CHASSIS_M2_ENABLED
#define CHASSIS_M2_ENABLED                  0U
#endif
#ifndef CHASSIS_M3_ENABLED
#define CHASSIS_M3_ENABLED                  1U
#endif
#ifndef CHASSIS_M4_ENABLED
#define CHASSIS_M4_ENABLED                  0U
#endif

#ifndef CHASSIS_M1_SIDE
#define CHASSIS_M1_SIDE                     MOTOR_SIDE_LEFT
#endif
#ifndef CHASSIS_M2_SIDE
#define CHASSIS_M2_SIDE                     MOTOR_SIDE_LEFT
#endif
#ifndef CHASSIS_M3_SIDE
#define CHASSIS_M3_SIDE                     MOTOR_SIDE_RIGHT
#endif
#ifndef CHASSIS_M4_SIDE
#define CHASSIS_M4_SIDE                     MOTOR_SIDE_RIGHT
#endif

/* 方向符号配置。
 *
 * CHASSIS_Mx_MOTOR_DIR:
 *   1 / -1。PWM 方向修正。底盘正输出必须对应车辆前进。
 *
 * CHASSIS_Mx_ENCODER_DIR:
 *   1 / -1。编码器速度方向修正。车辆前进时 status 中 mm/s 应为正。
 */
#define CHASSIS_M1_MOTOR_DIR                -1
#define CHASSIS_M2_MOTOR_DIR                1
#define CHASSIS_M3_MOTOR_DIR                1
#define CHASSIS_M4_MOTOR_DIR                1
#define CHASSIS_M1_ENCODER_DIR              1
#define CHASSIS_M2_ENCODER_DIR              1
#define CHASSIS_M3_ENCODER_DIR              -1
#define CHASSIS_M4_ENCODER_DIR              -1

/* PWM 输出限制 */
#define CHASSIS_PWM_MAX_PERMILLE            900
#define CHASSIS_PWM_DEADBAND_PERMILLE       0
#define MOTOR_DIRECTION_CHANGE_COAST_CYCLES 2U
#define DRV8874_WAKE_DELAY_MS               2U

/* 编码器物理参数 */
#define CHASSIS_ENCODER_BASE_PPR            11.0f
#define CHASSIS_ENCODER_QUADRATURE_MULT     4.0f
#define CHASSIS_MOTOR_GEAR_RATIO            56.0f

/* 轮子物理参数 */
#define CHASSIS_WHEEL_RADIUS_M              0.035f
#define CHASSIS_TRACK_WIDTH_M               0.178f  /* 左右轮距 (m)，非轴距 */
#define CHASSIS_WHEEL_BASE_M                CHASSIS_TRACK_WIDTH_M  /* 向后兼容别名 */

/* 编码器采样时序 */
#define CHASSIS_MIN_ENCODER_DT_MS           1U
#define CHASSIS_MAX_ENCODER_DT_MS           100U

/* ADC 监控参数 */
#define ADC_MONITOR_CHANNEL_COUNT           5U
#define ADC_MONITOR_VREF_V                  3.3f
#define ADC_MONITOR_RESOLUTION_COUNTS       4095.0f
#define ADC_MONITOR_BATTERY_R_UPPER_OHM     47000.0f
#define ADC_MONITOR_BATTERY_R_LOWER_OHM     10000.0f
#define ADC_MONITOR_BATTERY_DIVIDER         ((ADC_MONITOR_BATTERY_R_UPPER_OHM + ADC_MONITOR_BATTERY_R_LOWER_OHM) / ADC_MONITOR_BATTERY_R_LOWER_OHM)
#define ADC_MONITOR_BATTERY_FILTER_ALPHA    0.10f
#define ADC_MONITOR_CURRENT_ZERO_SAMPLES    256U
#define ADC_MONITOR_CALIBRATION_ENABLED     1U

/* 电机电流参数 */
#define MOTOR_CURRENT_SHUNT_OHM             0.1f
#define MOTOR_CURRENT_ZERO_V                0.0f
#define MOTOR_CURRENT_VOLTS_PER_AMP         0.1f
#define MOTOR_CURRENT_FILTER_ALPHA          0.25f
#define MOTOR_CURRENT_LIMIT_A               0.8f
#define MOTOR_OVERCURRENT_DEBOUNCE_COUNT    5U

/* 电机保护阈值 */
#define MOTOR_RATED_CURRENT_A               0.65f
#define MOTOR_STALL_CURRENT_A               2.4f

#ifdef __cplusplus
}
#endif

#endif /* BSP_CONFIG_H */
