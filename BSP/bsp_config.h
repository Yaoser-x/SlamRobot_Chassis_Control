#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

#include "chassis_layout_config.h"

/**
 * @file bsp_config.h
 * @brief BSP 层硬件配置常量。
 *
 * 包含电机布局、编码器物理参数、PWM 限制、ADC 标定等 BSP 驱动所需的
 * 编译期常量。App 层通过 chassis_config.h 间接包含此文件。
 */

#ifdef __cplusplus
extern "C"
{
#endif

#define CHASSIS_MOTOR_COUNT                       4U

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
 *   M2 = 左侧，M3 = 右侧，M1/M4 禁用。
 *   V2.0 实板 M2: EN/PWM=PE11，PH/GPIO=PC7，nFAULT=PD14，ENC=PD12/PD13，IPROPI=PC1。
 *   V2.0 实板 M3: EN/PWM=PE13，PH/GPIO=PC8，nFAULT=PA3，ENC=PB4/PB5，IPROPI=PC2。
 *
 * 安全规则:
 *   左右两侧必须各至少启用一路电机；否则拒绝运动输出。
 */
/* 方向符号配置。
 *
 * CHASSIS_Mx_MOTOR_DIR:
 *   1 / -1。PWM 方向修正。底盘正输出必须对应车辆前进。
 *
 * CHASSIS_Mx_ENCODER_DIR:
 *   1 / -1。编码器速度方向修正。车辆前进时 status 中 mm/s 应为正。
 */

/* PWM 输出限制 */
#define CHASSIS_PWM_MAX_PERMILLE                  900
#define CHASSIS_PWM_DEADBAND_PERMILLE             0
#define MOTOR_DIRECTION_CHANGE_SETTLE_CYCLES      2U
#define MOTOR_REVERSE_SPEED_THRESHOLD_MPS         0.02f
#define MOTOR_REVERSE_MAX_BRAKE_CYCLES            20U
#define DRV8874_WAKE_DELAY_MS                     2U
#define DRV8874_STARTUP_STABLE_MS                 5U
#define DRV8874_STARTUP_TIMEOUT_MS                50U

/* 编码器物理参数 */
#define CHASSIS_ENCODER_BASE_PPR                  11.0f
#define CHASSIS_ENCODER_QUADRATURE_MULT           4.0f
#define CHASSIS_MOTOR_GEAR_RATIO                  56.0f

/* 轮子物理参数 */
#define CHASSIS_WHEEL_RADIUS_M                    0.035f
#define CHASSIS_TRACK_WIDTH_M                     0.176f /* 有效轮距 (m)，机械测量 181.5mm，有效值 ~176mm 补偿轮胎变形 */
#define CHASSIS_WHEEL_BASE_M                      CHASSIS_TRACK_WIDTH_M /* 向后兼容别名 */

/* 编码器采样时序 */
#define CHASSIS_MIN_ENCODER_DT_MS                 1U
#define CHASSIS_MAX_ENCODER_DT_MS                 100U
#define CHASSIS_ENCODER_MAX_ABS_MPS               2.5f
#define CHASSIS_ENCODER_SPIKE_REJECT_MPS          0.45f
#define CHASSIS_ENCODER_FILTER_MIN_SAMPLES        3U
#define CHASSIS_ENCODER_REBUILD_REJECTS           3U
#define CHASSIS_ENCODER_MAX_CONSECUTIVE_ANOMALIES 10U
#define CHASSIS_ENCODER_SIDE_SPEED_DIFF_MPS       0.25f
#define CHASSIS_ENCODER_SIDE_COUNT_DIFF           1000

/* ADC 监控参数 */
#define ADC_MONITOR_CHANNEL_COUNT                 5U
#define ADC_MONITOR_VREF_V                        3.3f
#define ADC_MONITOR_RESOLUTION_COUNTS             4095.0f
#define ADC_MONITOR_BATTERY_R_UPPER_OHM           47000.0f
#define ADC_MONITOR_BATTERY_R_LOWER_OHM           10000.0f
#define ADC_MONITOR_BATTERY_DIVIDER                                                                                    \
    ((ADC_MONITOR_BATTERY_R_UPPER_OHM + ADC_MONITOR_BATTERY_R_LOWER_OHM) / ADC_MONITOR_BATTERY_R_LOWER_OHM)
#define ADC_MONITOR_BATTERY_FILTER_ALPHA 0.10f
#define ADC_MONITOR_CURRENT_ZERO_SAMPLES 256U
#define ADC_MONITOR_CALIBRATION_ENABLED  1U

/* 电机电流参数 */
#ifndef MOTOR_CURRENT_ZERO_V
#define MOTOR_CURRENT_ZERO_V 0.0f
#endif
#ifndef MOTOR_CURRENT_VOLTS_PER_AMP
#define MOTOR_CURRENT_VOLTS_PER_AMP 1.0f
#endif
#ifndef MOTOR_CURRENT_VOLTS_PER_AMP_M1
#define MOTOR_CURRENT_VOLTS_PER_AMP_M1 MOTOR_CURRENT_VOLTS_PER_AMP
#endif
#ifndef MOTOR_CURRENT_VOLTS_PER_AMP_M2
#define MOTOR_CURRENT_VOLTS_PER_AMP_M2 MOTOR_CURRENT_VOLTS_PER_AMP
#endif
#ifndef MOTOR_CURRENT_VOLTS_PER_AMP_M3
#define MOTOR_CURRENT_VOLTS_PER_AMP_M3 MOTOR_CURRENT_VOLTS_PER_AMP
#endif
#ifndef MOTOR_CURRENT_VOLTS_PER_AMP_M4
#define MOTOR_CURRENT_VOLTS_PER_AMP_M4 MOTOR_CURRENT_VOLTS_PER_AMP
#endif
#ifndef MOTOR_CURRENT_FILTER_ALPHA
#define MOTOR_CURRENT_FILTER_ALPHA 0.25f
#endif
#ifndef MOTOR_CURRENT_LIMIT_A
#define MOTOR_CURRENT_LIMIT_A 0.0f
#endif
#ifndef MOTOR_CURRENT_GUARD_OBSERVE_ONLY
#define MOTOR_CURRENT_GUARD_OBSERVE_ONLY 1U
#endif
#ifndef MOTOR_CURRENT_SOFT_LIMIT_ENABLED
#define MOTOR_CURRENT_SOFT_LIMIT_ENABLED 0U
#endif
#ifndef MOTOR_ADC_OVERCURRENT_FAULT_ENABLED
#define MOTOR_ADC_OVERCURRENT_FAULT_ENABLED 0U
#endif
#ifndef MOTOR_OVERCURRENT_DEBOUNCE_COUNT
#define MOTOR_OVERCURRENT_DEBOUNCE_COUNT 5U
#endif
#ifndef MOTOR_OVERCURRENT_STARTUP_BLANK_MS
#define MOTOR_OVERCURRENT_STARTUP_BLANK_MS 200U
#endif
#ifndef MOTOR_OVERCURRENT_STARTUP_REARM_MS
#define MOTOR_OVERCURRENT_STARTUP_REARM_MS 200U
#endif
#ifndef ADC_MONITOR_CURRENT_ZERO_MAX_SPAN_RAW
#define ADC_MONITOR_CURRENT_ZERO_MAX_SPAN_RAW 20U
#endif
#ifndef ADC_MONITOR_CONTROL_MIN_WINDOW_SAMPLES
#define ADC_MONITOR_CONTROL_MIN_WINDOW_SAMPLES 2U
#endif
#ifndef ADC_MONITOR_CONTROL_MAX_WINDOW_SAMPLES
#define ADC_MONITOR_CONTROL_MAX_WINDOW_SAMPLES 80U
#endif
#ifndef ADC_MONITOR_CURRENT_SPIKE_MAX_RAW
#define ADC_MONITOR_CURRENT_SPIKE_MAX_RAW 512U
#endif

/* 电机保护阈值 */
#define MOTOR_RATED_CURRENT_A 0.65f
#define MOTOR_STALL_CURRENT_A 2.4f

#ifdef __cplusplus
}
#endif

#endif /* BSP_CONFIG_H */
