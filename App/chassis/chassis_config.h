#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_MOTOR_COUNT                 4U

#define CHASSIS_CONTROL_PERIOD_MS           10U
#define CHASSIS_ENCODER_PERIOD_MS           10U
#define CHASSIS_ADC_PERIOD_MS               20U
#define CHASSIS_IMU_PERIOD_MS               20U
#define CHASSIS_LED_PERIOD_MS               50U
#define CHASSIS_PS2_PERIOD_MS               20U
#define CHASSIS_ESP12F_PERIOD_MS            5U
#define CHASSIS_LINE_PERIOD_MS              5U
#define UPPER_UART_TASK_PERIOD_MS           5U
#define UPPER_UART_STATUS_PERIOD_MS         50U
#define ESP12F_STATUS_PERIOD_MS             100U

#define CHASSIS_CMD_TIMEOUT_MS              500U
#define CHASSIS_PWM_MAX_PERMILLE            900
#define CHASSIS_PWM_DEADBAND_PERMILLE       0
#define MOTOR_DIRECTION_CHANGE_COAST_CYCLES 2U
#define DRV8874_WAKE_DELAY_MS               2U

#define CHASSIS_MAX_LINEAR_MPS              0.5f
#define CHASSIS_OPENLOOP_FULL_MPS           0.5f
#define CHASSIS_ANGULAR_EPSILON_RPS         0.0001f

#define PS2_LINEAR_MAX_MPS                  CHASSIS_MAX_LINEAR_MPS
#define PS2_ANGULAR_MAX_RPS                 5.0f
#define PS2_DPAD_LINEAR_MPS                 CHASSIS_MAX_LINEAR_MPS
#define PS2_DPAD_ANGULAR_RPS                PS2_ANGULAR_MAX_RPS
#define PS2_OFFLINE_FAIL_LIMIT              3U
#define PS2_AXIS_CENTER                     128
#define PS2_AXIS_DEADZONE                   18
#define PS2_MANUAL_CANCEL_THRESHOLD         0.12f
#define PS2_MACRO_SHORT_TURN_MS             1571U
#define PS2_MACRO_LONG_TURN_MS              6283U
#define PS2_MACRO_L1_MASK                   0x04U
#define PS2_MACRO_R1_MASK                   0x08U
#define PS2_MACRO_L2_MASK                   0x01U
#define PS2_MACRO_R2_MASK                   0x02U

#define CHASSIS_SPEED_RAMP_MPS2             1.0f
#define CHASSIS_ANGULAR_RAMP_RPS2           10.0f

#define CHASSIS_WHEEL_RADIUS_M              0.035f
#define CHASSIS_WHEEL_BASE_M                0.178f
#define CHASSIS_MIN_ENCODER_DT_MS           1U
#define CHASSIS_MAX_ENCODER_DT_MS           100U

#define CHASSIS_ENCODER_BASE_PPR            11.0f
#define CHASSIS_ENCODER_QUADRATURE_MULT     4.0f
#define CHASSIS_MOTOR_GEAR_RATIO            56.0f

#define CHASSIS_M1_MOTOR_DIR                1
#define CHASSIS_M2_MOTOR_DIR                1
#define CHASSIS_M3_MOTOR_DIR                1
#define CHASSIS_M4_MOTOR_DIR                1
#define CHASSIS_M1_ENCODER_DIR              1
#define CHASSIS_M2_ENCODER_DIR              1
#define CHASSIS_M3_ENCODER_DIR              -1
#define CHASSIS_M4_ENCODER_DIR              -1

#define ADC_MONITOR_CHANNEL_COUNT           5U
#define ADC_MONITOR_VREF_V                  3.3f
#define ADC_MONITOR_RESOLUTION_COUNTS       4095.0f
#define ADC_MONITOR_BATTERY_R_UPPER_OHM     47000.0f
#define ADC_MONITOR_BATTERY_R_LOWER_OHM     10000.0f
#define ADC_MONITOR_BATTERY_DIVIDER         ((ADC_MONITOR_BATTERY_R_UPPER_OHM + ADC_MONITOR_BATTERY_R_LOWER_OHM) / ADC_MONITOR_BATTERY_R_LOWER_OHM)
#define MOTOR_CURRENT_SHUNT_OHM             0.1f
#define MOTOR_CURRENT_ZERO_V                0.0f
#define MOTOR_CURRENT_VOLTS_PER_AMP         0.1f
#define MOTOR_CURRENT_FILTER_ALPHA          0.25f
#define MOTOR_CURRENT_LIMIT_A               0.8f
#define MOTOR_OVERCURRENT_DEBOUNCE_COUNT    5U
#define ADC_MONITOR_CALIBRATION_ENABLED     1U

#define CHASSIS_PID_ENABLED                 1U
#define CHASSIS_PID_CORRECTION_LIMIT        500.0f
#define CHASSIS_PID_STOP_EPSILON_MPS        0.005f
#define CHASSIS_PID_DIRECTION_EPSILON_MPS   0.02f
#define CHASSIS_PID_FEEDBACK_MIN_TARGET_MPS 0.08f
#define CHASSIS_PID_FEEDBACK_MIN_SPEED_MPS  0.01f
#define CHASSIS_PID_FEEDBACK_LOSS_COUNT     50U

#define CHASSIS_PID_KP_M1                   1200.0f
#define CHASSIS_PID_KI_M1                   0.0f
#define CHASSIS_PID_KD_M1                   0.0f
#define CHASSIS_PID_KP_M2                   1200.0f
#define CHASSIS_PID_KI_M2                   0.0f
#define CHASSIS_PID_KD_M2                   0.0f
#define CHASSIS_PID_KP_M3                   1400.0f
#define CHASSIS_PID_KI_M3                   0.0f
#define CHASSIS_PID_KD_M3                   0.0f
#define CHASSIS_PID_KP_M4                   1400.0f
#define CHASSIS_PID_KI_M4                   0.0f
#define CHASSIS_PID_KD_M4                   0.0f
#define CHASSIS_PID_INTEGRAL_LIMIT          1.5f

#define BATTERY_LOW_WARN_V                  10.5f
#define BATTERY_LOW_MONITOR_ENABLED         0U
#define MOTOR_RATED_CURRENT_A               0.65f
#define MOTOR_STALL_CURRENT_A               2.4f

#ifdef __cplusplus
}
#endif

#endif
