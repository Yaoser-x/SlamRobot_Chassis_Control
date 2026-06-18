#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_MOTOR_COUNT                 4U

/* 电机布局编译期配置。
 *
 * 命令链路:
 *   PS2/RPI/ESP/debug -> linear_x/angular_z -> left_mps/right_mps -> enabled motors.
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
 *
 * 常用两驱:
 *   M1+M3: CHASSIS_M2_ENABLED=0U，CHASSIS_M4_ENABLED=0U。
 *   M2+M4: CHASSIS_M1_ENABLED=0U，CHASSIS_M3_ENABLED=0U。
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

#define CHASSIS_CONTROL_PERIOD_MS           10U
#define CHASSIS_ENCODER_PERIOD_MS           10U
#define CHASSIS_ADC_PERIOD_MS               20U
#define CHASSIS_IMU_PERIOD_MS               10U
#define CHASSIS_LED_PERIOD_MS               50U
#define CHASSIS_PS2_PERIOD_MS               20U
#define CHASSIS_ESP12F_PERIOD_MS            5U
#define CHASSIS_LINE_PERIOD_MS              5U

/* 巡线跟踪控制参数 */
#define LINE_DEFAULT_ENABLED                0U
#define LINE_SPEED_MPS                      0.15f
#define LINE_KP                             2.5f
#define LINE_ANGULAR_MAX_RPS                2.0f
#define LINE_SENSOR_TIMEOUT_MS              50U
#define LINE_DETECT_THRESHOLD_COUNT         1U
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
#define PS2_LINE_TOGGLE_MASK                0x10U

#define CHASSIS_SPEED_RAMP_MPS2             1.0f
#define CHASSIS_ANGULAR_RAMP_RPS2           10.0f

#define CHASSIS_WHEEL_RADIUS_M              0.035f
#define CHASSIS_WHEEL_BASE_M                0.178f
#define CHASSIS_MIN_ENCODER_DT_MS           1U
#define CHASSIS_MAX_ENCODER_DT_MS           100U

#define CHASSIS_ENCODER_BASE_PPR            11.0f
#define CHASSIS_ENCODER_QUADRATURE_MULT     4.0f
#define CHASSIS_MOTOR_GEAR_RATIO            56.0f

/* 方向符号配置。
 *
 * CHASSIS_Mx_MOTOR_DIR:
 *   1 / -1。PWM 方向修正。底盘正输出必须对应车辆前进。
 *   用 USART1 的 m1/m2/m3/m4 F/R 验证桥臂原始方向。
 *
 * CHASSIS_Mx_ENCODER_DIR:
 *   1 / -1。编码器速度方向修正。车辆前进时 status 中 mm/s 应为正。
 *   用低速正转配合 status 验证。
 */
#define CHASSIS_M1_MOTOR_DIR                -1
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
#define ADC_MONITOR_BATTERY_FILTER_ALPHA    0.10f
#define ADC_MONITOR_CURRENT_ZERO_SAMPLES    256U
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

/* OLED SSD1306 配置 */
#define OLED_I2C_ADDR                   0x3CU  /* 7-bit addr; HAL <<1 = 0x78 */
#define OLED_TASK_PERIOD_MS             100U
#define OLED_WELCOME_DURATION_MS        5000U
#define OLED_SELFCHECK_ITEM_MS          600U
#define OLED_SELFCHECK_TOTAL_ITEMS      8U
#define OLED_ERROR_BLINK_PERIOD_MS      500U
#define OLED_MODULE_TIMEOUT_RPI_MS      500U
#define OLED_MODULE_TIMEOUT_LINE_MS     50U

#ifdef __cplusplus
}
#endif

#endif
