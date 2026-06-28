#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/**
 * @file chassis_config.h
 * @brief 底盘控制算法与任务配置。
 *
 * 硬件常量（电机布局、编码器、PWM、ADC）见 BSP/bsp_config.h。
 * 本文件仅包含 App 层的控制算法、任务周期、协议和调参常量。
 *
 * @note OLED 配置暂留此文件（历史原因）；待 OLED 驱动稳定后
 *       可迁移至 BSP/bsp_config.h 或 App/display/ 独立头文件。
 */

#include "bsp_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 任务周期 */
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

#define CHASSIS_CMD_TIMEOUT_MS              500U  /* 默认超时；control_manager 已按源独立配置 */

#define CHASSIS_MAX_LINEAR_MPS              0.5f
#define CHASSIS_MAX_ANGULAR_RPS             10.0f
#define CHASSIS_OPENLOOP_FULL_MPS           0.5f
#define CHASSIS_ANGULAR_EPSILON_RPS         0.0001f

/* PS2 手柄调参 */
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

/* 速度斜坡 */
#define CHASSIS_SPEED_RAMP_MPS2             1.0f
#define CHASSIS_ANGULAR_RAMP_RPS2           10.0f
#define CHASSIS_OUTPUT_SLEW_STEP_PER_CYCLE  5

/* PID 控制 */
#define CHASSIS_PID_ENABLED                 1U
#define CHASSIS_PID_CORRECTION_LIMIT        500.0f
#define CHASSIS_PID_STOP_EPSILON_MPS        0.005f
#define CHASSIS_PID_DIRECTION_EPSILON_MPS   0.02f
#define CHASSIS_PID_FEEDBACK_MIN_TARGET_MPS 0.08f
#define CHASSIS_PID_FEEDBACK_MIN_SPEED_MPS  0.01f
#define CHASSIS_PID_FEEDBACK_LOSS_COUNT     50U

#define CHASSIS_PID_KP_M1                   50.0f
#define CHASSIS_PID_KI_M1                   8.0f
#define CHASSIS_PID_KD_M1                   0.05f
#define CHASSIS_PID_KP_M2                   1000.0f
#define CHASSIS_PID_KI_M2                   800.0f
#define CHASSIS_PID_KD_M2                   0.15f
#define CHASSIS_PID_KP_M3                   1200.0f
#define CHASSIS_PID_KI_M3                   1000.0f
#define CHASSIS_PID_KD_M3                   0.18f
#define CHASSIS_PID_KP_M4                   100.0f
#define CHASSIS_PID_KI_M4                   0.0f
#define CHASSIS_PID_KD_M4                   0.0f
#define CHASSIS_PID_INTEGRAL_LIMIT          60.0f

/* 电池监控 */
#define BATTERY_LOW_WARN_V                  10.5f
#define BATTERY_LOW_MONITOR_ENABLED         0U

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
