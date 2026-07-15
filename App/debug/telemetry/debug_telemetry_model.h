#ifndef DEBUG_TELEMETRY_MODEL_H
#define DEBUG_TELEMETRY_MODEL_H

#include "adc_monitor.h"
#include "chassis_service.h"
#include "encoder_driver.h"
#include "esp12f_service.h"
#include "imu_bmi270.h"
#include "line_uart.h"
#include "motor_driver.h"
#include "ps2_control_service.h"
#include "safety_service.h"

typedef struct
{
    adc_monitor_state_t         adc;
    encoder_state_t             encoder;
    chassis_service_snapshot_t  chassis;
    safety_service_snapshot_t   monitor;
    motor_driver_state_t        motor;
    imu_bmi270_state_t          imu;
    ps2_control_service_state_t ps2;
    line_uart_state_t           line;
    esp12f_service_state_t      esp;
    float                       motor_log_speed_mps[MOTOR_ID_COUNT];
} debug_full_log_snapshot_t;

#endif
