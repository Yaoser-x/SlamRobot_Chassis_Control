#ifndef DEBUG_TELEMETRY_MODEL_H
#define DEBUG_TELEMETRY_MODEL_H

#include "power_adc_driver.h"
#include "motion_control_service.h"
#include "wheel_encoder_driver.h"
#include "wireless_communication_service.h"
#include "bmi270_driver.h"
#include "line_sensor_driver.h"
#include "motor_driver.h"
#include "teleoperation_service.h"
#include "safety_management_service.h"

typedef struct
{
    power_adc_driver_state_t       adc;
    wheel_encoder_state_t          encoder;
    motion_control_status_t        chassis;
    safety_management_status_t     monitor;
    motor_driver_state_t           motor;
    bmi270_driver_state_t          imu;
    teleoperation_status_t         ps2;
    line_sensor_driver_state_t     line;
    wireless_communication_state_t esp;
    float                          motor_log_speed_mps[MOTOR_ID_COUNT];
} debug_full_log_snapshot_t;

#endif
