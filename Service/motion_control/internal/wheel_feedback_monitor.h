#ifndef CHASSIS_FEEDBACK_GUARD_H
#define CHASSIS_FEEDBACK_GUARD_H

#include <stdint.h>

#include "motion_control_config.h"
#include "motion_control_status.h"
#include "motor_driver.h"
#include "state_estimation_status.h"

typedef struct
{
    uint8_t  loss_count[MOTOR_ID_COUNT];
    uint32_t no_motion_since_ms[MOTOR_ID_COUNT];
    uint8_t  no_motion_active[MOTOR_ID_COUNT];
    float    min_target_mps;
    float    min_speed_mps;
    uint32_t feedback_timeout_ms;
    uint8_t  feedback_loss_count;
} wheel_feedback_monitor_t;

/** Initialize chassis feedback-loss tracking. */
void WheelFeedbackMonitor_Init(wheel_feedback_monitor_t *guard, const motion_control_config_t *config);

/** Reset chassis feedback-loss tracking and snapshot flags. */
void WheelFeedbackMonitor_Reset(wheel_feedback_monitor_t *guard, motion_control_status_t *snapshot);

/** Check device-phase no-motion timeout faults. */
uint8_t WheelFeedbackMonitor_DetectFault(wheel_feedback_monitor_t              *guard,
                                         uint32_t                               now_ms,
                                         motion_control_status_t               *snapshot,
                                         const state_estimation_wheel_status_t *encoder,
                                         const motor_driver_state_t            *motor);

/** Debounce whether one motor feedback sample is usable by the speed loop. */
uint8_t WheelFeedbackMonitor_CheckUsable(wheel_feedback_monitor_t *guard,
                                         motion_control_status_t  *snapshot,
                                         motor_id_t                motor,
                                         float                     target_mps,
                                         float                     actual_mps,
                                         uint8_t                   encoder_valid);

#endif
