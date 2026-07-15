#ifndef CHASSIS_FEEDBACK_GUARD_H
#define CHASSIS_FEEDBACK_GUARD_H

#include <stdint.h>

#include "chassis_service.h"
#include "encoder_driver.h"
#include "motor_driver.h"

typedef struct
{
    uint8_t  loss_count[MOTOR_ID_COUNT];
    uint32_t no_motion_since_ms[MOTOR_ID_COUNT];
    uint8_t  no_motion_active[MOTOR_ID_COUNT];
} chassis_feedback_guard_t;

/** Initialize chassis feedback-loss tracking. */
void ChassisFeedbackGuard_Init(chassis_feedback_guard_t *guard);

/** Reset chassis feedback-loss tracking and snapshot flags. */
void ChassisFeedbackGuard_Reset(chassis_feedback_guard_t *guard, chassis_service_snapshot_t *snapshot);

/** Check device-phase no-motion timeout faults. */
uint8_t ChassisFeedbackGuard_DetectFault(chassis_feedback_guard_t   *guard,
                                         uint32_t                    now_ms,
                                         chassis_service_snapshot_t *snapshot,
                                         const encoder_state_t      *encoder,
                                         const motor_driver_state_t *motor);

/** Debounce whether one motor feedback sample is usable by the speed loop. */
uint8_t ChassisFeedbackGuard_CheckUsable(chassis_feedback_guard_t   *guard,
                                         chassis_service_snapshot_t *snapshot,
                                         motor_id_t                  motor,
                                         float                       target_mps,
                                         float                       actual_mps,
                                         uint8_t                     encoder_valid);

#endif
