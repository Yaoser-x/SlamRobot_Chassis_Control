#ifndef CURRENT_GUARD_H
#define CURRENT_GUARD_H

#include <stdint.h>

#include "motor_driver.h"
#include "motion_control_config.h"
#include "power_management_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint8_t  observe_over_limit[MOTOR_ID_COUNT];
        uint8_t  soft_limit_would_apply[MOTOR_ID_COUNT];
        uint8_t  soft_limit_applied[MOTOR_ID_COUNT];
        uint8_t  fault_would_latch[MOTOR_ID_COUNT];
        uint8_t  control_valid[MOTOR_ID_COUNT];
        int16_t  applied_permille[MOTOR_ID_COUNT];
        uint32_t observe_over_limit_count[MOTOR_ID_COUNT];
        uint32_t fault_would_latch_count[MOTOR_ID_COUNT];
    } motor_current_limiter_state_t;

    void    MotorCurrentLimiter_Init(const motion_control_config_t *config);
    int16_t MotorCurrentLimiter_ApplyMotorLimit(motor_id_t                       motor,
                                                int16_t                          requested_permille,
                                                const power_management_status_t *power_status,
                                                uint32_t                         now_ms,
                                                uint8_t                         *limited);
    void    MotorCurrentLimiter_GetState(motor_current_limiter_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
