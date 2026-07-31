#include "motor_current_limiter.h"

#include "motor_hardware_layout.h"
#include "parameter_management_types.h"

static motor_current_limiter_state_t motor_current_limiter_state;
static uint8_t                       motor_current_limiter_over_limit_debounce[MOTOR_ID_COUNT];
static uint8_t                       motor_current_limiter_observe_only;
static uint8_t                       current_soft_limit_enabled;

static int16_t MotorCurrentLimiter_ClampPermille(int32_t value)
{
    if (value > 1000)
    {
        return 1000;
    }
    if (value < -1000)
    {
        return -1000;
    }
    return (int16_t)value;
}

static uint8_t MotorCurrentLimiter_IsControlValid(motor_id_t motor, const power_management_status_t *power_status)
{
    uint8_t mask;

    if (power_status == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return 0U;
    }
    mask = (uint8_t)(1U << (uint8_t)motor);
    return (power_status->current_control_valid != 0U && (power_status->current_control_valid_mask & mask) != 0U) ? 1U
                                                                                                                  : 0U;
}

void MotorCurrentLimiter_Init(const motion_control_config_t *config)
{
    motor_current_limiter_state        = (motor_current_limiter_state_t){0};
    motor_current_limiter_observe_only = (config != 0) ? config->motor_current_limiter_observe_only : 1U;
    current_soft_limit_enabled         = (config != 0) ? config->current_soft_limit_enabled : 0U;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        motor_current_limiter_over_limit_debounce[i] = 0U;
    }
}

int16_t MotorCurrentLimiter_ApplyMotorLimit(motor_id_t                       motor,
                                            int16_t                          requested_permille,
                                            const power_management_status_t *power_status,
                                            const param_model_t             *params,
                                            uint32_t                         now_ms,
                                            uint8_t                         *limited)
{
    uint8_t control_valid;
    uint8_t over_soft_limit  = 0U;
    uint8_t over_fault_limit = 0U;
    int16_t applied;
    uint8_t debounce_count;

    (void)now_ms;
    if (limited != 0)
    {
        *limited = 0U;
    }
    if ((uint32_t)motor >= MOTOR_ID_COUNT || params == 0)
    {
        return 0;
    }

    motor_current_limiter_state.observe_over_limit[motor]     = 0U;
    motor_current_limiter_state.soft_limit_would_apply[motor] = 0U;
    motor_current_limiter_state.soft_limit_applied[motor]     = 0U;
    motor_current_limiter_state.fault_would_latch[motor]      = 0U;
    motor_current_limiter_state.control_valid[motor]          = 0U;
    motor_current_limiter_state.applied_permille[motor]       = requested_permille;

    if (MotorHardwareLayout_MotorEnabled(motor) == 0U)
    {
        motor_current_limiter_state.applied_permille[motor] = 0;
        return 0;
    }

    applied        = requested_permille;
    debounce_count = (uint8_t)((params->current_fault_debounce_ms + 9U) / 10U);
    if (debounce_count == 0U)
    {
        debounce_count = 1U;
    }
    control_valid                                    = MotorCurrentLimiter_IsControlValid(motor, power_status);
    motor_current_limiter_state.control_valid[motor] = control_valid;
    if (control_valid == 0U || power_status == 0)
    {
        return applied;
    }

    if (power_status->current_a[motor] > params->current_observe_a[motor])
    {
        motor_current_limiter_state.observe_over_limit_count[motor]++;
        motor_current_limiter_state.observe_over_limit[motor] = 1U;
    }
    if (power_status->current_a[motor] > params->current_fault_a[motor])
    {
        over_fault_limit = 1U;
        if (motor_current_limiter_over_limit_debounce[motor] < debounce_count)
        {
            motor_current_limiter_over_limit_debounce[motor]++;
        }
    }
    else
    {
        motor_current_limiter_over_limit_debounce[motor] = 0U;
    }

    if (params->current_soft_limit_a[motor] > 0.0f
        && power_status->current_a[motor] > params->current_soft_limit_a[motor] && requested_permille != 0)
    {
        over_soft_limit                                           = 1U;
        motor_current_limiter_state.soft_limit_would_apply[motor] = 1U;
    }

    if (over_fault_limit != 0U && motor_current_limiter_over_limit_debounce[motor] >= debounce_count)
    {
        motor_current_limiter_state.fault_would_latch[motor] = 1U;
        motor_current_limiter_state.fault_would_latch_count[motor]++;
    }

    if (motor_current_limiter_observe_only == 0U && current_soft_limit_enabled != 0U && over_soft_limit != 0U)
    {
        applied = MotorCurrentLimiter_ClampPermille(
            (int32_t)((float)requested_permille
                      * (params->current_soft_limit_a[motor] / power_status->current_a[motor])));
        motor_current_limiter_state.soft_limit_applied[motor] = 1U;
        if (limited != 0)
        {
            *limited = 1U;
        }
    }

    motor_current_limiter_state.applied_permille[motor] = applied;
    return applied;
}

void MotorCurrentLimiter_GetState(motor_current_limiter_state_t *state)
{
    if (state != 0)
    {
        *state = motor_current_limiter_state;
    }
}
