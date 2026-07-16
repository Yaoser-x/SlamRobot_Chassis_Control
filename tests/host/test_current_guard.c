#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "bsp_config.h"
#include "motor_current_limiter.h"
#include "param_service.h"
#include "power_management_status.h"

static const motion_control_config_t guard_config = {
#if defined(CURRENT_GUARD_TEST_SOFT_LIMIT)
    .motor_current_limiter_observe_only = 0U,
    .current_soft_limit_enabled         = 1U,
#else
    .motor_current_limiter_observe_only = 1U,
    .current_soft_limit_enabled         = 0U,
#endif
};

uint32_t ParamService_GetSnapshot(param_model_t *params)
{
    *params = (param_model_t){0};
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        params->current_observe_a[i]    = MOTOR_STALL_CURRENT_A;
        params->current_soft_limit_a[i] = MOTOR_CURRENT_LIMIT_A;
        params->current_fault_a[i]      = MOTOR_STALL_CURRENT_A;
    }
    params->current_fault_debounce_ms = (uint16_t)(MOTOR_OVERCURRENT_DEBOUNCE_COUNT * 10U);
    return 1U;
}

uint32_t ParameterManagement_GetSnapshot(param_model_t *params)
{
    return ParamService_GetSnapshot(params);
}

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)printf("FAIL: %s\n", message);
        exit(1);
    }
}

static power_management_status_t valid_adc_state(void)
{
    power_management_status_t state = {0};
    state.current_valid             = 1U;
    state.current_control_valid     = 1U;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        state.current_control_valid_mask |= (uint8_t)(1U << i);
    }
    return state;
}

#if !defined(CURRENT_GUARD_TEST_SOFT_LIMIT)
static void test_observe_only_does_not_change_pwm(void)
{
    power_management_status_t     adc = valid_adc_state();
    motor_current_limiter_state_t state;
    uint8_t                       limited = 0U;
    int16_t                       applied;

    MotorCurrentLimiter_Init(&guard_config);
    adc.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;
    applied                    = MotorCurrentLimiter_ApplyMotorLimit(MOTOR_ID_M2, 600, &adc, 1000U, &limited);
    MotorCurrentLimiter_GetState(&state);

    require_int(applied == 600, "observe-only keeps requested PWM");
    require_int(limited == 0U, "observe-only does not report applied soft limit");
    require_int(state.observe_over_limit[MOTOR_ID_M2] != 0U, "observe-only records over-limit observation");
    require_int(state.soft_limit_would_apply[MOTOR_ID_M2] == 0U,
                "disabled soft-limit threshold does not report would-apply");
    require_int(state.soft_limit_applied[MOTOR_ID_M2] == 0U, "observe-only does not apply soft limit");
}

static void test_fault_would_latch_uses_consecutive_samples(void)
{
    power_management_status_t     adc = valid_adc_state();
    motor_current_limiter_state_t state;

    MotorCurrentLimiter_Init(&guard_config);
    adc.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;
    for (uint8_t i = 0U; i < (uint8_t)(MOTOR_OVERCURRENT_DEBOUNCE_COUNT - 1U); ++i)
    {
        (void)MotorCurrentLimiter_ApplyMotorLimit(MOTOR_ID_M2, 600, &adc, 1000U + i, 0);
    }
    MotorCurrentLimiter_GetState(&state);
    require_int(state.fault_would_latch[MOTOR_ID_M2] == 0U, "guard does not latch before debounce threshold");

    adc.current_a[MOTOR_ID_M2] = 0.0f;
    (void)MotorCurrentLimiter_ApplyMotorLimit(MOTOR_ID_M2, 600, &adc, 2000U, 0);

    adc.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;
    (void)MotorCurrentLimiter_ApplyMotorLimit(MOTOR_ID_M2, 600, &adc, 3000U, 0);
    MotorCurrentLimiter_GetState(&state);
    require_int(state.fault_would_latch[MOTOR_ID_M2] == 0U, "below-threshold sample resets debounce");
}
#endif

static void test_invalid_current_never_intervenes(void)
{
    power_management_status_t     adc = valid_adc_state();
    motor_current_limiter_state_t state;
    uint8_t                       limited = 99U;
    int16_t                       applied;

    MotorCurrentLimiter_Init(&guard_config);
    adc.current_control_valid      = 0U;
    adc.current_control_valid_mask = 0U;
    adc.current_a[MOTOR_ID_M2]     = MOTOR_STALL_CURRENT_A + 2.0f;
    applied                        = MotorCurrentLimiter_ApplyMotorLimit(MOTOR_ID_M2, 700, &adc, 1000U, &limited);
    MotorCurrentLimiter_GetState(&state);

    require_int(applied == 700, "invalid current keeps requested PWM");
    require_int(limited == 0U, "invalid current clears limited output flag");
    require_int(state.observe_over_limit[MOTOR_ID_M2] == 0U, "invalid current does not observe over-limit");
    require_int(state.control_valid[MOTOR_ID_M2] == 0U, "invalid current reports guard invalid");
}

#if defined(CURRENT_GUARD_TEST_SOFT_LIMIT)
static void test_soft_limit_scales_when_enabled(void)
{
    power_management_status_t     adc = valid_adc_state();
    motor_current_limiter_state_t state;
    uint8_t                       limited = 0U;
    int16_t                       applied;

    MotorCurrentLimiter_Init(&guard_config);
    adc.current_a[MOTOR_ID_M2] = 2.0f;
    applied                    = MotorCurrentLimiter_ApplyMotorLimit(MOTOR_ID_M2, 600, &adc, 1000U, &limited);
    MotorCurrentLimiter_GetState(&state);

    require_int(applied == 300, "enabled soft limit scales PWM by current ratio");
    require_int(limited != 0U, "enabled soft limit reports applied limit");
    require_int(state.soft_limit_would_apply[MOTOR_ID_M2] != 0U, "enabled soft limit records would apply");
    require_int(state.soft_limit_applied[MOTOR_ID_M2] != 0U, "enabled soft limit records applied");
}
#endif

static void test_disabled_motor_does_not_participate(void)
{
    power_management_status_t     adc = valid_adc_state();
    motor_current_limiter_state_t state;
    uint8_t                       limited = 99U;
    int16_t                       applied;

    MotorCurrentLimiter_Init(&guard_config);
    adc.current_a[MOTOR_ID_M1] = MOTOR_STALL_CURRENT_A + 2.0f;
    applied                    = MotorCurrentLimiter_ApplyMotorLimit(MOTOR_ID_M1, 600, &adc, 1000U, &limited);
    MotorCurrentLimiter_GetState(&state);

    require_int(applied == 0, "disabled motor output is forced to zero by guard");
    require_int(limited == 0U, "disabled motor clears limited flag");
    require_int(state.observe_over_limit[MOTOR_ID_M1] == 0U, "disabled motor does not observe over-limit");
}

int main(void)
{
#if !defined(CURRENT_GUARD_TEST_SOFT_LIMIT)
    test_observe_only_does_not_change_pwm();
    test_fault_would_latch_uses_consecutive_samples();
#endif
    test_invalid_current_never_intervenes();
#if defined(CURRENT_GUARD_TEST_SOFT_LIMIT)
    test_soft_limit_scales_when_enabled();
#endif
    test_disabled_motor_does_not_participate();
    return 0;
}
