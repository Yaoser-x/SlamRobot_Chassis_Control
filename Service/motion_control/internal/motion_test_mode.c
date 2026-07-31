#include "motion_test_mode.h"

#include "motor_hardware_layout.h"
#include "motor_output_coordinator.h"
#include "platform_critical.h"
#include "platform_time.h"

#define MOTION_TEST_MODE_MAX_PERMILLE 300

static int16_t MotionTestMode_ClampDiagnostic(int32_t value)
{
    if (value > MOTION_TEST_MODE_MAX_PERMILLE)
    {
        return MOTION_TEST_MODE_MAX_PERMILLE;
    }
    if (value < -MOTION_TEST_MODE_MAX_PERMILLE)
    {
        return -MOTION_TEST_MODE_MAX_PERMILLE;
    }
    return (int16_t)value;
}

static void MotionTestMode_ClearUnsafe(motion_test_mode_t *mode)
{
    mode->open_loop_enabled                = 0U;
    mode->raw_input_enabled                = 0U;
    mode->lease_active                     = 0U;
    mode->open_loop_side[MOTOR_SIDE_LEFT]  = 0;
    mode->open_loop_side[MOTOR_SIDE_RIGHT] = 0;
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        mode->raw_forward[index] = 0;
        mode->raw_reverse[index] = 0;
    }
}

void MotionTestMode_Init(motion_test_mode_t *mode, const motion_control_config_t *config)
{
    if (mode != 0 && config != 0)
    {
        *mode          = (motion_test_mode_t){0};
        mode->lease_ms = config->test_mode_lease_ms;
    }
}

void MotionTestMode_Cancel(motion_test_mode_t *mode)
{
    platform_critical_state_t critical;

    if (mode == 0)
    {
        return;
    }
    critical = PlatformCritical_Enter();
    MotionTestMode_ClearUnsafe(mode);
    PlatformCritical_Exit(critical);
}

void MotionTestMode_GetSnapshot(motion_test_mode_t *mode, uint32_t now_ms, motion_test_mode_snapshot_t *snapshot)
{
    platform_critical_state_t critical;

    if (mode == 0 || snapshot == 0)
    {
        return;
    }
    critical  = PlatformCritical_Enter();
    *snapshot = (motion_test_mode_snapshot_t){0};
    if (mode->lease_active != 0U && (uint32_t)(now_ms - mode->last_refresh_ms) > mode->lease_ms)
    {
        MotionTestMode_ClearUnsafe(mode);
        snapshot->expired = 1U;
    }
    snapshot->open_loop_active                 = mode->open_loop_enabled;
    snapshot->raw_input_active                 = mode->raw_input_enabled;
    snapshot->open_loop_side[MOTOR_SIDE_LEFT]  = mode->open_loop_side[MOTOR_SIDE_LEFT];
    snapshot->open_loop_side[MOTOR_SIDE_RIGHT] = mode->open_loop_side[MOTOR_SIDE_RIGHT];
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        snapshot->raw_forward[index] = mode->raw_forward[index];
        snapshot->raw_reverse[index] = mode->raw_reverse[index];
    }
    PlatformCritical_Exit(critical);
}

void MotionTestMode_SetOpenLoop(motion_test_mode_t *mode, int16_t left_permille, int16_t right_permille)
{
    platform_critical_state_t critical;

    if (mode == 0)
    {
        return;
    }
    critical                               = PlatformCritical_Enter();
    mode->open_loop_side[MOTOR_SIDE_LEFT]  = MotionTestMode_ClampDiagnostic(left_permille);
    mode->open_loop_side[MOTOR_SIDE_RIGHT] = MotionTestMode_ClampDiagnostic(right_permille);
    mode->open_loop_enabled                = (left_permille != 0 || right_permille != 0) ? 1U : 0U;
    mode->raw_input_enabled                = 0U;
    mode->lease_active                     = mode->open_loop_enabled;
    mode->last_refresh_ms                  = PlatformTime_TaskNowMs();
    PlatformCritical_Exit(critical);
}

void MotionTestMode_SetRawSides(motion_test_mode_t *mode,
                                int16_t             left_forward,
                                int16_t             left_reverse,
                                int16_t             right_forward,
                                int16_t             right_reverse)
{
    platform_critical_state_t critical;

    if (mode == 0)
    {
        return;
    }
    critical = PlatformCritical_Enter();
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        int16_t forward =
            (MotorHardwareLayout_MotorSide((motor_id_t)index) == MOTOR_SIDE_LEFT) ? left_forward : right_forward;
        int16_t reverse =
            (MotorHardwareLayout_MotorSide((motor_id_t)index) == MOTOR_SIDE_LEFT) ? left_reverse : right_reverse;
        if (MotorHardwareLayout_MotorEnabled((motor_id_t)index) == 0U)
        {
            forward = 0;
            reverse = 0;
        }
        mode->raw_forward[index] = MotionTestMode_ClampDiagnostic(forward);
        mode->raw_reverse[index] = MotionTestMode_ClampDiagnostic(reverse);
        if (mode->raw_forward[index] < 0)
        {
            mode->raw_forward[index] = 0;
        }
        if (mode->raw_reverse[index] < 0)
        {
            mode->raw_reverse[index] = 0;
        }
    }
    mode->raw_input_enabled = 0U;
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        if (mode->raw_forward[index] != 0 || mode->raw_reverse[index] != 0)
        {
            mode->raw_input_enabled = 1U;
        }
    }
    mode->open_loop_enabled = 0U;
    mode->lease_active      = mode->raw_input_enabled;
    mode->last_refresh_ms   = PlatformTime_TaskNowMs();
    PlatformCritical_Exit(critical);
}

void MotionTestMode_SetRawMotor(motion_test_mode_t *mode,
                                motor_id_t          motor,
                                int16_t             forward_permille,
                                int16_t             reverse_permille)
{
    platform_critical_state_t critical;

    if (mode == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return;
    }
    critical = PlatformCritical_Enter();
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        mode->raw_forward[index] = 0;
        mode->raw_reverse[index] = 0;
    }
    if (MotorHardwareLayout_MotorEnabled(motor) == 0U)
    {
        mode->raw_input_enabled = 0U;
        mode->open_loop_enabled = 0U;
        mode->lease_active      = 0U;
        PlatformCritical_Exit(critical);
        return;
    }
    mode->raw_forward[motor] = MotionTestMode_ClampDiagnostic(forward_permille);
    mode->raw_reverse[motor] = MotionTestMode_ClampDiagnostic(reverse_permille);
    if (mode->raw_forward[motor] < 0)
    {
        mode->raw_forward[motor] = 0;
    }
    if (mode->raw_reverse[motor] < 0)
    {
        mode->raw_reverse[motor] = 0;
    }
    mode->raw_input_enabled = (mode->raw_forward[motor] != 0 || mode->raw_reverse[motor] != 0) ? 1U : 0U;
    mode->open_loop_enabled = 0U;
    mode->lease_active      = mode->raw_input_enabled;
    mode->last_refresh_ms   = PlatformTime_TaskNowMs();
    PlatformCritical_Exit(critical);
}

void MotionTestMode_ApplyOpenLoop(const motion_test_mode_snapshot_t *test,
                                  motion_control_status_t           *chassis,
                                  wheel_speed_control_loop_t        *speed_loop,
                                  const power_management_status_t   *power,
                                  const param_model_t               *params)
{
    if (test == 0 || chassis == 0 || speed_loop == 0 || power == 0 || params == 0)
    {
        return;
    }
    WheelSpeedControlLoop_Reset(speed_loop);
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        motor_id_t motor  = (motor_id_t)index;
        int16_t    output = 0;

        chassis->motor_pid_active[index]    = 0U;
        chassis->motor_feedback_lost[index] = 0U;
        chassis->motor_error_mps[index]     = 0.0f;
        if (MotorHardwareLayout_MotorEnabled(motor) != 0U)
        {
            output = (MotorHardwareLayout_MotorSide(motor) == MOTOR_SIDE_LEFT) ? test->open_loop_side[MOTOR_SIDE_LEFT]
                                                                               : test->open_loop_side[MOTOR_SIDE_RIGHT];
        }
        MotorOutputCoordinator_SetMotorWithPower(chassis, motor, output, power, params);
    }
}

void MotionTestMode_ApplyRaw(const motion_test_mode_snapshot_t *test,
                             motion_control_status_t           *chassis,
                             wheel_speed_control_loop_t        *speed_loop,
                             const power_management_status_t   *power,
                             const param_model_t               *params)
{
    if (test == 0 || chassis == 0 || speed_loop == 0 || power == 0 || params == 0)
    {
        return;
    }
    WheelSpeedControlLoop_Reset(speed_loop);
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        motor_id_t motor  = (motor_id_t)index;
        int16_t    output = 0;

        if (MotorHardwareLayout_MotorEnabled(motor) != 0U)
        {
            output =
                MotorOutputCoordinator_Clamp((int32_t)test->raw_forward[index] - (int32_t)test->raw_reverse[index]);
        }
        MotorOutputCoordinator_SetMotorWithPower(chassis, motor, output, power, params);
        chassis->motor_pid_active[index]    = 0U;
        chassis->motor_feedback_lost[index] = 0U;
        chassis->motor_error_mps[index]     = 0.0f;
    }
}
