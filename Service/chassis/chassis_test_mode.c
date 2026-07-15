#include "chassis_test_mode.h"

#include "chassis_layout.h"
#include "chassis_output_service.h"
#include "control_config.h"
#include "platform_critical.h"
#include "platform_time.h"

static void ChassisTestMode_ClearUnsafe(chassis_test_mode_t *mode)
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

void ChassisTestMode_Init(chassis_test_mode_t *mode)
{
    if (mode != 0)
    {
        *mode = (chassis_test_mode_t){0};
    }
}

void ChassisTestMode_Cancel(chassis_test_mode_t *mode)
{
    platform_critical_state_t critical;

    if (mode == 0)
    {
        return;
    }
    critical = PlatformCritical_Enter();
    ChassisTestMode_ClearUnsafe(mode);
    PlatformCritical_Exit(critical);
}

void ChassisTestMode_GetSnapshot(chassis_test_mode_t *mode, uint32_t now_ms, chassis_test_mode_snapshot_t *snapshot)
{
    platform_critical_state_t critical;

    if (mode == 0 || snapshot == 0)
    {
        return;
    }
    critical  = PlatformCritical_Enter();
    *snapshot = (chassis_test_mode_snapshot_t){0};
    if (mode->lease_active != 0U && (uint32_t)(now_ms - mode->last_refresh_ms) > CHASSIS_TEST_MODE_LEASE_MS)
    {
        ChassisTestMode_ClearUnsafe(mode);
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

void ChassisTestMode_SetOpenLoop(chassis_test_mode_t *mode, int16_t left_permille, int16_t right_permille)
{
    platform_critical_state_t critical;

    if (mode == 0)
    {
        return;
    }
    critical                               = PlatformCritical_Enter();
    mode->open_loop_side[MOTOR_SIDE_LEFT]  = ChassisOutputService_Clamp(left_permille);
    mode->open_loop_side[MOTOR_SIDE_RIGHT] = ChassisOutputService_Clamp(right_permille);
    mode->open_loop_enabled                = (left_permille != 0 || right_permille != 0) ? 1U : 0U;
    mode->raw_input_enabled                = 0U;
    mode->lease_active                     = mode->open_loop_enabled;
    mode->last_refresh_ms                  = PlatformTime_TaskNowMs();
    PlatformCritical_Exit(critical);
}

void ChassisTestMode_SetRawSides(chassis_test_mode_t *mode,
                                 int16_t              left_forward,
                                 int16_t              left_reverse,
                                 int16_t              right_forward,
                                 int16_t              right_reverse)
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
            (ChassisLayout_MotorSide((motor_id_t)index) == MOTOR_SIDE_LEFT) ? left_forward : right_forward;
        int16_t reverse =
            (ChassisLayout_MotorSide((motor_id_t)index) == MOTOR_SIDE_LEFT) ? left_reverse : right_reverse;
        if (ChassisLayout_MotorEnabled((motor_id_t)index) == 0U)
        {
            forward = 0;
            reverse = 0;
        }
        mode->raw_forward[index] = ChassisOutputService_Clamp(forward);
        mode->raw_reverse[index] = ChassisOutputService_Clamp(reverse);
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

void ChassisTestMode_SetRawMotor(chassis_test_mode_t *mode,
                                 motor_id_t           motor,
                                 int16_t              forward_permille,
                                 int16_t              reverse_permille)
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
    if (ChassisLayout_MotorEnabled(motor) == 0U)
    {
        mode->raw_input_enabled = 0U;
        mode->open_loop_enabled = 0U;
        mode->lease_active      = 0U;
        PlatformCritical_Exit(critical);
        return;
    }
    mode->raw_forward[motor] = ChassisOutputService_Clamp(forward_permille);
    mode->raw_reverse[motor] = ChassisOutputService_Clamp(reverse_permille);
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

void ChassisTestMode_ApplyOpenLoop(const chassis_test_mode_snapshot_t *test,
                                   chassis_service_snapshot_t         *chassis,
                                   chassis_speed_loop_t               *speed_loop)
{
    if (test == 0 || chassis == 0 || speed_loop == 0)
    {
        return;
    }
    ChassisSpeedLoop_Reset(speed_loop);
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        motor_id_t motor  = (motor_id_t)index;
        int16_t    output = 0;

        chassis->motor_pid_active[index]    = 0U;
        chassis->motor_feedback_lost[index] = 0U;
        chassis->motor_error_mps[index]     = 0.0f;
        if (ChassisLayout_MotorEnabled(motor) != 0U)
        {
            output = (ChassisLayout_MotorSide(motor) == MOTOR_SIDE_LEFT) ? test->open_loop_side[MOTOR_SIDE_LEFT]
                                                                         : test->open_loop_side[MOTOR_SIDE_RIGHT];
        }
        ChassisOutputService_SetMotor(chassis, motor, output);
    }
}

void ChassisTestMode_ApplyRaw(const chassis_test_mode_snapshot_t *test,
                              chassis_service_snapshot_t         *chassis,
                              chassis_speed_loop_t               *speed_loop)
{
    adc_monitor_state_t adc;

    if (test == 0 || chassis == 0 || speed_loop == 0)
    {
        return;
    }
    AdcMonitor_GetState(&adc);
    ChassisSpeedLoop_Reset(speed_loop);
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        motor_id_t motor  = (motor_id_t)index;
        int16_t    output = 0;

        if (ChassisLayout_MotorEnabled(motor) != 0U)
        {
            output = ChassisOutputService_Clamp((int32_t)test->raw_forward[index] - (int32_t)test->raw_reverse[index]);
        }
        ChassisOutputService_SetMotorWithAdc(chassis, motor, output, &adc);
        chassis->motor_pid_active[index]    = 0U;
        chassis->motor_feedback_lost[index] = 0U;
        chassis->motor_error_mps[index]     = 0.0f;
    }
}
