#include "chassis_service.h"

#include "control_config.h"
#include "motion_control_service.h"

void ChassisService_Init(void)
{
    const motion_control_config_t config = {
        .max_linear_mps                 = CHASSIS_MAX_LINEAR_MPS,
        .max_angular_rps                = CHASSIS_MAX_ANGULAR_RPS,
        .open_loop_full_mps             = CHASSIS_OPENLOOP_FULL_MPS,
        .angular_epsilon_rps            = CHASSIS_ANGULAR_EPSILON_RPS,
        .speed_ramp_mps2                = CHASSIS_SPEED_RAMP_MPS2,
        .angular_ramp_rps2              = CHASSIS_ANGULAR_RAMP_RPS2,
        .maintenance_max_speed_mps      = CHASSIS_MAINTENANCE_MAX_SPEED_MPS,
        .pid_correction_limit           = CHASSIS_PID_CORRECTION_LIMIT,
        .pid_stop_epsilon_mps           = CHASSIS_PID_STOP_EPSILON_MPS,
        .pid_direction_epsilon_mps      = CHASSIS_PID_DIRECTION_EPSILON_MPS,
        .pid_feedback_min_target_mps    = CHASSIS_PID_FEEDBACK_MIN_TARGET_MPS,
        .pid_feedback_min_speed_mps     = CHASSIS_PID_FEEDBACK_MIN_SPEED_MPS,
        .test_mode_lease_ms             = CHASSIS_TEST_MODE_LEASE_MS,
        .encoder_feedback_timeout_ms    = CHASSIS_ENCODER_FEEDBACK_TIMEOUT_MS,
        .pid_feedback_loss_count        = CHASSIS_PID_FEEDBACK_LOSS_COUNT,
        .pid_enabled                    = CHASSIS_PID_ENABLED,
        .wheel_speed_proportional_scale = CHASSIS_WHEEL_SPEED_PROPORTIONAL_SCALE,
    };

    (void)MotionControl_Init(&config);
}

void ChassisService_Step(uint32_t now_ms)
{
    MotionControl_Step(now_ms);
}

uint8_t ChassisService_IsStepActive(void)
{
    return MotionControl_IsStepActive();
}

void ChassisService_EmergencyStop(void)
{
    MotionControl_EmergencyStop();
}

void ChassisService_CancelTestMode(void)
{
    MotionControl_CancelTestMode();
}

void ChassisService_OpenLoopTest(int16_t left_permille, int16_t right_permille)
{
    MotionControl_OpenLoopTest(left_permille, right_permille);
}

void ChassisService_RawInputTest(int16_t left_forward_permille,
                                 int16_t left_reverse_permille,
                                 int16_t right_forward_permille,
                                 int16_t right_reverse_permille)
{
    MotionControl_RawInputTest(left_forward_permille,
                               left_reverse_permille,
                               right_forward_permille,
                               right_reverse_permille);
}

void ChassisService_RawMotorInputTest(motor_id_t motor, int16_t forward_permille, int16_t reverse_permille)
{
    MotionControl_RawMotorInputTest((uint8_t)motor, forward_permille, reverse_permille);
}

void ChassisService_ResolveSideTargets(float linear_x, float angular_z, float *left_mps, float *right_mps)
{
    MotionControl_ResolveSideTargets(linear_x, angular_z, left_mps, right_mps);
}

void ChassisService_GetState(chassis_service_snapshot_t *state)
{
    (void)MotionControl_GetStatus(state);
}
