#include "chassis_service.h"

#include "control_config.h"
#include "command_management_service.h"
#include "motion_control_service.h"
#include "motion_control_maintenance.h"
#include "parameter_management_service.h"
#include "power_management_service.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"

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
    motion_control_input_t      input = {0};
    motion_control_event_t      event;
    command_management_status_t command_status;
    safety_management_status_t  safety_status;
    state_estimation_status_t   state_status;

    input.now_ms                = now_ms;
    input.nominal_period_ms     = 10U;
    input.parameters.generation = ParameterManagement_GetSnapshot(&input.parameters.value);
    input.parameters.validity   = (input.parameters.generation != 0UL) ? 1UL : 0UL;
    (void)StateEstimation_GetStatus(now_ms, &state_status);
    input.wheel.value            = state_status.wheel;
    input.wheel.sample_time_ms   = state_status.wheel.last_update_ms;
    input.wheel.generation       = state_status.wheel_generation;
    input.wheel.validity         = (state_status.wheel_generation != 0UL) ? 1UL : 0UL;
    input.imu.value              = state_status.imu;
    input.imu.sample_time_ms     = state_status.imu.last_update_ms;
    input.imu.generation         = state_status.imu_sample_generation;
    input.imu.validity           = (state_status.imu_sample_generation != 0UL) ? 1UL : 0UL;
    input.power.generation       = PowerManagement_GetStatus(&input.power.value);
    input.power.validity         = (input.power.generation != 0UL) ? 1UL : 0UL;
    input.command.validity       = CommandManagement_GetActiveSnapshot(now_ms, &input.command.value, &command_status);
    input.command.sample_time_ms = input.command.value.timestamp_ms;
    input.command.generation     = command_status.generation;
    input.command.revoke_generation = command_status.motion_revoke_generation;
    (void)SafetyManagement_GetStatus(&safety_status);
    input.safety_permit.value          = safety_status.motion_permit;
    input.safety_permit.sample_time_ms = safety_status.motion_permit.issued_at_ms;
    input.safety_permit.generation     = safety_status.motion_permit.generation;
    input.safety_permit.validity       = (safety_status.motion_permit.generation != 0UL) ? 1UL : 0UL;
    input.normal_motion_allowed        = safety_status.motion_allowed;
    input.diagnostic_motion_allowed    = SafetyManagement_IsDiagnosticMotionAllowed();
    MotionControl_StepWithInput(&input, &event);
    if ((event.flags & MOTION_EVENT_DRIVER_FAULT) != 0UL)
    {
        SafetyManagement_SetFaultStop(1U);
    }
    if ((event.flags & MOTION_EVENT_ENCODER_FEEDBACK_LOST) != 0UL)
    {
        SafetyManagement_LatchEncoderFeedbackFault();
    }
    if ((event.flags & MOTION_EVENT_COMMAND_REVOKE) != 0UL)
    {
        CommandManagement_ClearAll();
    }
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
    param_model_t params;

    (void)ParameterManagement_GetSnapshot(&params);
    MotionControl_ResolveSideTargetsWithParameters(linear_x, angular_z, &params, left_mps, right_mps);
}

void ChassisService_GetState(chassis_service_snapshot_t *state)
{
    (void)MotionControl_GetStatus(state);
}
