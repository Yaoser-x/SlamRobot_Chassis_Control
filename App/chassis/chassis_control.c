#include "chassis_control.h"

#include "adc_monitor.h"
#include "chassis_config.h"
#include "chassis_layout.h"
#include "chassis_math.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "current_guard.h"
#include "encoder_driver.h"
#include "main.h"
#include "motor_driver.h"
#include "param_store.h"
#include "pid_controller.h"
#include "system_monitor.h"
#include "straight_controller.h"
#include "imu_bmi270.h"

static chassis_control_state_t chassis_state;
static uint8_t                 open_loop_test_enabled;
static uint8_t                 raw_input_test_enabled;
static int16_t                 open_loop_side[2];
static int16_t                 raw_forward[MOTOR_ID_COUNT];
static int16_t                 raw_reverse[MOTOR_ID_COUNT];
static float                   ramped_linear_x;
static float                   ramped_angular_z;
static float                   last_pid_target_mps[MOTOR_ID_COUNT];
static uint8_t                 feedback_loss_count[MOTOR_ID_COUNT];
static uint32_t                feedback_no_motion_since_ms[MOTOR_ID_COUNT];
static uint8_t                 feedback_no_motion_active[MOTOR_ID_COUNT];
static pid_state_t             pid_motor[MOTOR_ID_COUNT];
static uint32_t                last_control_step_ms;
static uint8_t                 control_dt_initialized;
static volatile uint8_t        control_step_active;
static uint32_t                test_mode_last_refresh_ms;
static uint8_t                 test_mode_lease_active;
static param_store_t           runtime_params;
static uint32_t                runtime_params_generation;
static straight_controller_t   straight_controller;

static int16_t ChassisControl_ClampPermille(int32_t permille)
{
    if (permille > CHASSIS_PWM_MAX_PERMILLE)
    {
        return CHASSIS_PWM_MAX_PERMILLE;
    }
    if (permille < -CHASSIS_PWM_MAX_PERMILLE)
    {
        return -CHASSIS_PWM_MAX_PERMILLE;
    }
    return (int16_t)permille;
}

static float ChassisControl_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int8_t ChassisControl_TargetSign(float value)
{
    if (value > CHASSIS_PID_DIRECTION_EPSILON_MPS)
    {
        return 1;
    }
    if (value < -CHASSIS_PID_DIRECTION_EPSILON_MPS)
    {
        return -1;
    }
    return 0;
}

static void ChassisControl_ResolveSideTargetsWithParams(float                linear_x,
                                                        float                angular_z,
                                                        const param_store_t *params,
                                                        float               *left_mps,
                                                        float               *right_mps)
{
    ChassisMath_ResolveDifferentialTargets(linear_x, angular_z, params->track_width_m, left_mps, right_mps);
}

void ChassisControl_ResolveSideTargets(float linear_x, float angular_z, float *left_mps, float *right_mps)
{
    param_store_t params;

    (void)ParamStore_GetSnapshot(&params);
    ChassisControl_ResolveSideTargetsWithParams(linear_x, angular_z, &params, left_mps, right_mps);
}

static float ChassisControl_RampToward(float current, float target, float step)
{
    if (current < target)
    {
        current += step;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= step;
        if (current < target)
        {
            current = target;
        }
    }
    return current;
}

static void ChassisControl_ResetRamps(void)
{
    ramped_linear_x  = 0.0f;
    ramped_angular_z = 0.0f;
    StraightController_Reset(&straight_controller);
    chassis_state.straight_active                 = 0U;
    chassis_state.straight_direction              = 0;
    chassis_state.straight_transition_distance_m  = 0.0f;
    chassis_state.straight_in_transition          = 0U;
    chassis_state.straight_trim_mps               = 0.0f;
    chassis_state.straight_wheel_correction_mps   = 0.0f;
    chassis_state.straight_heading_error_deg      = 0.0f;
    chassis_state.straight_heading_integral_deg_s = 0.0f;
    chassis_state.straight_heading_correction_mps = 0.0f;
    chassis_state.straight_total_correction_mps   = 0.0f;
    chassis_state.straight_heading_degraded       = 0U;
    chassis_state.straight_derated                = 0U;
    chassis_state.pwm_saturated                   = 0U;
    chassis_state.control_source                  = CONTROL_SOURCE_NONE;
}

static void ChassisControl_ResetPidTargets(void)
{
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        last_pid_target_mps[i]               = 0.0f;
        feedback_loss_count[i]               = 0U;
        feedback_no_motion_since_ms[i]       = 0UL;
        feedback_no_motion_active[i]         = 0U;
        chassis_state.motor_feedback_lost[i] = 0U;
    }
    chassis_state.left_feedback_lost  = 0U;
    chassis_state.right_feedback_lost = 0U;
}

static uint8_t ChassisControl_FeedbackFaultDetected(uint32_t                    now_ms,
                                                    const encoder_state_t      *encoder_state,
                                                    const motor_driver_state_t *motor_state)
{
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) == 0U)
        {
            feedback_no_motion_active[i] = 0U;
            continue;
        }
        if (encoder_state->speed_valid[i] == 0U)
        {
            chassis_state.motor_feedback_lost[i] = 1U;
            return 1U;
        }
        if (ChassisControl_AbsFloat(chassis_state.motor_requested_mps[i]) < CHASSIS_PID_FEEDBACK_MIN_TARGET_MPS
            || ChassisControl_AbsFloat(encoder_state->speed_mps[i]) >= CHASSIS_PID_FEEDBACK_MIN_SPEED_MPS)
        {
            feedback_no_motion_active[i] = 0U;
            continue;
        }
        if (motor_state->phase[i] != MOTOR_DRIVER_PHASE_RUN || motor_state->effective_pwm[i] == 0)
        {
            feedback_no_motion_active[i] = 0U;
            continue;
        }
        if (feedback_no_motion_active[i] == 0U)
        {
            feedback_no_motion_active[i]   = 1U;
            feedback_no_motion_since_ms[i] = now_ms;
        }
        else if ((uint32_t)(now_ms - feedback_no_motion_since_ms[i]) >= CHASSIS_ENCODER_FEEDBACK_TIMEOUT_MS)
        {
            chassis_state.motor_feedback_lost[i] = 1U;
            return 1U;
        }
    }
    return 0U;
}

static uint8_t ChassisControl_RefreshRuntimeParams(void)
{
    param_store_t params;
    uint32_t      generation = ParamStore_GetSnapshot(&params);

    if (generation == runtime_params_generation)
    {
        return 0U;
    }

    runtime_params            = params;
    runtime_params_generation = generation;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        pid_params_t pid_params = {
            params.pid_kp[i],
            params.pid_ki[i],
            params.pid_kd[i],
            params.pid_integral_limit,
            CHASSIS_PID_CORRECTION_LIMIT,
        };

        if (pid_motor[i].initialized == 0U)
        {
            PidController_Init(&pid_motor[i], &pid_params);
        }
        else
        {
            PidController_SetParams(&pid_motor[i], &pid_params);
        }
    }
    ChassisControl_ResetRamps();
    ChassisControl_ResetPidTargets();
    return 1U;
}

static void ChassisControl_ClearTestModeUnsafe(void)
{
    open_loop_test_enabled           = 0U;
    raw_input_test_enabled           = 0U;
    test_mode_lease_active           = 0U;
    open_loop_side[MOTOR_SIDE_LEFT]  = 0;
    open_loop_side[MOTOR_SIDE_RIGHT] = 0;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        raw_forward[i] = 0;
        raw_reverse[i] = 0;
    }
}

void ChassisControl_CancelTestMode(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    ChassisControl_ClearTestModeUnsafe();
    __set_PRIMASK(primask);
}

static void ChassisControl_ScaleWheelTargets(float *left_mps, float *right_mps)
{
    float abs_left;
    float abs_right;
    float max_abs;

    if (CHASSIS_WHEEL_SPEED_PROPORTIONAL_SCALE == 0U)
    {
        return;
    }
    if (left_mps == 0 || right_mps == 0)
    {
        return;
    }

    abs_left  = (*left_mps < 0.0f) ? -*left_mps : *left_mps;
    abs_right = (*right_mps < 0.0f) ? -*right_mps : *right_mps;
    max_abs   = (abs_left > abs_right) ? abs_left : abs_right;

    if (max_abs > CHASSIS_OPENLOOP_FULL_MPS)
    {
        float scale = CHASSIS_OPENLOOP_FULL_MPS / max_abs;
        *left_mps *= scale;
        *right_mps *= scale;
    }
}

static int16_t ChassisControl_MpsToPermille(float target_mps)
{
    int32_t permille;

    if (CHASSIS_OPENLOOP_FULL_MPS <= 0.0f)
    {
        return 0;
    }
    if (target_mps > CHASSIS_OPENLOOP_FULL_MPS)
    {
        target_mps = CHASSIS_OPENLOOP_FULL_MPS;
    }
    else if (target_mps < -CHASSIS_OPENLOOP_FULL_MPS)
    {
        target_mps = -CHASSIS_OPENLOOP_FULL_MPS;
    }

    permille = (int32_t)((target_mps / CHASSIS_OPENLOOP_FULL_MPS) * (float)CHASSIS_PWM_MAX_PERMILLE);
    return ChassisControl_ClampPermille(permille);
}

static int16_t ChassisControl_ApplyCurrentLimit(motor_id_t                 motor,
                                                int16_t                    permille,
                                                const adc_monitor_state_t *adc_state,
                                                uint8_t                   *limited)
{
    return ChassisControl_ClampPermille(CurrentGuard_ApplyMotorLimit(motor, permille, adc_state, 0U, limited));
}

static void ChassisControl_SetMotorOutput(motor_id_t motor, int16_t permille)
{
    adc_monitor_state_t adc_state;
    int16_t             applied;

    if (ChassisLayout_MotorEnabled(motor) == 0U)
    {
        chassis_state.motor_current_limited[motor] = 0U;
        chassis_state.motor_output_permille[motor] = 0;
        MotorDriver_SetPermille(motor, 0);
        return;
    }

    permille = ChassisControl_ClampPermille(permille);
    applied  = permille;

    AdcMonitor_GetState(&adc_state);
    chassis_state.motor_current_limited[motor] = 0U;
    applied = ChassisControl_ApplyCurrentLimit(motor, applied, &adc_state, &chassis_state.motor_current_limited[motor]);
    chassis_state.motor_output_permille[motor] = applied;
    MotorDriver_SetPermille(motor, applied);
}

static float ChassisControl_SelectSideValue(motor_id_t motor, float left_value, float right_value)
{
    return (ChassisLayout_MotorSide(motor) == MOTOR_SIDE_LEFT) ? left_value : right_value;
}

static uint8_t ChassisControl_AnyActiveMotorOutput(void)
{
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U && chassis_state.motor_output_permille[i] != 0)
        {
            return 1U;
        }
    }
    return 0U;
}

static void ChassisControl_SyncSideState(void)
{
    uint8_t left_count          = 0U;
    uint8_t right_count         = 0U;
    float   left_target_sum     = 0.0f;
    float   right_target_sum    = 0.0f;
    float   left_requested_sum  = 0.0f;
    float   right_requested_sum = 0.0f;
    float   left_actual_sum     = 0.0f;
    float   right_actual_sum    = 0.0f;
    float   left_error_sum      = 0.0f;
    float   right_error_sum     = 0.0f;
    int32_t left_output_sum     = 0;
    int32_t right_output_sum    = 0;

    chassis_state.left_speed_valid      = (ChassisLayout_SideMotorCount(MOTOR_SIDE_LEFT) != 0U) ? 1U : 0U;
    chassis_state.right_speed_valid     = (ChassisLayout_SideMotorCount(MOTOR_SIDE_RIGHT) != 0U) ? 1U : 0U;
    chassis_state.left_pid_active       = 0U;
    chassis_state.right_pid_active      = 0U;
    chassis_state.left_feedback_lost    = 0U;
    chassis_state.right_feedback_lost   = 0U;
    chassis_state.left_current_limited  = 0U;
    chassis_state.right_current_limited = 0U;

    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        motor_id_t motor = (motor_id_t)i;
        if (ChassisLayout_MotorEnabled(motor) == 0U)
        {
            continue;
        }

        if (ChassisLayout_MotorSide(motor) == MOTOR_SIDE_LEFT)
        {
            left_count++;
            left_target_sum += chassis_state.motor_target_mps[i];
            left_requested_sum += chassis_state.motor_requested_mps[i];
            left_actual_sum += chassis_state.motor_actual_mps[i];
            left_error_sum += chassis_state.motor_error_mps[i];
            left_output_sum += chassis_state.motor_output_permille[i];
            if (chassis_state.motor_speed_valid[i] == 0U)
            {
                chassis_state.left_speed_valid = 0U;
            }
            chassis_state.left_pid_active |= chassis_state.motor_pid_active[i];
            chassis_state.left_feedback_lost |= chassis_state.motor_feedback_lost[i];
            chassis_state.left_current_limited |= chassis_state.motor_current_limited[i];
        }
        else
        {
            right_count++;
            right_target_sum += chassis_state.motor_target_mps[i];
            right_requested_sum += chassis_state.motor_requested_mps[i];
            right_actual_sum += chassis_state.motor_actual_mps[i];
            right_error_sum += chassis_state.motor_error_mps[i];
            right_output_sum += chassis_state.motor_output_permille[i];
            if (chassis_state.motor_speed_valid[i] == 0U)
            {
                chassis_state.right_speed_valid = 0U;
            }
            chassis_state.right_pid_active |= chassis_state.motor_pid_active[i];
            chassis_state.right_feedback_lost |= chassis_state.motor_feedback_lost[i];
            chassis_state.right_current_limited |= chassis_state.motor_current_limited[i];
        }
    }

    chassis_state.left_target_mps       = (left_count != 0U) ? (left_target_sum / (float)left_count) : 0.0f;
    chassis_state.right_target_mps      = (right_count != 0U) ? (right_target_sum / (float)right_count) : 0.0f;
    chassis_state.left_requested_mps    = (left_count != 0U) ? (left_requested_sum / (float)left_count) : 0.0f;
    chassis_state.right_requested_mps   = (right_count != 0U) ? (right_requested_sum / (float)right_count) : 0.0f;
    chassis_state.left_actual_mps       = (left_count != 0U) ? (left_actual_sum / (float)left_count) : 0.0f;
    chassis_state.right_actual_mps      = (right_count != 0U) ? (right_actual_sum / (float)right_count) : 0.0f;
    chassis_state.left_error_mps        = (left_count != 0U) ? (left_error_sum / (float)left_count) : 0.0f;
    chassis_state.right_error_mps       = (right_count != 0U) ? (right_error_sum / (float)right_count) : 0.0f;
    chassis_state.left_output_permille  = (left_count != 0U) ? (int16_t)(left_output_sum / (int32_t)left_count) : 0;
    chassis_state.right_output_permille = (right_count != 0U) ? (int16_t)(right_output_sum / (int32_t)right_count) : 0;
}

static void ChassisControl_SetSideTargets(float left_mps, float right_mps, uint8_t requested)
{
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        motor_id_t motor  = (motor_id_t)i;
        float      target = 0.0f;

        if (ChassisLayout_MotorEnabled(motor) != 0U)
        {
            target = ChassisControl_SelectSideValue(motor, left_mps, right_mps);
        }
        chassis_state.motor_target_mps[i] = target;
        if (requested != 0U)
        {
            chassis_state.motor_requested_mps[i] = target;
        }
    }
}

static uint8_t
ChassisControl_CheckFeedbackUsable(motor_id_t motor, float target_mps, float actual_mps, uint8_t encoder_valid)
{
    chassis_state.motor_feedback_lost[motor] = 0U;
    if (encoder_valid == 0U)
    {
        feedback_loss_count[motor]               = CHASSIS_PID_FEEDBACK_LOSS_COUNT;
        chassis_state.motor_feedback_lost[motor] = 1U;
        return 0U;
    }
    if (ChassisControl_AbsFloat(target_mps) < CHASSIS_PID_FEEDBACK_MIN_TARGET_MPS
        || ChassisControl_AbsFloat(actual_mps) >= CHASSIS_PID_FEEDBACK_MIN_SPEED_MPS)
    {
        feedback_loss_count[motor] = 0U;
        return 1U;
    }
    if (feedback_loss_count[motor] < CHASSIS_PID_FEEDBACK_LOSS_COUNT)
    {
        feedback_loss_count[motor]++;
    }
    if (feedback_loss_count[motor] >= CHASSIS_PID_FEEDBACK_LOSS_COUNT)
    {
        chassis_state.motor_feedback_lost[motor] = 1U;
        return 0U;
    }
    return 1U;
}

static uint8_t ChassisControl_ShouldFreezePid(motor_driver_phase_t phase)
{
    return (phase == MOTOR_DRIVER_PHASE_RAMP_DOWN || phase == MOTOR_DRIVER_PHASE_REVERSE_BRAKE
            || phase == MOTOR_DRIVER_PHASE_PH_SETTLE)
               ? 1U
               : 0U;
}

static int16_t ChassisControl_StepMotorPid(motor_id_t motor,
                                           float      target_mps,
                                           float      actual_mps,
                                           uint8_t    speed_valid,
                                           float      dt_s,
                                           int8_t     actuator_limit_direction,
                                           int16_t    base_permille)
{
    int8_t last_sign;
    int8_t target_sign;
    float  pid_out;

    chassis_state.motor_pid_active[motor] = 0U;
    chassis_state.motor_error_mps[motor]  = 0.0f;

    if (ChassisControl_AbsFloat(target_mps) <= CHASSIS_PID_STOP_EPSILON_MPS || speed_valid == 0U)
    {
        PidController_Reset(&pid_motor[motor]);
        last_pid_target_mps[motor] = target_mps;
        return 0;
    }

    last_sign   = ChassisControl_TargetSign(last_pid_target_mps[motor]);
    target_sign = ChassisControl_TargetSign(target_mps);
    if (last_sign != 0 && target_sign != 0 && last_sign != target_sign)
    {
        PidController_Reset(&pid_motor[motor]);
    }
    last_pid_target_mps[motor] = target_mps;

    chassis_state.motor_error_mps[motor]  = target_mps - actual_mps;
    pid_out                               = PidController_StepBounded(&pid_motor[motor],
                                        target_mps,
                                        actual_mps,
                                        dt_s,
                                        actuator_limit_direction,
                                        (float)(-CHASSIS_PWM_MAX_PERMILLE - base_permille),
                                        (float)(CHASSIS_PWM_MAX_PERMILLE - base_permille));
    chassis_state.motor_pid_active[motor] = 1U;
    return ChassisControl_ClampPermille((int32_t)base_permille + (int32_t)pid_out);
}

static void ChassisControl_StopOutput(void)
{
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        chassis_state.motor_target_mps[i]    = 0.0f;
        chassis_state.motor_requested_mps[i] = 0.0f;
        chassis_state.motor_error_mps[i]     = 0.0f;
        chassis_state.motor_pid_active[i]    = 0U;
        chassis_state.motor_feedback_lost[i] = 0U;
        ChassisControl_SetMotorOutput((motor_id_t)i, 0);
    }
    ChassisControl_ResetRamps();
    ChassisControl_ResetPidTargets();
    chassis_state.output_enabled = ChassisControl_AnyActiveMotorOutput();
    ChassisControl_SyncSideState();
}

void ChassisControl_Init(void)
{
    MotorDriver_Init();
    MotorDriver_SetSpeedGetter(EncoderDriver_GetMotorSpeedMps);
    ControlManager_Init();
    CurrentGuard_Init();
    chassis_state             = (chassis_control_state_t){0};
    pid_motor[0]              = (pid_state_t){0};
    pid_motor[1]              = (pid_state_t){0};
    pid_motor[2]              = (pid_state_t){0};
    pid_motor[3]              = (pid_state_t){0};
    runtime_params_generation = 0UL;
    StraightController_Init(&straight_controller);
    last_control_step_ms   = 0U;
    control_dt_initialized = 0U;
    control_step_active    = 0U;
    open_loop_test_enabled = 0U;
    raw_input_test_enabled = 0U;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        raw_forward[i] = 0;
        raw_reverse[i] = 0;
    }
    open_loop_side[MOTOR_SIDE_LEFT]  = 0;
    open_loop_side[MOTOR_SIDE_RIGHT] = 0;
    test_mode_last_refresh_ms        = 0UL;
    test_mode_lease_active           = 0U;
    (void)ChassisControl_RefreshRuntimeParams();
    ChassisControl_ResetRamps();
    ChassisControl_ResetPidTargets();
    MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
}

static void ChassisControl_StepImpl(uint32_t now_ms)
{
    chassis_cmd_t   cmd;
    encoder_state_t encoder_state;
    uint8_t         valid_cmd;
    uint8_t         open_loop_active;
    uint8_t         raw_input_active;
    uint8_t         test_mode_expired = 0U;
    int16_t         open_loop_snapshot[2];
    int16_t         raw_forward_snapshot[MOTOR_ID_COUNT];
    int16_t         raw_reverse_snapshot[MOTOR_ID_COUNT];
    uint32_t        primask;
    float           dt_s;

    (void)ChassisControl_RefreshRuntimeParams();
    MotorDriver_UpdateFaults();
    if (MotorDriver_HasFault() != 0U)
    {
        ControlManager_SetFaultStop(1U);
    }
    EncoderDriver_GetState(&encoder_state);
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        chassis_state.motor_actual_mps[i]  = encoder_state.speed_mps[i];
        chassis_state.motor_speed_valid[i] = encoder_state.speed_valid[i];
    }
    ChassisControl_SyncSideState();

    if (ControlManager_IsMaintenanceLocked() != 0U || ControlManager_IsEmergencyStop() != 0U
        || ControlManager_IsFaultStop() != 0U)
    {
        ChassisControl_CancelTestMode();
        ChassisControl_EmergencyStop();
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (test_mode_lease_active != 0U && (uint32_t)(now_ms - test_mode_last_refresh_ms) > CHASSIS_TEST_MODE_LEASE_MS)
    {
        ChassisControl_ClearTestModeUnsafe();
        test_mode_expired = 1U;
    }
    open_loop_active                     = open_loop_test_enabled;
    raw_input_active                     = raw_input_test_enabled;
    open_loop_snapshot[MOTOR_SIDE_LEFT]  = open_loop_side[MOTOR_SIDE_LEFT];
    open_loop_snapshot[MOTOR_SIDE_RIGHT] = open_loop_side[MOTOR_SIDE_RIGHT];
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        raw_forward_snapshot[i] = raw_forward[i];
        raw_reverse_snapshot[i] = raw_reverse[i];
    }
    __set_PRIMASK(primask);
    if (test_mode_expired != 0U)
    {
        ControlManager_ClearCommand();
        ChassisControl_StopOutput();
        return;
    }

    valid_cmd = ControlManager_GetCommand(&cmd, now_ms);
    if (ChassisMath_ControlDt(now_ms, &last_control_step_ms, &control_dt_initialized, &dt_s) == 0U)
    {
        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            PidController_Reset(&pid_motor[i]);
        }
        ChassisControl_StopOutput();
        return;
    }

    {
        adc_monitor_state_t adc_state;

        AdcMonitor_GetState(&adc_state);
        if (adc_state.current_zero_valid == 0U)
        {
            ChassisControl_StopOutput();
            return;
        }
    }

    if (open_loop_active != 0U)
    {
        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            PidController_Reset(&pid_motor[i]);
            chassis_state.motor_pid_active[i]    = 0U;
            chassis_state.motor_feedback_lost[i] = 0U;
            chassis_state.motor_error_mps[i]     = 0.0f;
            if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U)
            {
                ChassisControl_SetMotorOutput(
                    (motor_id_t)i,
                    ChassisControl_SelectSideValue((motor_id_t)i,
                                                   (float)open_loop_snapshot[MOTOR_SIDE_LEFT],
                                                   (float)open_loop_snapshot[MOTOR_SIDE_RIGHT]));
            }
            else
            {
                ChassisControl_SetMotorOutput((motor_id_t)i, 0);
            }
        }
        chassis_state.output_enabled = ChassisControl_AnyActiveMotorOutput();
        ChassisControl_ResetRamps();
        ChassisControl_SetSideTargets(0.0f, 0.0f, 1U);
        ChassisControl_SyncSideState();
        return;
    }

    if (raw_input_active != 0U)
    {
        adc_monitor_state_t adc_state;

        AdcMonitor_GetState(&adc_state);
        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            PidController_Reset(&pid_motor[i]);
            chassis_state.motor_current_limited[i] = 0U;
            if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U)
            {
                int16_t target =
                    ChassisControl_ClampPermille((int32_t)raw_forward_snapshot[i] - (int32_t)raw_reverse_snapshot[i]);
                int16_t applied = target;
                applied         = ChassisControl_ApplyCurrentLimit((motor_id_t)i,
                                                           applied,
                                                           &adc_state,
                                                           &chassis_state.motor_current_limited[i]);
                MotorDriver_SetPermille((motor_id_t)i, applied);
                chassis_state.motor_output_permille[i] = applied;
            }
            else
            {
                MotorDriver_SetPermille((motor_id_t)i, 0);
                chassis_state.motor_output_permille[i] = 0;
            }
            chassis_state.motor_pid_active[i]    = 0U;
            chassis_state.motor_feedback_lost[i] = 0U;
            chassis_state.motor_error_mps[i]     = 0.0f;
        }
        ChassisControl_ResetRamps();
        ChassisControl_SetSideTargets(0.0f, 0.0f, 1U);
        chassis_state.output_enabled = ChassisControl_AnyActiveMotorOutput();
        ChassisControl_SyncSideState();
        return;
    }

    if (valid_cmd != 0U)
    {
        motor_driver_state_t motor_state;
        float                req_left;
        float                req_right;
        float                ramp_left;
        float                ramp_right;
        float                linear_step  = runtime_params.speed_ramp_mps2 * dt_s;
        float                angular_step = runtime_params.angular_ramp_rps2 * dt_s;

        if (ChassisLayout_HasBothSides() == 0U)
        {
            ControlManager_ClearCommand();
            ChassisControl_StopOutput();
            return;
        }

        ChassisControl_ResolveSideTargetsWithParams(cmd.linear_x,
                                                    cmd.angular_z,
                                                    &runtime_params,
                                                    &req_left,
                                                    &req_right);
        ChassisControl_ScaleWheelTargets(&req_left, &req_right);
        ChassisControl_SetSideTargets(req_left, req_right, 1U);

        ramped_linear_x  = ChassisControl_RampToward(ramped_linear_x, cmd.linear_x, linear_step);
        ramped_angular_z = ChassisControl_RampToward(ramped_angular_z, cmd.angular_z, angular_step);
        ChassisControl_ResolveSideTargetsWithParams(ramped_linear_x,
                                                    ramped_angular_z,
                                                    &runtime_params,
                                                    &ramp_left,
                                                    &ramp_right);
        if (ChassisControl_AbsFloat(cmd.angular_z) <= 0.0001f && ChassisControl_AbsFloat(cmd.linear_x) > 0.001f)
        {
            imu_bmi270_state_t           imu_state;
            uint8_t                      imu_valid;
            straight_controller_params_t straight_params;
            straight_controller_input_t  straight_input;
            straight_controller_result_t straight_result;

            ImuBmi270_GetState(&imu_state);
            imu_valid       = (imu_state.online != 0U && imu_state.gyro_calibrated != 0U
                         && (uint32_t)(now_ms - imu_state.last_update_ms) <= 100U
                         && (imu_state.quality_flags
                             & (IMU_BMI270_QUALITY_SPI_ERROR | IMU_BMI270_QUALITY_TIMESTAMP_ERROR
                                | IMU_BMI270_QUALITY_GYRO_SATURATION | IMU_BMI270_QUALITY_INIT_FAILED
                                | IMU_BMI270_QUALITY_PROFILE_MISMATCH))
                                == 0U)
                                  ? 1U
                                  : 0U;
            straight_params = (straight_controller_params_t){
                .trim_forward_015_mps         = runtime_params.straight_trim_forward_015_mps,
                .trim_forward_030_mps         = runtime_params.straight_trim_forward_030_mps,
                .trim_reverse_015_mps         = runtime_params.straight_trim_reverse_015_mps,
                .trim_reverse_030_mps         = runtime_params.straight_trim_reverse_030_mps,
                .wheel_coupling_gain          = runtime_params.straight_wheel_coupling_gain,
                .heading_kp                   = runtime_params.straight_heading_kp,
                .heading_ki                   = runtime_params.straight_heading_ki,
                .heading_integral_limit_deg_s = runtime_params.straight_heading_integral_limit_deg_s,
                .max_speed_mps                = runtime_params.straight_max_speed_mps,
                .heading_enabled              = runtime_params.straight_heading_hold_enabled,
            };
            straight_input = (straight_controller_input_t){
                .now_ms                = now_ms,
                .source                = cmd.source,
                .generation            = ControlManager_GetMotionRevokeGeneration(),
                .requested_linear_mps  = ramped_linear_x,
                .requested_angular_rps = cmd.angular_z,
                .actual_left_mps       = chassis_state.left_actual_mps,
                .actual_right_mps      = chassis_state.right_actual_mps,
                .left_speed_valid      = chassis_state.left_speed_valid,
                .right_speed_valid     = chassis_state.right_speed_valid,
                .left_output_permille  = chassis_state.left_output_permille,
                .right_output_permille = chassis_state.right_output_permille,
                .left_current_limited  = chassis_state.left_current_limited,
                .right_current_limited = chassis_state.right_current_limited,
                .imu_valid             = imu_valid,
                .gyro_z_dps            = imu_state.gyro_corrected_dps[2],
            };
            straight_result = StraightController_Step(&straight_controller, &straight_params, &straight_input);
            ramp_left       = straight_result.left_target_mps;
            ramp_right      = straight_result.right_target_mps;
            chassis_state.straight_active                 = straight_result.active;
            chassis_state.straight_direction              = straight_result.direction;
            chassis_state.straight_transition_distance_m  = straight_result.transition_distance_m;
            chassis_state.straight_in_transition          = straight_result.in_transition;
            chassis_state.straight_trim_mps               = straight_result.trim_correction_mps;
            chassis_state.straight_wheel_correction_mps   = straight_result.wheel_correction_mps;
            chassis_state.straight_heading_error_deg      = straight_result.heading_error_deg;
            chassis_state.straight_heading_integral_deg_s = straight_result.heading_integral_deg_s;
            chassis_state.straight_heading_correction_mps = straight_result.heading_correction_mps;
            chassis_state.straight_total_correction_mps   = straight_result.total_correction_mps;
            chassis_state.straight_heading_degraded       = straight_result.heading_degraded;
            chassis_state.straight_derated                = straight_result.derated;
            chassis_state.straight_out_of_range           = straight_result.out_of_range;
        }
        else
        {
            StraightController_Reset(&straight_controller);
            chassis_state.straight_active                 = 0U;
            chassis_state.straight_direction              = 0;
            chassis_state.straight_transition_distance_m  = 0.0f;
            chassis_state.straight_in_transition          = 0U;
            chassis_state.straight_trim_mps               = 0.0f;
            chassis_state.straight_wheel_correction_mps   = 0.0f;
            chassis_state.straight_heading_error_deg      = 0.0f;
            chassis_state.straight_heading_integral_deg_s = 0.0f;
            chassis_state.straight_heading_correction_mps = 0.0f;
            chassis_state.straight_total_correction_mps   = 0.0f;
            chassis_state.straight_heading_degraded       = 0U;
            chassis_state.straight_derated                = 0U;
            chassis_state.straight_out_of_range           = 0U;
        }
        chassis_state.control_source = cmd.source;
        chassis_state.pwm_saturated =
            (uint8_t)(ChassisControl_AbsFloat((float)chassis_state.left_output_permille) >= 850.0f
                      || ChassisControl_AbsFloat((float)chassis_state.right_output_permille) >= 850.0f);
        ChassisControl_ScaleWheelTargets(&ramp_left, &ramp_right);
        ChassisControl_SetSideTargets(ramp_left, ramp_right, 0U);

        MotorDriver_GetState(&motor_state);
        if (ChassisControl_FeedbackFaultDetected(now_ms, &encoder_state, &motor_state) != 0U)
        {
            SystemMonitor_LatchEncoderFeedbackFault();
            ChassisControl_EmergencyStop();
            return;
        }

        if (req_left == 0.0f && req_right == 0.0f)
        {
            for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
            {
                PidController_Reset(&pid_motor[i]);
            }
            ChassisControl_ResetPidTargets();
            for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
            {
                ChassisControl_SetMotorOutput((motor_id_t)i, 0);
            }
            chassis_state.output_enabled = ChassisControl_AnyActiveMotorOutput();
            ChassisControl_SyncSideState();
            return;
        }

        for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
        {
            int16_t permille;
            int16_t base_permille;
            int8_t  actuator_limit_direction = 0;
            if (ChassisLayout_MotorEnabled((motor_id_t)i) == 0U)
            {
                PidController_Reset(&pid_motor[i]);
                chassis_state.motor_pid_active[i]    = 0U;
                chassis_state.motor_feedback_lost[i] = 0U;
                chassis_state.motor_error_mps[i]     = 0.0f;
                ChassisControl_SetMotorOutput((motor_id_t)i, 0);
                continue;
            }
            base_permille = ChassisControl_MpsToPermille(chassis_state.motor_target_mps[i]);
            if (chassis_state.motor_current_limited[i] != 0U
                || ChassisControl_AbsFloat((float)motor_state.applied_pwm[i])
                       < ChassisControl_AbsFloat((float)motor_state.requested_pwm[i]))
            {
                actuator_limit_direction = ChassisControl_TargetSign(chassis_state.motor_target_mps[i]);
            }
            if (CHASSIS_PID_ENABLED != 0U)
            {
                uint8_t feedback_usable            = ChassisControl_CheckFeedbackUsable((motor_id_t)i,
                                                                             chassis_state.motor_target_mps[i],
                                                                             chassis_state.motor_actual_mps[i],
                                                                             encoder_state.speed_valid[i]);
                chassis_state.motor_speed_valid[i] = feedback_usable;
                if (ChassisControl_ShouldFreezePid(motor_state.phase[i]) != 0U)
                {
                    chassis_state.motor_pid_active[i] = 0U;
                    chassis_state.motor_error_mps[i]  = 0.0f;
                    permille                          = base_permille;
                }
                else
                {
                    permille = ChassisControl_StepMotorPid((motor_id_t)i,
                                                           chassis_state.motor_target_mps[i],
                                                           chassis_state.motor_actual_mps[i],
                                                           feedback_usable,
                                                           dt_s,
                                                           actuator_limit_direction,
                                                           base_permille);
                }
            }
            else
            {
                chassis_state.motor_pid_active[i]    = 0U;
                chassis_state.motor_feedback_lost[i] = 0U;
                chassis_state.motor_error_mps[i]     = 0.0f;
                permille                             = base_permille;
            }
            ChassisControl_SetMotorOutput((motor_id_t)i, permille);
        }
        chassis_state.output_enabled = 1U;
        ChassisControl_SyncSideState();
        chassis_state.pwm_saturated =
            (uint8_t)(ChassisControl_AbsFloat((float)chassis_state.left_output_permille) >= 850.0f
                      || ChassisControl_AbsFloat((float)chassis_state.right_output_permille) >= 850.0f);
    }
    else
    {
        ChassisControl_StopOutput();
    }
}

void ChassisControl_Step(uint32_t now_ms)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    control_step_active = 1U;
    __set_PRIMASK(primask);
    ChassisControl_StepImpl(now_ms);
    primask = __get_PRIMASK();
    __disable_irq();
    control_step_active = 0U;
    __set_PRIMASK(primask);
}

uint8_t ChassisControl_IsStepActive(void)
{
    uint8_t  active;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    active = control_step_active;
    __set_PRIMASK(primask);
    return active;
}

void ChassisControl_EmergencyStop(void)
{
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        PidController_Reset(&pid_motor[i]);
    }
    ChassisControl_CancelTestMode();
    ChassisControl_ResetRamps();
    ChassisControl_ResetPidTargets();
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        chassis_state.motor_target_mps[i]      = 0.0f;
        chassis_state.motor_requested_mps[i]   = 0.0f;
        chassis_state.motor_error_mps[i]       = 0.0f;
        chassis_state.motor_output_permille[i] = 0;
        chassis_state.motor_current_limited[i] = 0U;
        chassis_state.motor_pid_active[i]      = 0U;
        chassis_state.motor_feedback_lost[i]   = 0U;
    }
    chassis_state.output_enabled = 0U;
    MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
    ChassisControl_SyncSideState();
}

void ChassisControl_OpenLoopTest(int16_t left_permille, int16_t right_permille)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    open_loop_side[MOTOR_SIDE_LEFT]  = ChassisControl_ClampPermille(left_permille);
    open_loop_side[MOTOR_SIDE_RIGHT] = ChassisControl_ClampPermille(right_permille);
    open_loop_test_enabled           = ((left_permille != 0) || (right_permille != 0)) ? 1U : 0U;
    raw_input_test_enabled           = 0U;
    test_mode_lease_active           = open_loop_test_enabled;
    test_mode_last_refresh_ms        = osKernelGetTickCount();
    __set_PRIMASK(primask);
}

void ChassisControl_RawInputTest(int16_t left_forward_permille,
                                 int16_t left_reverse_permille,
                                 int16_t right_forward_permille,
                                 int16_t right_reverse_permille)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U
            && ChassisLayout_MotorSide((motor_id_t)i) == MOTOR_SIDE_LEFT)
        {
            raw_forward[i] = ChassisControl_ClampPermille(left_forward_permille);
            raw_reverse[i] = ChassisControl_ClampPermille(left_reverse_permille);
        }
        else if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U)
        {
            raw_forward[i] = ChassisControl_ClampPermille(right_forward_permille);
            raw_reverse[i] = ChassisControl_ClampPermille(right_reverse_permille);
        }
        else
        {
            raw_forward[i] = 0;
            raw_reverse[i] = 0;
        }
        if (raw_forward[i] < 0)
        {
            raw_forward[i] = 0;
        }
        if (raw_reverse[i] < 0)
        {
            raw_reverse[i] = 0;
        }
    }
    raw_input_test_enabled = 0U;
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (raw_forward[i] != 0 || raw_reverse[i] != 0)
        {
            raw_input_test_enabled = 1U;
        }
    }
    open_loop_test_enabled    = 0U;
    test_mode_lease_active    = raw_input_test_enabled;
    test_mode_last_refresh_ms = osKernelGetTickCount();
    __set_PRIMASK(primask);
}

void ChassisControl_RawMotorInputTest(motor_id_t motor, int16_t forward_permille, int16_t reverse_permille)
{
    uint32_t primask;

    if ((uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        raw_forward[i] = 0;
        raw_reverse[i] = 0;
    }
    if (ChassisLayout_MotorEnabled(motor) == 0U)
    {
        raw_input_test_enabled = 0U;
        open_loop_test_enabled = 0U;
        test_mode_lease_active = 0U;
        __set_PRIMASK(primask);
        return;
    }
    raw_forward[motor] = ChassisControl_ClampPermille(forward_permille);
    raw_reverse[motor] = ChassisControl_ClampPermille(reverse_permille);
    if (raw_forward[motor] < 0)
    {
        raw_forward[motor] = 0;
    }
    if (raw_reverse[motor] < 0)
    {
        raw_reverse[motor] = 0;
    }
    raw_input_test_enabled    = ((raw_forward[motor] != 0) || (raw_reverse[motor] != 0)) ? 1U : 0U;
    open_loop_test_enabled    = 0U;
    test_mode_lease_active    = raw_input_test_enabled;
    test_mode_last_refresh_ms = osKernelGetTickCount();
    __set_PRIMASK(primask);
}

void ChassisControl_GetState(chassis_control_state_t *state)
{
    if (state != 0)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        *state = chassis_state;
        __set_PRIMASK(primask);
    }
}
