#include "motor_driver.h"
#include "motor_hw_map.h"

#include "chassis_layout.h"
#include "chassis_layout_config.h"
#include "cmsis_os2.h"
#include "main.h"
#include "direction_apply.h"
#include "motor_config.h"
#include "motor_reversal_state.h"
#include "tim.h"

#define MOTOR_PWM_RISE_STEP_PER_CYCLE 15
#define MOTOR_PWM_FALL_STEP_PER_CYCLE 25
#define MOTOR_REVERSE_BRAKE_CYCLES    2U

static const motor_reversal_config_t motor_reversal_config = {
    .rise_step_per_cycle         = MOTOR_PWM_RISE_STEP_PER_CYCLE,
    .fall_step_per_cycle         = MOTOR_PWM_FALL_STEP_PER_CYCLE,
    .fixed_brake_cycles          = MOTOR_REVERSE_BRAKE_CYCLES,
    .feedback_brake_cycles       = MOTOR_REVERSE_MAX_BRAKE_CYCLES,
    .reverse_speed_threshold_mps = MOTOR_REVERSE_SPEED_THRESHOLD_MPS,
};

static motor_speed_getter_t g_speed_getter;
static int8_t               motor_direction[MOTOR_ID_COUNT] = {
    CHASSIS_M1_MOTOR_DIR,
    CHASSIS_M2_MOTOR_DIR,
    CHASSIS_M3_MOTOR_DIR,
    CHASSIS_M4_MOTOR_DIR,
};

void MotorDriver_SetSpeedGetter(motor_speed_getter_t getter)
{
    g_speed_getter = getter;
}

void MotorDriver_SetDirectionConfig(const int8_t direction[MOTOR_ID_COUNT])
{
    if (direction == 0)
    {
        return;
    }
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        motor_direction[i] = (direction[i] < 0) ? -1 : 1;
    }
}

static motor_reversal_state_t motor_runtime[MOTOR_ID_COUNT];
static motor_driver_state_t   motor_state;

static void MotorDriver_UpdateEffectivePwmAll(void);

static void MotorDriver_SyncBreakSnapshot(void)
{
    motor_break_snapshot_t snapshot;

    MotorBreak_GetSnapshot(&snapshot);
    motor_state.tim1_moe_active          = snapshot.tim1_moe_active;
    motor_state.tim1_break_flag          = snapshot.tim1_break_flag;
    motor_state.tim1_break_latched       = snapshot.tim1_break_latched;
    motor_state.tim8_moe_active          = snapshot.tim8_moe_active;
    motor_state.tim8_break_flag          = snapshot.tim8_break_flag;
    motor_state.tim1_break_count         = snapshot.tim1_break_count;
    motor_state.tim8_break_count         = snapshot.tim8_break_count;
    motor_state.tim1_break_last_ms       = snapshot.tim1_break_last_ms;
    motor_state.tim8_break_last_ms       = snapshot.tim8_break_last_ms;
    motor_state.startup_qualified        = snapshot.startup_qualified;
    motor_state.startup_pre_wake_bif     = snapshot.startup_pre_wake_bif;
    motor_state.startup_bkin_high        = snapshot.startup_bkin_high;
    motor_state.startup_nfault_high_mask = snapshot.startup_nfault_high_mask;
    motor_state.break_origin             = snapshot.break_origin;
}

static void MotorDriver_ClearRuntimeAfterBreak(void)
{
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        MotorReversalState_ClearOutput(&motor_runtime[i]);
        motor_state.requested_pwm[i]   = 0;
        motor_state.applied_pwm[i]     = 0;
        motor_state.output_permille[i] = 0;
        motor_state.effective_pwm[i]   = 0;
        motor_state.pending_dir[i]     = 0;
        motor_state.phase[i]           = MOTOR_DRIVER_PHASE_IDLE_BRAKE;
    }
    MotorDriver_SyncBreakSnapshot();
}

void MotorDriver_OnTim1BreakFromIsr(void)
{
    MotorBreak_OnTim1Runtime(osKernelGetTickCount());
    MotorDriver_ClearRuntimeAfterBreak();
}

static uint8_t MotorDriver_ReadNfaultHighMask(void)
{
    uint8_t mask = 0U;

    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        const motor_hw_t *hw = MotorHwMap_Get((motor_id_t)i);
        if (HAL_GPIO_ReadPin(hw->fault_port, hw->fault_pin) == GPIO_PIN_SET)
        {
            mask |= (uint8_t)(1U << i);
        }
    }
    return mask;
}

static uint8_t MotorDriver_StartupInputsHigh(uint8_t *nfault_high_mask)
{
    uint8_t required_mask = 0U;
    uint8_t high_mask     = MotorDriver_ReadNfaultHighMask();

    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U)
        {
            required_mask |= (uint8_t)(1U << i);
        }
    }
    if (nfault_high_mask != 0)
    {
        *nfault_high_mask = high_mask;
    }
    return (MotorBreak_IsBkinHigh() != 0U && (high_mask & required_mask) == required_mask) ? 1U : 0U;
}

static void MotorDriver_UpdateBreakStatus(void)
{
    uint32_t primask;

    if (MotorBreak_Update(osKernelGetTickCount()) != 0U)
    {
        MotorDriver_ClearRuntimeAfterBreak();
    }

    primask = __get_PRIMASK();
    __disable_irq();
    MotorDriver_SyncBreakSnapshot();
    MotorDriver_UpdateEffectivePwmAll();
    __set_PRIMASK(primask);
}

static uint8_t MotorDriver_IsValidMotor(motor_id_t motor)
{
    return ((uint32_t)motor < MOTOR_ID_COUNT) ? 1U : 0U;
}

static int16_t MotorDriver_ComputeEffectivePwm(motor_id_t motor)
{
    uint32_t index = (uint32_t)motor;

    if (MotorDriver_IsValidMotor(motor) == 0U || ChassisLayout_MotorEnabled(motor) == 0U
        || motor_state.fault_active[index] != 0U || motor_state.tim1_moe_active == 0U
        || motor_state.tim1_break_latched != 0U || motor_state.tim1_break_flag != 0U)
    {
        return 0;
    }
    return motor_runtime[index].applied_pwm;
}

static void MotorDriver_UpdateEffectivePwmAll(void)
{
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        motor_state.effective_pwm[i] = MotorDriver_ComputeEffectivePwm((motor_id_t)i);
    }
}

static int16_t MotorDriver_ClampPositivePermille(int16_t permille)
{
    if (permille < 0)
    {
        return 0;
    }
    if (permille > CHASSIS_PWM_MAX_PERMILLE)
    {
        return CHASSIS_PWM_MAX_PERMILLE;
    }
    return permille;
}

static int16_t MotorDriver_ClampSignedPermille(int32_t permille)
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

static uint32_t MotorDriver_PulseFromPermille(TIM_HandleTypeDef *htim, int16_t permille)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);

    permille = MotorDriver_ClampPositivePermille(permille);
    if (permille > 0 && permille < CHASSIS_PWM_DEADBAND_PERMILLE)
    {
        permille = CHASSIS_PWM_DEADBAND_PERMILLE;
    }
    return ((arr + 1U) * (uint32_t)permille) / 1000U;
}

static int16_t MotorDriver_AbsSigned(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static void MotorDriver_SetEnPulse(const motor_hw_t *motor, uint32_t in1_pulse)
{
    __HAL_TIM_SET_COMPARE(motor->in1_htim, motor->in1_channel, in1_pulse);
}

static void MotorDriver_WritePhase(const motor_hw_t *motor, int8_t ph_dir)
{
    GPIO_PinState phase_state = (ph_dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;

    HAL_GPIO_WritePin(motor->phase_port, motor->phase_pin, phase_state);
}

static void MotorDriver_SetRaw(const motor_hw_t *motor, uint32_t in1_pulse, int8_t ph_dir)
{
    MotorDriver_SetEnPulse(motor, in1_pulse);
    MotorDriver_WritePhase(motor, ph_dir);
}

static void MotorDriver_RecordRuntime(motor_id_t motor)
{
    uint32_t                      primask;
    const motor_reversal_state_t *runtime;

    if (MotorDriver_IsValidMotor(motor) == 0U)
    {
        return;
    }

    runtime = &motor_runtime[(uint32_t)motor];
    primask = __get_PRIMASK();
    __disable_irq();
    motor_state.output_permille[(uint32_t)motor] = runtime->applied_pwm;
    motor_state.requested_pwm[(uint32_t)motor]   = runtime->requested_pwm;
    motor_state.applied_pwm[(uint32_t)motor]     = runtime->applied_pwm;
    motor_state.effective_pwm[(uint32_t)motor]   = MotorDriver_ComputeEffectivePwm(motor);
    motor_state.current_ph_dir[(uint32_t)motor]  = runtime->current_ph_dir;
    motor_state.pending_dir[(uint32_t)motor]     = runtime->pending_dir;
    motor_state.phase[(uint32_t)motor]           = runtime->phase;
    __set_PRIMASK(primask);
}

static void MotorDriver_ApplyRuntimeOutput(motor_id_t motor)
{
    const motor_hw_t             *hw      = MotorHwMap_Get(motor);
    const motor_reversal_state_t *runtime = &motor_runtime[(uint32_t)motor];
    uint32_t pulse = MotorDriver_PulseFromPermille(hw->in1_htim, MotorDriver_AbsSigned(runtime->applied_pwm));

    MotorDriver_SetEnPulse(hw, pulse);
    MotorDriver_RecordRuntime(motor);
}

static void MotorDriver_ClearRuntimeOutput(motor_id_t motor)
{
    const motor_hw_t *hw = MotorHwMap_Get(motor);

    MotorReversalState_ClearOutput(&motor_runtime[(uint32_t)motor]);
    MotorDriver_SetEnPulse(hw, 0U);
    MotorDriver_RecordRuntime(motor);
}

static void MotorDriver_DisableRuntimeOutput(motor_id_t motor)
{
    const motor_hw_t *hw = MotorHwMap_Get(motor);

    MotorReversalState_Disable(&motor_runtime[(uint32_t)motor]);
    MotorDriver_SetRaw(hw, 0U, -1);
    MotorDriver_RecordRuntime(motor);
}

static void MotorDriver_StartPwm(const motor_hw_t *motor)
{
    (void)HAL_TIM_PWM_Start(motor->in1_htim, motor->in1_channel);
}

void MotorDriver_Init(void)
{
    uint8_t startup_qualified = 1U;

    motor_direction[MOTOR_ID_M1] = CHASSIS_M1_MOTOR_DIR;
    motor_direction[MOTOR_ID_M2] = CHASSIS_M2_MOTOR_DIR;
    motor_direction[MOTOR_ID_M3] = CHASSIS_M3_MOTOR_DIR;
    motor_direction[MOTOR_ID_M4] = CHASSIS_M4_MOTOR_DIR;
    uint8_t  nfault_high_mask    = 0U;
    uint32_t stable_high_ms      = 0U;

    MotorBreak_Init();
    HAL_GPIO_WritePin(DRV_SLEEP_ALL_GPIO_Port, DRV_SLEEP_ALL_Pin, GPIO_PIN_SET);
    HAL_Delay(DRV8874_WAKE_DELAY_MS);
    motor_state               = (motor_driver_state_t){0};
    motor_state.sleep_enabled = 1U;
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        MotorReversalState_Init(&motor_runtime[i]);
        const motor_hw_t *hw = MotorHwMap_Get((motor_id_t)i);
        MotorDriver_StartPwm(hw);
        MotorDriver_SetRaw(hw, 0U, -1);
        MotorDriver_RecordRuntime((motor_id_t)i);
    }
    startup_qualified = 0U;
    for (uint32_t elapsed_ms = 0U; elapsed_ms < DRV8874_STARTUP_TIMEOUT_MS; ++elapsed_ms)
    {
        if (MotorDriver_StartupInputsHigh(&nfault_high_mask) != 0U)
        {
            stable_high_ms++;
            if (stable_high_ms >= DRV8874_STARTUP_STABLE_MS)
            {
                startup_qualified = 1U;
                break;
            }
        }
        else
        {
            stable_high_ms = 0U;
        }
        HAL_Delay(1U);
    }
    /* 清除 PWM 启动期间可能由 HAL_TIM_PWM_Start 触发的 TIM1/TIM8 break 标志，
     避免因 BKIN 脚在 DRV 唤醒过程中短暂低电平导致的误锁存。 */
    if (MotorBreak_CompleteStartup(startup_qualified, nfault_high_mask, osKernelGetTickCount()) != 0U)
    {
        MotorDriver_ClearRuntimeAfterBreak();
    }
    else
    {
        MotorDriver_SyncBreakSnapshot();
    }
    MotorDriver_UpdateFaults();
}

void MotorDriver_SetPermille(motor_id_t motor, int16_t permille)
{
    motor_reversal_state_t *runtime;
    motor_reversal_input_t  input = {0};
    motor_reversal_output_t output;

    if (MotorDriver_IsValidMotor(motor) == 0U)
    {
        return;
    }

    runtime = &motor_runtime[(uint32_t)motor];
    if (motor_state.tim1_break_latched != 0U)
    {
        MotorDriver_ClearRuntimeOutput(motor);
        return;
    }
    if (ChassisLayout_MotorEnabled(motor) == 0U)
    {
        MotorDriver_DisableRuntimeOutput(motor);
        return;
    }

    input.requested_pwm =
        MotorDriver_ClampSignedPermille(DirectionApply_Signed((int32_t)permille, motor_direction[(uint32_t)motor]));
    input.speed_feedback_available = (g_speed_getter != 0) ? 1U : 0U;
    if (runtime->phase == MOTOR_DRIVER_PHASE_REVERSE_BRAKE && g_speed_getter != 0)
    {
        input.speed_mps = g_speed_getter(motor);
    }
    output = MotorReversalState_Step(runtime, &motor_reversal_config, &input);
    if (output.phase_changed != 0U)
    {
        const motor_hw_t *hw = MotorHwMap_Get(motor);
        MotorDriver_SetEnPulse(hw, 0U);
        MotorDriver_WritePhase(hw, output.current_ph_dir);
    }
    MotorDriver_ApplyRuntimeOutput(motor);
}

void MotorDriver_SetSidePermille(motor_side_t side, int16_t permille)
{
    if (side == MOTOR_SIDE_LEFT)
    {
        MotorDriver_SetPermille(MOTOR_ID_M1, permille);
        MotorDriver_SetPermille(MOTOR_ID_M2, permille);
    }
    else
    {
        MotorDriver_SetPermille(MOTOR_ID_M3, permille);
        MotorDriver_SetPermille(MOTOR_ID_M4, permille);
    }
}

void MotorDriver_Stop(motor_id_t motor, motor_stop_mode_t mode)
{
    if (MotorDriver_IsValidMotor(motor) == 0U)
    {
        return;
    }

    (void)mode;
    MotorDriver_ClearRuntimeOutput(motor);
}

void MotorDriver_StopSide(motor_side_t side, motor_stop_mode_t mode)
{
    if (side == MOTOR_SIDE_LEFT)
    {
        MotorDriver_Stop(MOTOR_ID_M1, mode);
        MotorDriver_Stop(MOTOR_ID_M2, mode);
    }
    else
    {
        MotorDriver_Stop(MOTOR_ID_M3, mode);
        MotorDriver_Stop(MOTOR_ID_M4, mode);
    }
}

void MotorDriver_StopAll(motor_stop_mode_t mode)
{
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        MotorDriver_Stop((motor_id_t)i, mode);
    }
}

void MotorDriver_UpdateFaults(void)
{
    uint8_t  fault_active[MOTOR_ID_COUNT];
    uint32_t now_ms = osKernelGetTickCount();
    uint32_t primask;

    MotorDriver_UpdateBreakStatus();
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U)
        {
            const motor_hw_t *hw = MotorHwMap_Get((motor_id_t)i);
            fault_active[i]      = (HAL_GPIO_ReadPin(hw->fault_port, hw->fault_pin) == GPIO_PIN_RESET) ? 1U : 0U;
        }
        else
        {
            fault_active[i] = 0U;
        }
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        uint8_t previous            = motor_state.fault_active[i];
        motor_state.fault_active[i] = fault_active[i];
        if (previous != fault_active[i])
        {
            motor_state.fault_edge_count[i]++;
            motor_state.fault_last_change_ms[i] = now_ms;
            motor_state.fault_low_since_ms[i]   = (fault_active[i] != 0U) ? now_ms : 0U;
        }
        else if (fault_active[i] == 0U)
        {
            motor_state.fault_low_since_ms[i] = 0U;
        }
    }
    MotorDriver_UpdateEffectivePwmAll();
    __set_PRIMASK(primask);
}

uint8_t MotorDriver_HasFault(void)
{
    MotorDriver_UpdateFaults();
    if (motor_state.tim1_break_latched != 0U)
    {
        return 1U;
    }
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (motor_state.fault_active[i] != 0U)
        {
            return 1U;
        }
    }
    return 0U;
}

uint8_t MotorDriver_ClearBreakLatch(void)
{
    uint32_t primask;
    uint8_t  external_safe = 1U;
    uint8_t  cleared;

    MotorDriver_UpdateFaults();
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U && motor_state.fault_active[i] != 0U)
        {
            external_safe = 0U;
        }
        if (motor_state.effective_pwm[i] != 0 || motor_runtime[i].applied_pwm != 0)
        {
            external_safe = 0U;
        }
    }
    primask = __get_PRIMASK();
    __disable_irq();
    cleared = MotorBreak_ClearLatch(external_safe);
    MotorDriver_SyncBreakSnapshot();
    MotorDriver_UpdateEffectivePwmAll();
    __set_PRIMASK(primask);
    return cleared;
}

void MotorDriver_GetState(motor_driver_state_t *state)
{
    uint32_t primask;

    if (state != 0)
    {
        MotorDriver_UpdateFaults();
        primask = __get_PRIMASK();
        __disable_irq();
        *state = motor_state;
        __set_PRIMASK(primask);
    }
}
