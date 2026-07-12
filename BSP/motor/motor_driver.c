#include "motor_driver.h"

#include "bsp_config.h"
#include "chassis_layout.h"
#include "cmsis_os2.h"
#include "main.h"
#include "param_store.h"
#include "direction_apply.h"
#include "tim.h"

typedef struct
{
    TIM_HandleTypeDef *in1_htim;
    uint32_t           in1_channel;
    GPIO_TypeDef      *phase_port;
    uint16_t           phase_pin;
    GPIO_TypeDef      *fault_port;
    uint16_t           fault_pin;
} motor_hw_t;

typedef struct
{
    int16_t              requested_pwm;
    int16_t              applied_pwm;
    int8_t               current_ph_dir;
    int8_t               pending_dir;
    motor_driver_phase_t phase;
    uint8_t              wait_cycles;
    uint8_t              phase_initialized;
} motor_runtime_t;

#define MOTOR_PWM_RISE_STEP_PER_CYCLE 15
#define MOTOR_PWM_FALL_STEP_PER_CYCLE 25
#define MOTOR_REVERSE_BRAKE_CYCLES    2U
#define MOTOR_PHASE_SWITCH_GAP_CALLS  MOTOR_ID_COUNT

static motor_speed_getter_t g_speed_getter;

void MotorDriver_SetSpeedGetter(motor_speed_getter_t getter)
{
    g_speed_getter = getter;
}

/* CubeMX labels keep legacy M2/M3 names; logical M2/M3 nFAULT pins are crossed here. */
static const motor_hw_t motor_hw[MOTOR_ID_COUNT] = {
    {&htim1, TIM_CHANNEL_1, M1_IN2_GPIO_Port, M1_IN2_Pin, M1_FAULT_GPIO_Port, M1_FAULT_Pin},
    {&htim1, TIM_CHANNEL_2, M2_IN2_GPIO_Port, M2_IN2_Pin, M3_FAULT_GPIO_Port, M3_FAULT_Pin},
    {&htim1, TIM_CHANNEL_3, M3_IN2_GPIO_Port, M3_IN2_Pin, M2_FAULT_GPIO_Port, M2_FAULT_Pin},
    {&htim1, TIM_CHANNEL_4, M4_IN2_GPIO_Port, M4_IN2_Pin, M4_FAULT_GPIO_Port, M4_FAULT_Pin},
};

static motor_runtime_t      motor_runtime[MOTOR_ID_COUNT];
static motor_driver_state_t motor_state;
static uint8_t              phase_switch_gap_calls;

static void MotorDriver_UpdateEffectivePwmAll(void);

static void MotorDriver_LatchTim1BreakLocked(void)
{
    htim1.Instance->BDTR &= ~TIM_BDTR_MOE;
    htim1.Instance->CCR1 = 0U;
    htim1.Instance->CCR2 = 0U;
    htim1.Instance->CCR3 = 0U;
    htim1.Instance->CCR4 = 0U;
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        motor_runtime[i].requested_pwm = 0;
        motor_runtime[i].applied_pwm   = 0;
        motor_runtime[i].pending_dir   = 0;
        motor_runtime[i].phase         = MOTOR_DRIVER_PHASE_IDLE_BRAKE;
        motor_runtime[i].wait_cycles   = 0U;
        motor_state.requested_pwm[i]   = 0;
        motor_state.applied_pwm[i]     = 0;
        motor_state.output_permille[i] = 0;
        motor_state.effective_pwm[i]   = 0;
        motor_state.pending_dir[i]     = 0;
        motor_state.phase[i]           = MOTOR_DRIVER_PHASE_IDLE_BRAKE;
    }
    motor_state.tim1_moe_active    = 0U;
    motor_state.tim1_break_flag    = 1U;
    motor_state.tim1_break_latched = 1U;
    motor_state.tim1_break_count++;
    motor_state.tim1_break_last_ms = osKernelGetTickCount();
}

void MotorDriver_OnTim1BreakFromIsr(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    motor_state.break_origin = MOTOR_BREAK_ORIGIN_TIM1_RUNTIME;
    MotorDriver_LatchTim1BreakLocked();
}

static uint8_t MotorDriver_ReadNfaultHighMask(void)
{
    uint8_t mask = 0U;

    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (HAL_GPIO_ReadPin(motor_hw[i].fault_port, motor_hw[i].fault_pin) == GPIO_PIN_SET)
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
    return (HAL_GPIO_ReadPin(TIM1_BKIN_GPIO_Port, TIM1_BKIN_Pin) == GPIO_PIN_SET
            && (high_mask & required_mask) == required_mask)
               ? 1U
               : 0U;
}

static void MotorDriver_UpdateBreakStatus(void)
{
    uint8_t  tim1_break_flag = (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET) ? 1U : 0U;
    uint8_t  tim8_break_flag = (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_BREAK) != RESET) ? 1U : 0U;
    uint8_t  tim1_moe_active = ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
    uint8_t  tim8_moe_active = ((htim8.Instance->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
    uint32_t primask;

    if (tim1_break_flag != 0U)
    {
        MotorDriver_OnTim1BreakFromIsr();
        tim1_moe_active = 0U;
    }
    if (tim8_break_flag != 0U)
    {
        __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
    }

    primask = __get_PRIMASK();
    __disable_irq();
    motor_state.tim1_moe_active = tim1_moe_active;
    motor_state.tim1_break_flag = tim1_break_flag;
    motor_state.tim8_moe_active = tim8_moe_active;
    motor_state.tim8_break_flag = tim8_break_flag;
    if (tim8_break_flag != 0U)
    {
        motor_state.tim8_break_count++;
        motor_state.tim8_break_last_ms = osKernelGetTickCount();
    }
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

static int8_t MotorDriver_Sign(int16_t value)
{
    if (value > 0)
    {
        return 1;
    }
    if (value < 0)
    {
        return -1;
    }
    return 0;
}

static int16_t MotorDriver_AbsSigned(int16_t value)
{
    return (value < 0) ? (int16_t)-value : value;
}

static int16_t MotorDriver_RampMagnitude(uint16_t current, uint16_t target)
{
    if (current < target)
    {
        uint16_t next = (uint16_t)(current + MOTOR_PWM_RISE_STEP_PER_CYCLE);
        return (next > target) ? (int16_t)target : (int16_t)next;
    }
    if (current > target)
    {
        if ((uint16_t)(current - target) <= MOTOR_PWM_FALL_STEP_PER_CYCLE)
        {
            return (int16_t)target;
        }
        return (int16_t)(current - MOTOR_PWM_FALL_STEP_PER_CYCLE);
    }
    return (int16_t)current;
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
    uint32_t               primask;
    const motor_runtime_t *runtime;

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
    const motor_hw_t      *hw      = &motor_hw[(uint32_t)motor];
    const motor_runtime_t *runtime = &motor_runtime[(uint32_t)motor];
    uint32_t pulse = MotorDriver_PulseFromPermille(hw->in1_htim, MotorDriver_AbsSigned(runtime->applied_pwm));

    MotorDriver_SetEnPulse(hw, pulse);
    MotorDriver_RecordRuntime(motor);
}

static void MotorDriver_ClearRuntimeOutput(motor_id_t motor)
{
    const motor_hw_t *hw = &motor_hw[(uint32_t)motor];

    motor_runtime[(uint32_t)motor].requested_pwm = 0;
    motor_runtime[(uint32_t)motor].applied_pwm   = 0;
    motor_runtime[(uint32_t)motor].pending_dir   = 0;
    motor_runtime[(uint32_t)motor].phase         = MOTOR_DRIVER_PHASE_IDLE_BRAKE;
    motor_runtime[(uint32_t)motor].wait_cycles   = 0U;
    MotorDriver_SetEnPulse(hw, 0U);
    MotorDriver_RecordRuntime(motor);
}

static void MotorDriver_DisableRuntimeOutput(motor_id_t motor)
{
    const motor_hw_t *hw = &motor_hw[(uint32_t)motor];

    motor_runtime[(uint32_t)motor] = (motor_runtime_t){
        .current_ph_dir = -1,
        .phase          = MOTOR_DRIVER_PHASE_IDLE_BRAKE,
    };
    MotorDriver_SetRaw(hw, 0U, -1);
    MotorDriver_RecordRuntime(motor);
}

static uint8_t MotorDriver_CanSwitchPhase(void)
{
    if (phase_switch_gap_calls != 0U)
    {
        phase_switch_gap_calls--;
        return 0U;
    }
    return 1U;
}

static void MotorDriver_SwitchPhase(motor_id_t motor, int8_t target_dir)
{
    motor_runtime_t *runtime = &motor_runtime[(uint32_t)motor];

    MotorDriver_SetEnPulse(&motor_hw[(uint32_t)motor], 0U);
    MotorDriver_WritePhase(&motor_hw[(uint32_t)motor], target_dir);
    runtime->current_ph_dir    = target_dir;
    runtime->pending_dir       = 0;
    runtime->phase             = MOTOR_DRIVER_PHASE_RAMP_UP;
    runtime->phase_initialized = 1U;
    phase_switch_gap_calls     = MOTOR_PHASE_SWITCH_GAP_CALLS;
    MotorDriver_RecordRuntime(motor);
}

static void MotorDriver_StartPwm(const motor_hw_t *motor)
{
    (void)HAL_TIM_PWM_Start(motor->in1_htim, motor->in1_channel);
}

void MotorDriver_Init(void)
{
    uint8_t  pre_wake_bif;
    uint8_t  startup_qualified = 1U;
    uint8_t  nfault_high_mask  = 0U;
    uint32_t stable_high_ms    = 0U;

    htim1.Instance->DIER &= ~TIM_IT_BREAK;
    htim1.Instance->BDTR &= ~TIM_BDTR_MOE;
    pre_wake_bif = (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET) ? 1U : 0U;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    HAL_GPIO_WritePin(DRV_SLEEP_ALL_GPIO_Port, DRV_SLEEP_ALL_Pin, GPIO_PIN_SET);
    HAL_Delay(DRV8874_WAKE_DELAY_MS);
    motor_state               = (motor_driver_state_t){0};
    phase_switch_gap_calls    = 0U;
    motor_state.sleep_enabled = 1U;
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        motor_runtime[i] = (motor_runtime_t){
            .current_ph_dir = -1,
            .phase          = MOTOR_DRIVER_PHASE_IDLE_BRAKE,
        };
        MotorDriver_StartPwm(&motor_hw[i]);
        htim1.Instance->BDTR &= ~TIM_BDTR_MOE;
        MotorDriver_SetRaw(&motor_hw[i], 0U, -1);
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
    motor_state.startup_pre_wake_bif = pre_wake_bif;
    motor_state.startup_bkin_high    = (HAL_GPIO_ReadPin(TIM1_BKIN_GPIO_Port, TIM1_BKIN_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    motor_state.startup_nfault_high_mask = nfault_high_mask;
    motor_state.startup_qualified        = startup_qualified;
    /* 清除 PWM 启动期间可能由 HAL_TIM_PWM_Start 触发的 TIM1/TIM8 break 标志，
     避免因 BKIN 脚在 DRV 唤醒过程中短暂低电平导致的误锁存。 */
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
    if (startup_qualified == 0U || __HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET
        || HAL_GPIO_ReadPin(TIM1_BKIN_GPIO_Port, TIM1_BKIN_Pin) == GPIO_PIN_RESET)
    {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
        motor_state.break_origin = MOTOR_BREAK_ORIGIN_STARTUP_TIMEOUT;
        MotorDriver_LatchTim1BreakLocked();
    }
    else
    {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
        htim1.Instance->BDTR |= TIM_BDTR_MOE;
    }
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    MotorDriver_UpdateFaults();
}

void MotorDriver_SetPermille(motor_id_t motor, int16_t permille)
{
    param_store_t    params;
    motor_runtime_t *runtime;
    int16_t          corrected;
    int8_t           target_dir;
    uint16_t         target_mag;
    uint16_t         applied_mag;
    int8_t           applied_dir;
    int16_t          next_mag;

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

    (void)ParamStore_GetSnapshot(&params);
    corrected =
        MotorDriver_ClampSignedPermille(DirectionApply_Signed((int32_t)permille, params.motor_dir[(uint32_t)motor]));
    runtime->requested_pwm = corrected;
    target_dir             = MotorDriver_Sign(corrected);
    target_mag             = (uint16_t)MotorDriver_AbsSigned(corrected);
    applied_dir            = MotorDriver_Sign(runtime->applied_pwm);
    applied_mag            = (uint16_t)MotorDriver_AbsSigned(runtime->applied_pwm);

    if (target_dir == 0)
    {
        next_mag             = MotorDriver_RampMagnitude(applied_mag, 0U);
        runtime->applied_pwm = (applied_dir != 0) ? (int16_t)(next_mag * applied_dir) : 0;
        runtime->pending_dir = 0;
        runtime->wait_cycles = 0U;
        runtime->phase = (runtime->applied_pwm == 0) ? MOTOR_DRIVER_PHASE_IDLE_BRAKE : MOTOR_DRIVER_PHASE_RAMP_DOWN;
        MotorDriver_ApplyRuntimeOutput(motor);
        return;
    }

    if (runtime->applied_pwm != 0 && applied_dir != target_dir)
    {
        next_mag             = MotorDriver_RampMagnitude(applied_mag, 0U);
        runtime->applied_pwm = (int16_t)(next_mag * applied_dir);
        runtime->pending_dir = target_dir;
        if (runtime->applied_pwm == 0)
        {
            runtime->phase       = MOTOR_DRIVER_PHASE_REVERSE_BRAKE;
            runtime->wait_cycles = (g_speed_getter != 0) ? MOTOR_REVERSE_MAX_BRAKE_CYCLES : MOTOR_REVERSE_BRAKE_CYCLES;
        }
        else
        {
            runtime->phase = MOTOR_DRIVER_PHASE_RAMP_DOWN;
        }
        MotorDriver_ApplyRuntimeOutput(motor);
        return;
    }

    if (runtime->applied_pwm == 0 && runtime->current_ph_dir != target_dir)
    {
        runtime->pending_dir = target_dir;
        runtime->applied_pwm = 0;

        if (runtime->phase == MOTOR_DRIVER_PHASE_REVERSE_BRAKE)
        {
            uint8_t ready_to_reverse = 0U;
            if (g_speed_getter != 0)
            {
                float speed     = g_speed_getter(motor);
                float abs_speed = (speed < 0.0f) ? -speed : speed;
                if (abs_speed < MOTOR_REVERSE_SPEED_THRESHOLD_MPS)
                {
                    ready_to_reverse = 1U;
                }
            }
            if (ready_to_reverse == 0U && runtime->wait_cycles > 1U)
            {
                runtime->wait_cycles--;
            }
            else
            {
                runtime->wait_cycles = 0U;
                runtime->phase       = MOTOR_DRIVER_PHASE_PH_SETTLE;
            }
            MotorDriver_ApplyRuntimeOutput(motor);
            return;
        }

        if (runtime->phase_initialized != 0U)
        {
            runtime->phase = MOTOR_DRIVER_PHASE_PH_SETTLE;
            if (MotorDriver_CanSwitchPhase() == 0U)
            {
                MotorDriver_ApplyRuntimeOutput(motor);
                return;
            }
            MotorDriver_SwitchPhase(motor, target_dir);
            return;
        }

        MotorDriver_WritePhase(&motor_hw[(uint32_t)motor], target_dir);
        runtime->current_ph_dir    = target_dir;
        runtime->pending_dir       = 0;
        runtime->phase_initialized = 1U;
        runtime->phase             = MOTOR_DRIVER_PHASE_RAMP_UP;
    }

    if (runtime->phase == MOTOR_DRIVER_PHASE_PH_SETTLE)
    {
        if (MotorDriver_CanSwitchPhase() == 0U)
        {
            MotorDriver_ApplyRuntimeOutput(motor);
            return;
        }
        MotorDriver_SwitchPhase(motor, target_dir);
        return;
    }

    next_mag             = MotorDriver_RampMagnitude((uint16_t)MotorDriver_AbsSigned(runtime->applied_pwm), target_mag);
    runtime->applied_pwm = (int16_t)(next_mag * target_dir);
    runtime->pending_dir = 0;
    runtime->phase       = (runtime->applied_pwm == 0) ? MOTOR_DRIVER_PHASE_IDLE_BRAKE : MOTOR_DRIVER_PHASE_RUN;
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
    phase_switch_gap_calls = 0U;
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
            fault_active[i] =
                (HAL_GPIO_ReadPin(motor_hw[i].fault_port, motor_hw[i].fault_pin) == GPIO_PIN_RESET) ? 1U : 0U;
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

    MotorDriver_UpdateFaults();
    if (HAL_GPIO_ReadPin(TIM1_BKIN_GPIO_Port, TIM1_BKIN_Pin) == GPIO_PIN_RESET)
    {
        return 0U;
    }
    for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U && motor_state.fault_active[i] != 0U)
        {
            return 0U;
        }
        if (motor_state.effective_pwm[i] != 0 || motor_runtime[i].applied_pwm != 0)
        {
            return 0U;
        }
    }
    if (htim1.Instance->CCR1 != 0U || htim1.Instance->CCR2 != 0U || htim1.Instance->CCR3 != 0U
        || htim1.Instance->CCR4 != 0U)
    {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    motor_state.tim1_break_flag    = 0U;
    motor_state.tim1_break_latched = 0U;
    htim1.Instance->BDTR |= TIM_BDTR_MOE;
    motor_state.tim1_moe_active = 1U;
    MotorDriver_UpdateEffectivePwmAll();
    __set_PRIMASK(primask);
    return 1U;
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
