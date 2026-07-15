#include "motor_break.h"

#include "main.h"
#include "tim.h"

static motor_break_snapshot_t break_state;

static void MotorBreak_ClearTim1Compares(void)
{
    htim1.Instance->CCR1 = 0U;
    htim1.Instance->CCR2 = 0U;
    htim1.Instance->CCR3 = 0U;
    htim1.Instance->CCR4 = 0U;
}

static uint8_t MotorBreak_Tim1ComparesZero(void)
{
    return (htim1.Instance->CCR1 == 0U && htim1.Instance->CCR2 == 0U && htim1.Instance->CCR3 == 0U
            && htim1.Instance->CCR4 == 0U)
               ? 1U
               : 0U;
}

static void MotorBreak_Latch(motor_break_origin_t origin, uint32_t now_ms)
{
    htim1.Instance->BDTR &= ~TIM_BDTR_MOE;
    MotorBreak_ClearTim1Compares();
    break_state.tim1_moe_active    = 0U;
    break_state.tim1_break_flag    = 1U;
    break_state.tim1_break_latched = 1U;
    break_state.tim1_break_count++;
    break_state.tim1_break_last_ms = now_ms;
    break_state.break_origin       = origin;
}

void MotorBreak_Init(void)
{
    uint8_t pre_wake_bif;

    htim1.Instance->DIER &= ~TIM_IT_BREAK;
    htim1.Instance->BDTR &= ~TIM_BDTR_MOE;
    pre_wake_bif = (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET) ? 1U : 0U;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    break_state                      = (motor_break_snapshot_t){0};
    break_state.startup_pre_wake_bif = pre_wake_bif;
}

uint8_t MotorBreak_CompleteStartup(uint8_t qualified, uint8_t nfault_high_mask, uint32_t now_ms)
{
    break_state.startup_qualified        = qualified;
    break_state.startup_bkin_high        = MotorBreak_IsBkinHigh();
    break_state.startup_nfault_high_mask = nfault_high_mask;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
    if (qualified == 0U || MotorBreak_IsBkinHigh() == 0U)
    {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
        MotorBreak_Latch(MOTOR_BREAK_ORIGIN_STARTUP_TIMEOUT, now_ms);
    }
    else
    {
        __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
        htim1.Instance->BDTR |= TIM_BDTR_MOE;
        break_state.tim1_moe_active = 1U;
    }
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
    return break_state.tim1_break_latched;
}

void MotorBreak_OnTim1Runtime(uint32_t now_ms)
{
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    MotorBreak_Latch(MOTOR_BREAK_ORIGIN_TIM1_RUNTIME, now_ms);
}

uint8_t MotorBreak_Update(uint32_t now_ms)
{
    uint8_t tim1_flag = (__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_BREAK) != RESET) ? 1U : 0U;
    uint8_t tim8_flag = (__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_BREAK) != RESET) ? 1U : 0U;

    if (tim1_flag != 0U)
    {
        MotorBreak_OnTim1Runtime(now_ms);
    }
    if (tim8_flag != 0U)
    {
        __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
        break_state.tim8_break_count++;
        break_state.tim8_break_last_ms = now_ms;
    }
    break_state.tim1_break_flag = tim1_flag;
    break_state.tim8_break_flag = tim8_flag;
    break_state.tim1_moe_active = ((htim1.Instance->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
    break_state.tim8_moe_active = ((htim8.Instance->BDTR & TIM_BDTR_MOE) != 0U) ? 1U : 0U;
    return tim1_flag;
}

uint8_t MotorBreak_ClearLatch(uint8_t external_safe)
{
    if (external_safe == 0U || MotorBreak_IsBkinHigh() == 0U || MotorBreak_Tim1ComparesZero() == 0U)
    {
        return 0U;
    }
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
    break_state.tim1_break_flag    = 0U;
    break_state.tim1_break_latched = 0U;
    htim1.Instance->BDTR |= TIM_BDTR_MOE;
    break_state.tim1_moe_active = 1U;
    return 1U;
}

uint8_t MotorBreak_IsBkinHigh(void)
{
    return (HAL_GPIO_ReadPin(TIM1_BKIN_GPIO_Port, TIM1_BKIN_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

void MotorBreak_GetSnapshot(motor_break_snapshot_t *snapshot)
{
    if (snapshot != 0)
    {
        *snapshot = break_state;
    }
}
