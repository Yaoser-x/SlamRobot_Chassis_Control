#include "motor_driver.h"

#include "chassis_config.h"
#include "chassis_layout.h"
#include "main.h"
#include "tim.h"

typedef struct
{
  TIM_HandleTypeDef *in1_htim;
  uint32_t in1_channel;
  TIM_HandleTypeDef *in2_htim;
  uint32_t in2_channel;
  GPIO_TypeDef *fault_port;
  uint16_t fault_pin;
} motor_hw_t;

typedef struct
{
  int8_t last_drive_sign;
  uint8_t direction_change_coast_cycles;
} motor_runtime_t;

/* CubeMX labels keep legacy M2/M3 names; logical M2/M3 nFAULT pins are crossed here. */
static const motor_hw_t motor_hw[MOTOR_ID_COUNT] = {
  { &htim1, TIM_CHANNEL_1, &htim8, TIM_CHANNEL_1, M1_FAULT_GPIO_Port, M1_FAULT_Pin },
  { &htim1, TIM_CHANNEL_2, &htim8, TIM_CHANNEL_2, M3_FAULT_GPIO_Port, M3_FAULT_Pin },
  { &htim1, TIM_CHANNEL_3, &htim8, TIM_CHANNEL_3, M2_FAULT_GPIO_Port, M2_FAULT_Pin },
  { &htim1, TIM_CHANNEL_4, &htim8, TIM_CHANNEL_4, M4_FAULT_GPIO_Port, M4_FAULT_Pin },
};

static motor_runtime_t motor_runtime[MOTOR_ID_COUNT];
static motor_driver_state_t motor_state;

static uint8_t MotorDriver_IsValidMotor(motor_id_t motor)
{
  return ((uint32_t)motor < MOTOR_ID_COUNT) ? 1U : 0U;
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

static void MotorDriver_SetRaw(const motor_hw_t *motor, uint32_t in1_pulse, uint32_t in2_pulse)
{
  __HAL_TIM_SET_COMPARE(motor->in1_htim, motor->in1_channel, in1_pulse);
  __HAL_TIM_SET_COMPARE(motor->in2_htim, motor->in2_channel, in2_pulse);
}

static void MotorDriver_StartPwm(const motor_hw_t *motor)
{
  (void)HAL_TIM_PWM_Start(motor->in1_htim, motor->in1_channel);
  (void)HAL_TIM_PWM_Start(motor->in2_htim, motor->in2_channel);
}

void MotorDriver_Init(void)
{
  HAL_GPIO_WritePin(DRV_SLEEP_ALL_GPIO_Port, DRV_SLEEP_ALL_Pin, GPIO_PIN_SET);
  HAL_Delay(DRV8874_WAKE_DELAY_MS);

  motor_state = (motor_driver_state_t){0};
  motor_state.sleep_enabled = 1U;
  for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    motor_runtime[i] = (motor_runtime_t){0};
    MotorDriver_StartPwm(&motor_hw[i]);
    MotorDriver_SetRaw(&motor_hw[i], 0U, 0U);
  }
  MotorDriver_UpdateFaults();
}

void MotorDriver_SetPermille(motor_id_t motor, int16_t permille)
{
  const motor_hw_t *hw;
  motor_runtime_t *runtime;
  int16_t corrected;
  int8_t drive_sign;
  uint32_t pulse;

  if (MotorDriver_IsValidMotor(motor) == 0U)
  {
    return;
  }

  hw = &motor_hw[(uint32_t)motor];
  runtime = &motor_runtime[(uint32_t)motor];
  if (ChassisLayout_MotorEnabled(motor) == 0U)
  {
    runtime->last_drive_sign = 0;
    runtime->direction_change_coast_cycles = 0U;
    MotorDriver_SetRaw(hw, 0U, 0U);
    return;
  }

  corrected = (int16_t)(permille * ChassisLayout_MotorDirection(motor));
  drive_sign = MotorDriver_Sign(corrected);

  if (corrected < 0)
  {
    pulse = MotorDriver_PulseFromPermille(hw->in2_htim, (int16_t)-corrected);
  }
  else
  {
    pulse = MotorDriver_PulseFromPermille(hw->in1_htim, corrected);
  }

  if (drive_sign == 0)
  {
    runtime->last_drive_sign = 0;
    runtime->direction_change_coast_cycles = 0U;
    MotorDriver_SetRaw(hw, 0U, 0U);
    return;
  }

  if (runtime->direction_change_coast_cycles > 0U)
  {
    runtime->direction_change_coast_cycles--;
    MotorDriver_SetRaw(hw, 0U, 0U);
    return;
  }

  if (runtime->last_drive_sign != 0 && drive_sign != runtime->last_drive_sign)
  {
    runtime->last_drive_sign = 0;
    runtime->direction_change_coast_cycles = MOTOR_DIRECTION_CHANGE_COAST_CYCLES;
    MotorDriver_SetRaw(hw, 0U, 0U);
    return;
  }

  if (corrected > 0)
  {
    MotorDriver_SetRaw(hw, pulse, 0U);
  }
  else
  {
    MotorDriver_SetRaw(hw, 0U, pulse);
  }
  runtime->last_drive_sign = drive_sign;
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

void MotorDriver_SetInputPermille(motor_id_t motor, int16_t forward_permille, int16_t reverse_permille)
{
  const motor_hw_t *hw;

  if (MotorDriver_IsValidMotor(motor) == 0U)
  {
    return;
  }

  hw = &motor_hw[(uint32_t)motor];
  if (ChassisLayout_MotorEnabled(motor) == 0U)
  {
    motor_runtime[(uint32_t)motor] = (motor_runtime_t){0};
    MotorDriver_SetRaw(hw, 0U, 0U);
    return;
  }

  motor_runtime[(uint32_t)motor] = (motor_runtime_t){0};
  MotorDriver_SetRaw(hw,
                     MotorDriver_PulseFromPermille(hw->in1_htim, forward_permille),
                     MotorDriver_PulseFromPermille(hw->in2_htim, reverse_permille));
}

void MotorDriver_SetSideInputPermille(motor_side_t side, int16_t forward_permille, int16_t reverse_permille)
{
  if (side == MOTOR_SIDE_LEFT)
  {
    MotorDriver_SetInputPermille(MOTOR_ID_M1, forward_permille, reverse_permille);
    MotorDriver_SetInputPermille(MOTOR_ID_M2, forward_permille, reverse_permille);
  }
  else
  {
    MotorDriver_SetInputPermille(MOTOR_ID_M3, forward_permille, reverse_permille);
    MotorDriver_SetInputPermille(MOTOR_ID_M4, forward_permille, reverse_permille);
  }
}

void MotorDriver_Stop(motor_id_t motor, motor_stop_mode_t mode)
{
  const motor_hw_t *hw;
  uint32_t in1_pulse = 0U;
  uint32_t in2_pulse = 0U;

  if (MotorDriver_IsValidMotor(motor) == 0U)
  {
    return;
  }

  hw = &motor_hw[(uint32_t)motor];
  motor_runtime[(uint32_t)motor] = (motor_runtime_t){0};
  if (mode == MOTOR_STOP_BRAKE)
  {
    in1_pulse = MotorDriver_PulseFromPermille(hw->in1_htim, CHASSIS_PWM_MAX_PERMILLE);
    in2_pulse = MotorDriver_PulseFromPermille(hw->in2_htim, CHASSIS_PWM_MAX_PERMILLE);
  }
  MotorDriver_SetRaw(hw, in1_pulse, in2_pulse);
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
  uint8_t fault_active[MOTOR_ID_COUNT];
  uint32_t primask;

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
    motor_state.fault_active[i] = fault_active[i];
  }
  __set_PRIMASK(primask);
}

uint8_t MotorDriver_HasFault(void)
{
  MotorDriver_UpdateFaults();
  for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    if (motor_state.fault_active[i] != 0U)
    {
      return 1U;
    }
  }
  return 0U;
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
