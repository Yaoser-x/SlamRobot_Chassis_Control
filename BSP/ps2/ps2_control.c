#include "ps2_control.h"

#include "chassis_config.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "line_control.h"
#include "main.h"

#define PS2_FRAME_LEN 9U
#define PS2_RETRY_LIMIT 3U
#define PS2_CLK_HALF_PERIOD_US 10U
#define PS2_BYTE_GAP_US 20U
#define PS2_FRAME_GAP_US 30U
#define PS2_DPAD_UP_MASK 0x10U
#define PS2_DPAD_RIGHT_MASK 0x20U
#define PS2_DPAD_DOWN_MASK 0x40U
#define PS2_DPAD_LEFT_MASK 0x80U

typedef struct
{
  uint8_t mode;
  uint8_t btn1;
  uint8_t btn2;
  uint8_t right_x;
  uint8_t right_y;
  uint8_t left_x;
  uint8_t left_y;
} ps2_sample_t;

static const uint8_t ps2_poll_frame[PS2_FRAME_LEN] = {
  0x01U, 0x42U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
};

static ps2_control_state_t ps2_state;
static uint8_t ps2_rx[PS2_FRAME_LEN];
static uint8_t last_btn2;
static uint8_t consecutive_read_failures;
static uint8_t macro_active;
static uint8_t macro_button;
static float macro_angular_z;
static uint32_t macro_end_ms;
static uint8_t ps2_timing_fault;

static void Ps2Control_CopyState(ps2_control_state_t *dst, const ps2_control_state_t *src)
{
  uint32_t primask;

  if (dst == 0 || src == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *dst = *src;
  __set_PRIMASK(primask);
}

static void Ps2Control_IncrementRxOk(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  ps2_state.rx_ok_count++;
  __set_PRIMASK(primask);
}

static void Ps2Control_IncrementRxFail(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  ps2_state.rx_fail_count++;
  __set_PRIMASK(primask);
}

static void Ps2Control_DwtDelayInit(void)
{
  if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0U)
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  ps2_timing_fault = 0U;
}

static void Ps2Control_DelayUs(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = (SystemCoreClock / 1000000U) * us;
  uint32_t guard = ticks + (ticks / 2U) + 1000U;

  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U || ticks == 0U)
  {
    ps2_timing_fault = 1U;
    return;
  }

  while ((DWT->CYCCNT - start) < ticks)
  {
    if (guard == 0U)
    {
      ps2_timing_fault = 1U;
      return;
    }
    guard--;
  }
}

static GPIO_TypeDef *Ps2Control_CmdPort(void)
{
  return PS2_DO_GPIO_Port;
}

static uint16_t Ps2Control_CmdPin(void)
{
  return PS2_DO_Pin;
}

static GPIO_TypeDef *Ps2Control_DatPort(void)
{
  return PS2_DI_GPIO_Port;
}

static uint16_t Ps2Control_DatPin(void)
{
  return PS2_DI_Pin;
}

static void Ps2Control_SetCmd(uint8_t high)
{
  HAL_GPIO_WritePin(Ps2Control_CmdPort(),
                    Ps2Control_CmdPin(),
                    (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Ps2Control_SetClk(uint8_t high)
{
  HAL_GPIO_WritePin(PS2_CLK_GPIO_Port,
                    PS2_CLK_Pin,
                    (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Ps2Control_SetCs(uint8_t high)
{
  HAL_GPIO_WritePin(PS2_CS_GPIO_Port,
                    PS2_CS_Pin,
                    (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t Ps2Control_ReadDat(void)
{
  return (HAL_GPIO_ReadPin(Ps2Control_DatPort(), Ps2Control_DatPin()) == GPIO_PIN_SET) ? 1U : 0U;
}

static void Ps2Control_ConfigurePins(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = Ps2Control_DatPin();
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Ps2Control_DatPort(), &gpio);

  gpio.Pin = Ps2Control_CmdPin();
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Ps2Control_CmdPort(), &gpio);

  gpio.Pin = PS2_CLK_Pin;
  HAL_GPIO_Init(PS2_CLK_GPIO_Port, &gpio);

  gpio.Pin = PS2_CS_Pin;
  HAL_GPIO_Init(PS2_CS_GPIO_Port, &gpio);

  Ps2Control_SetCmd(1U);
  Ps2Control_SetClk(1U);
  Ps2Control_SetCs(1U);
}

static uint8_t Ps2Control_TransferByte(uint8_t tx)
{
  uint8_t rx = 0U;

  for (uint8_t bit = 0x01U; bit != 0U; bit <<= 1U)
  {
    Ps2Control_SetCmd(((tx & bit) != 0U) ? 1U : 0U);
    Ps2Control_DelayUs(PS2_CLK_HALF_PERIOD_US);
    Ps2Control_SetClk(0U);
    Ps2Control_DelayUs(PS2_CLK_HALF_PERIOD_US);

    if (Ps2Control_ReadDat() != 0U)
    {
      rx |= bit;
    }

    Ps2Control_SetClk(1U);
    Ps2Control_DelayUs(PS2_CLK_HALF_PERIOD_US);
  }

  Ps2Control_SetCmd(1U);
  return rx;
}

static uint8_t Ps2Control_Send(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
  ps2_timing_fault = 0U;
  Ps2Control_SetCs(0U);
  Ps2Control_DelayUs(PS2_FRAME_GAP_US);

  for (uint8_t i = 0U; i < len; ++i)
  {
    uint8_t value = Ps2Control_TransferByte(tx[i]);
    if (ps2_timing_fault != 0U)
    {
      break;
    }
    if (rx != 0)
    {
      rx[i] = value;
    }
    Ps2Control_DelayUs(PS2_BYTE_GAP_US);
    if (ps2_timing_fault != 0U)
    {
      break;
    }
  }

  Ps2Control_SetCs(1U);
  Ps2Control_DelayUs(PS2_FRAME_GAP_US);
  return (ps2_timing_fault == 0U) ? 1U : 0U;
}

static uint8_t Ps2Control_HandshakeOk(const uint8_t *rx)
{
  return (rx[1] != 0xFFU && rx[2] == 0x5AU) ? 1U : 0U;
}

static uint8_t Ps2Control_IsAnalogMode(uint8_t mode)
{
  return (mode == 0x73U || mode == 0x79U) ? 1U : 0U;
}

static void Ps2Control_ShortPoll(void)
{
  static const uint8_t poll[5] = {0x01U, 0x42U, 0x00U, 0x00U, 0x00U};
  (void)Ps2Control_Send(poll, 0, (uint8_t)sizeof(poll));
}

static void Ps2Control_ConfigAnalog(void)
{
  static const uint8_t enter_cfg[PS2_FRAME_LEN] = {0x01U, 0x43U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t analog_cfg[PS2_FRAME_LEN] = {0x01U, 0x44U, 0x00U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U};
  static const uint8_t vibration_cfg[5] = {0x01U, 0x4DU, 0x00U, 0x00U, 0x01U};
  static const uint8_t exit_cfg[PS2_FRAME_LEN] = {0x01U, 0x43U, 0x00U, 0x00U, 0x5AU, 0x5AU, 0x5AU, 0x5AU, 0x5AU};

  Ps2Control_ShortPoll();
  Ps2Control_ShortPoll();
  Ps2Control_ShortPoll();
  HAL_Delay(2U);
  (void)Ps2Control_Send(enter_cfg, 0, (uint8_t)sizeof(enter_cfg));
  HAL_Delay(2U);
  (void)Ps2Control_Send(analog_cfg, 0, (uint8_t)sizeof(analog_cfg));
  HAL_Delay(2U);
  (void)Ps2Control_Send(vibration_cfg, 0, (uint8_t)sizeof(vibration_cfg));
  HAL_Delay(2U);
  (void)Ps2Control_Send(exit_cfg, 0, (uint8_t)sizeof(exit_cfg));
  HAL_Delay(2U);
}

static uint8_t Ps2Control_Probe(uint8_t require_analog)
{
  uint8_t rx[PS2_FRAME_LEN] = {0};

  for (uint8_t i = 0U; i < 6U; ++i)
  {
    if (Ps2Control_Send(ps2_poll_frame, rx, (uint8_t)sizeof(rx)) != 0U &&
        Ps2Control_HandshakeOk(rx) != 0U &&
        (require_analog == 0U || Ps2Control_IsAnalogMode(rx[1]) != 0U))
    {
      return 1U;
    }
    HAL_Delay(2U);
  }
  return 0U;
}

static uint8_t Ps2Control_InitMapping(void)
{
  for (uint8_t i = 0U; i < PS2_RETRY_LIMIT; ++i)
  {
    Ps2Control_ConfigAnalog();
    if (Ps2Control_Probe(1U) != 0U)
    {
      return 1U;
    }
  }
  return Ps2Control_Probe(0U);
}

static uint8_t Ps2Control_ReadSample(ps2_sample_t *sample)
{
  if (sample == 0)
  {
    return 0U;
  }

  if (Ps2Control_Send(ps2_poll_frame, ps2_rx, (uint8_t)sizeof(ps2_poll_frame)) == 0U ||
      Ps2Control_HandshakeOk(ps2_rx) == 0U)
  {
    Ps2Control_IncrementRxFail();
    return 0U;
  }
  Ps2Control_IncrementRxOk();

  sample->mode = ps2_rx[1];
  sample->btn1 = (uint8_t)~ps2_rx[3];
  sample->btn2 = (uint8_t)~ps2_rx[4];
  if (Ps2Control_IsAnalogMode(sample->mode) != 0U)
  {
    sample->right_x = ps2_rx[5];
    sample->right_y = ps2_rx[6];
    sample->left_x = ps2_rx[7];
    sample->left_y = ps2_rx[8];
  }
  else
  {
    sample->right_x = PS2_AXIS_CENTER;
    sample->right_y = PS2_AXIS_CENTER;
    sample->left_x = PS2_AXIS_CENTER;
    sample->left_y = PS2_AXIS_CENTER;
  }

  return 1U;
}

static float Ps2Control_NormalizeAxis(uint8_t raw)
{
  int32_t delta = (int32_t)raw - PS2_AXIS_CENTER;
  int32_t magnitude = (delta < 0) ? -delta : delta;

  if (magnitude <= PS2_AXIS_DEADZONE)
  {
    return 0.0f;
  }

  if (delta > 0)
  {
    return (float)(delta - PS2_AXIS_DEADZONE) / (float)(127 - PS2_AXIS_DEADZONE);
  }
  return (float)(delta + PS2_AXIS_DEADZONE) / (float)(128 - PS2_AXIS_DEADZONE);
}

static float Ps2Control_ClampFloat(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}

static void Ps2Control_SubmitCommand(float linear_x, float angular_z)
{
  chassis_cmd_t cmd = {
    .linear_x = linear_x,
    .angular_z = angular_z,
    .enable = 1U,
    .source = CONTROL_SOURCE_PS2,
    .timestamp_ms = osKernelGetTickCount(),
  };

  (void)ControlManager_SetCommand(&cmd);
}

static uint8_t Ps2Control_ManualInputActive(float linear_x, float angular_z)
{
  float abs_linear = (linear_x < 0.0f) ? -linear_x : linear_x;
  float abs_angular = (angular_z < 0.0f) ? -angular_z : angular_z;

  return (abs_linear > PS2_MANUAL_CANCEL_THRESHOLD ||
          abs_angular > PS2_MANUAL_CANCEL_THRESHOLD) ? 1U : 0U;
}

static void Ps2Control_ApplyDpad(const ps2_sample_t *sample, float *linear_x, float *angular_z)
{
  uint8_t up_pressed;
  uint8_t down_pressed;
  uint8_t left_pressed;
  uint8_t right_pressed;

  if (sample == 0 || linear_x == 0 || angular_z == 0)
  {
    return;
  }

  up_pressed = ((sample->btn1 & PS2_DPAD_UP_MASK) != 0U) ? 1U : 0U;
  down_pressed = ((sample->btn1 & PS2_DPAD_DOWN_MASK) != 0U) ? 1U : 0U;
  left_pressed = ((sample->btn1 & PS2_DPAD_LEFT_MASK) != 0U) ? 1U : 0U;
  right_pressed = ((sample->btn1 & PS2_DPAD_RIGHT_MASK) != 0U) ? 1U : 0U;

  if (up_pressed != 0U && down_pressed == 0U)
  {
    *linear_x = PS2_DPAD_LINEAR_MPS;
  }
  else if (down_pressed != 0U && up_pressed == 0U)
  {
    *linear_x = -PS2_DPAD_LINEAR_MPS;
  }

  if (left_pressed != 0U && right_pressed == 0U)
  {
    *angular_z = PS2_DPAD_ANGULAR_RPS;
  }
  else if (right_pressed != 0U && left_pressed == 0U)
  {
    *angular_z = -PS2_DPAD_ANGULAR_RPS;
  }
}

static uint8_t Ps2Control_StartMacro(uint8_t pressed, uint32_t now_ms)
{
  if ((pressed & PS2_MACRO_L1_MASK) != 0U)
  {
    macro_active = 1U;
    macro_button = PS2_MACRO_L1_MASK;
    macro_angular_z = PS2_ANGULAR_MAX_RPS;
    macro_end_ms = now_ms + PS2_MACRO_LONG_TURN_MS;
    return 1U;
  }
  if ((pressed & PS2_MACRO_R1_MASK) != 0U)
  {
    macro_active = 1U;
    macro_button = PS2_MACRO_R1_MASK;
    macro_angular_z = -PS2_ANGULAR_MAX_RPS;
    macro_end_ms = now_ms + PS2_MACRO_LONG_TURN_MS;
    return 1U;
  }
  if ((pressed & PS2_MACRO_L2_MASK) != 0U)
  {
    macro_active = 1U;
    macro_button = PS2_MACRO_L2_MASK;
    macro_angular_z = PS2_ANGULAR_MAX_RPS;
    macro_end_ms = now_ms + PS2_MACRO_SHORT_TURN_MS;
    return 1U;
  }
  if ((pressed & PS2_MACRO_R2_MASK) != 0U)
  {
    macro_active = 1U;
    macro_button = PS2_MACRO_R2_MASK;
    macro_angular_z = -PS2_ANGULAR_MAX_RPS;
    macro_end_ms = now_ms + PS2_MACRO_SHORT_TURN_MS;
    return 1U;
  }
  return 0U;
}

void Ps2Control_Init(void)
{
  Ps2Control_DwtDelayInit();
  ps2_state = (ps2_control_state_t){0};
  ps2_state.left_x = PS2_AXIS_CENTER;
  ps2_state.left_y = PS2_AXIS_CENTER;
  ps2_state.right_x = PS2_AXIS_CENTER;
  ps2_state.right_y = PS2_AXIS_CENTER;
  last_btn2 = 0U;
  consecutive_read_failures = 0U;
  macro_active = 0U;
  macro_button = 0U;
  macro_angular_z = 0.0f;
  macro_end_ms = 0U;

  Ps2Control_ConfigurePins();
  (void)Ps2Control_InitMapping();
  ps2_state.cmd_dat_swapped = 0U;
}

void Ps2Control_Update(void)
{
  ps2_sample_t sample;
  ps2_control_state_t next_state;
  float linear_x = 0.0f;
  float angular_z = 0.0f;
  uint8_t pressed_btn2;
  uint8_t command_active;
  uint32_t now_ms = osKernelGetTickCount();

  if (Ps2Control_ReadSample(&sample) == 0U)
  {
    if (consecutive_read_failures < PS2_OFFLINE_FAIL_LIMIT)
    {
      consecutive_read_failures++;
    }
    if (consecutive_read_failures < PS2_OFFLINE_FAIL_LIMIT)
    {
      return;
    }

    Ps2Control_CopyState(&next_state, &ps2_state);
    next_state.online = 0U;
    next_state.drive_enabled = 0U;
    next_state.macro_active = 0U;
    next_state.macro_button = 0U;
    next_state.linear_x = 0.0f;
    next_state.angular_z = 0.0f;
    Ps2Control_CopyState(&ps2_state, &next_state);
    macro_active = 0U;
    macro_button = 0U;
    macro_angular_z = 0.0f;
    ControlManager_ClearSource(CONTROL_SOURCE_PS2);
    return;
  }
  consecutive_read_failures = 0U;

  linear_x = -Ps2Control_NormalizeAxis(sample.left_y) * PS2_LINEAR_MAX_MPS;
  angular_z = -Ps2Control_NormalizeAxis(sample.right_x) * PS2_ANGULAR_MAX_RPS;
  Ps2Control_ApplyDpad(&sample, &linear_x, &angular_z);
  pressed_btn2 = (uint8_t)(sample.btn2 & (uint8_t)~last_btn2);
  last_btn2 = sample.btn2;

  /* 巡线模式切换：三角键上升沿触发 */
  if ((pressed_btn2 & PS2_LINE_TOGGLE_MASK) != 0U)
  {
    LineControl_Enable((LineControl_IsEnabled() == 0U) ? 1U : 0U);
  }

  linear_x = Ps2Control_ClampFloat(linear_x, PS2_LINEAR_MAX_MPS);
  angular_z = Ps2Control_ClampFloat(angular_z, PS2_ANGULAR_MAX_RPS);

  if (Ps2Control_ManualInputActive(linear_x, angular_z) != 0U)
  {
    macro_active = 0U;
    macro_button = 0U;
  }
  else if (macro_active != 0U)
  {
    if ((int32_t)(now_ms - macro_end_ms) >= 0)
    {
      macro_active = 0U;
      macro_button = 0U;
      macro_angular_z = 0.0f;
    }
    else
    {
      angular_z = macro_angular_z;
    }
  }
  else
  {
    (void)Ps2Control_StartMacro(pressed_btn2, now_ms);
    if (macro_active == 0U)
    {
      macro_button = 0U;
    }
    if (macro_active != 0U)
    {
      angular_z = macro_angular_z;
    }
  }

  command_active = (Ps2Control_ManualInputActive(linear_x, angular_z) != 0U ||
                    macro_active != 0U) ? 1U : 0U;

  if (command_active == 0U)
  {
    linear_x = 0.0f;
    angular_z = 0.0f;
  }

  Ps2Control_CopyState(&next_state, &ps2_state);
  next_state.online = 1U;
  next_state.analog_mode = Ps2Control_IsAnalogMode(sample.mode);
  next_state.drive_enabled = command_active;
  next_state.btn1 = sample.btn1;
  next_state.btn2 = sample.btn2;
  next_state.left_x = sample.left_x;
  next_state.left_y = sample.left_y;
  next_state.right_x = sample.right_x;
  next_state.right_y = sample.right_y;
  next_state.macro_active = macro_active;
  next_state.macro_button = macro_button;
  next_state.linear_x = linear_x;
  next_state.angular_z = angular_z;
  next_state.line_tracking_enabled = LineControl_IsEnabled();
  Ps2Control_CopyState(&ps2_state, &next_state);

  if (command_active == 0U)
  {
    ControlManager_ClearSource(CONTROL_SOURCE_PS2);
    return;
  }

  Ps2Control_SubmitCommand(linear_x, angular_z);
}

void Ps2Control_GetState(ps2_control_state_t *state)
{
  Ps2Control_CopyState(state, &ps2_state);
}
