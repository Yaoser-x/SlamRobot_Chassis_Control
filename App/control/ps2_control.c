#include "ps2_control.h"

#include "chassis_config.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "line_control.h"
#include "main.h"
#include "ps2_hw.h"

#define PS2_DPAD_UP_MASK    0x10U
#define PS2_DPAD_RIGHT_MASK 0x20U
#define PS2_DPAD_DOWN_MASK  0x40U
#define PS2_DPAD_LEFT_MASK  0x80U

static ps2_control_state_t ps2_state;
static uint8_t last_btn2;
static uint8_t consecutive_read_failures;
static uint8_t macro_active;
static uint8_t macro_button;
static float macro_angular_z;
static uint32_t macro_end_ms;

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

static void Ps2Control_ApplyDpad(const ps2_hw_sample_t *sample, float *linear_x, float *angular_z)
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

  Ps2Hw_Init();
  ps2_state.cmd_dat_swapped = 0U;
}

void Ps2Control_Update(void)
{
  ps2_hw_sample_t sample;
  ps2_control_state_t next_state;
  float linear_x = 0.0f;
  float angular_z = 0.0f;
  uint8_t pressed_btn2;
  uint8_t command_active;
  uint32_t now_ms = osKernelGetTickCount();

  if (Ps2Hw_ReadSample(&sample) == 0U)
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
  ps2_state.rx_ok_count++;

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
  next_state.analog_mode = Ps2Hw_IsAnalogMode(sample.mode);
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
