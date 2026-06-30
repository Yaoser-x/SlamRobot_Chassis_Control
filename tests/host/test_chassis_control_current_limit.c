#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "adc_monitor.h"
#include "bsp_config.h"
#include "chassis_config.h"
#include "chassis_control.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "pid_controller.h"

static adc_monitor_state_t fake_adc_state;
static encoder_state_t fake_encoder_state;
static int16_t fake_input_forward[MOTOR_ID_COUNT];
static int16_t fake_input_reverse[MOTOR_ID_COUNT];
static uint8_t fake_fault_stop;
static uint8_t fake_primask;

uint32_t __get_PRIMASK(void)
{
  return fake_primask;
}

void __disable_irq(void)
{
  fake_primask = 1U;
}

void __set_PRIMASK(uint32_t primask)
{
  fake_primask = primask;
}

void MotorDriver_Init(void)
{
}

void MotorDriver_SetPermille(motor_id_t motor, int16_t permille)
{
  fake_input_forward[motor] = (permille > 0) ? permille : 0;
  fake_input_reverse[motor] = (permille < 0) ? (int16_t)-permille : 0;
}

void MotorDriver_SetInputPermille(motor_id_t motor, int16_t forward_permille, int16_t reverse_permille)
{
  fake_input_forward[motor] = forward_permille;
  fake_input_reverse[motor] = reverse_permille;
}

void MotorDriver_StopAll(motor_stop_mode_t mode)
{
  (void)mode;
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    fake_input_forward[i] = 0;
    fake_input_reverse[i] = 0;
  }
}

void MotorDriver_UpdateFaults(void)
{
}

uint8_t MotorDriver_HasFault(void)
{
  return 0U;
}

void ControlManager_Init(void)
{
  fake_fault_stop = 0U;
}

void ControlManager_SetFaultStop(uint8_t enabled)
{
  fake_fault_stop = (enabled != 0U) ? 1U : 0U;
}

uint8_t ControlManager_IsEmergencyStop(void)
{
  return 0U;
}

uint8_t ControlManager_IsFaultStop(void)
{
  return fake_fault_stop;
}

uint8_t ControlManager_GetCommand(chassis_cmd_t *cmd, uint32_t now_ms)
{
  (void)cmd;
  (void)now_ms;
  return 0U;
}

void ControlManager_ClearCommand(void)
{
}

void EncoderDriver_GetState(encoder_state_t *state)
{
  *state = fake_encoder_state;
}

void AdcMonitor_GetState(adc_monitor_state_t *state)
{
  *state = fake_adc_state;
}

static void require_int(int condition, const char *message)
{
  if (condition == 0)
  {
    (void)printf("FAIL: %s\n", message);
    exit(1);
  }
}

static void reset_fake_chassis(void)
{
  fake_adc_state = (adc_monitor_state_t){0};
  fake_encoder_state = (encoder_state_t){0};
  fake_fault_stop = 0U;
  fake_primask = 0U;
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    fake_input_forward[i] = 0;
    fake_input_reverse[i] = 0;
    fake_encoder_state.speed_valid[i] = 1U;
  }
  fake_encoder_state.speed_valid_all = 1U;
  fake_adc_state.current_valid = 1U;
  ChassisControl_Init();
}

static void test_high_adc_current_does_not_throttle_pwm_output(void)
{
  chassis_control_state_t state;

  reset_fake_chassis();
  fake_adc_state.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;

  ChassisControl_RawMotorInputTest(MOTOR_ID_M2, 50, 0);
  ChassisControl_Step(1000U);
  ChassisControl_GetState(&state);

  require_int(fake_input_forward[MOTOR_ID_M2] == CHASSIS_OUTPUT_SLEW_STEP_PER_CYCLE,
              "M2 high ADC current does not throttle raw forward PWM");
  require_int(fake_input_reverse[MOTOR_ID_M2] == 0, "M2 raw current limit keeps reverse PWM off");
  require_int(state.motor_output_permille[MOTOR_ID_M2] == CHASSIS_OUTPUT_SLEW_STEP_PER_CYCLE,
              "M2 high ADC current state reports applied output");
  require_int(state.motor_current_limited[MOTOR_ID_M2] == 0U, "M2 high ADC current does not report dynamic limit");
}

int main(void)
{
  test_high_adc_current_does_not_throttle_pwm_output();
  return 0;
}
