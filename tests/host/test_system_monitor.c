#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "adc_monitor.h"
#include "bsp_config.h"
#include "chassis_layout.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "system_monitor.h"

static uint32_t fake_primask;
static uint32_t fake_tick_ms;
static adc_monitor_state_t fake_adc_state;
static encoder_state_t fake_encoder_state;
static motor_driver_state_t fake_motor_state;
static uint8_t fake_estop;
static uint8_t fake_fault_stop;
static uint8_t fake_active_source;
static uint8_t fake_set_fault_stop_count;

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

uint32_t osKernelGetTickCount(void)
{
  return fake_tick_ms;
}

void AdcMonitor_Update(void)
{
}

void AdcMonitor_GetState(adc_monitor_state_t *state)
{
  *state = fake_adc_state;
}

void EncoderDriver_GetState(encoder_state_t *state)
{
  *state = fake_encoder_state;
}

void MotorDriver_UpdateFaults(void)
{
}

void MotorDriver_GetState(motor_driver_state_t *state)
{
  *state = fake_motor_state;
}

uint8_t ControlManager_IsEmergencyStop(void)
{
  return fake_estop;
}

uint8_t ControlManager_IsFaultStop(void)
{
  return fake_fault_stop;
}

uint8_t ControlManager_GetActiveSource(void)
{
  return fake_active_source;
}

void ControlManager_SetFaultStop(uint8_t enabled)
{
  fake_fault_stop = (enabled != 0U) ? 1U : 0U;
  fake_set_fault_stop_count++;
}

uint8_t ChassisLayout_MotorEnabled(motor_id_t motor)
{
  return (motor == MOTOR_ID_M2 || motor == MOTOR_ID_M3) ? 1U : 0U;
}

static void require_int(int condition, const char *message)
{
  if (condition == 0)
  {
    (void)printf("FAIL: %s\n", message);
    exit(1);
  }
}

static void reset_fake_monitor(void)
{
  fake_primask = 0U;
  fake_tick_ms = 1000U;
  fake_adc_state = (adc_monitor_state_t){0};
  fake_encoder_state = (encoder_state_t){0};
  fake_motor_state = (motor_driver_state_t){0};
  fake_estop = 0U;
  fake_fault_stop = 0U;
  fake_active_source = CONTROL_SOURCE_DEBUG;
  fake_set_fault_stop_count = 0U;
  fake_adc_state.current_valid = 1U;
  fake_adc_state.battery_voltage = 12.0f;
  fake_encoder_state.speed_valid_all = 1U;
  SystemMonitor_Init();
}

static void update_and_advance(uint32_t step_ms)
{
  SystemMonitor_Update();
  fake_tick_ms += step_ms;
}

static void test_adc_overcurrent_does_not_fault_stop_when_software_fault_disabled(void)
{
  system_monitor_state_t state;

  reset_fake_monitor();
  fake_motor_state.output_permille[MOTOR_ID_M2] = 50;
  fake_adc_state.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;

  for (uint8_t i = 0U; i < MOTOR_OVERCURRENT_DEBOUNCE_COUNT + 3U; ++i)
  {
    update_and_advance(20U);
  }
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) == 0U,
              "disabled ADC software overcurrent does not latch M2 fault");
  require_int(fake_set_fault_stop_count == 0U, "disabled ADC software overcurrent does not request fault stop");

  fake_tick_ms = 1300U;
  for (uint8_t i = 0U; i < MOTOR_OVERCURRENT_DEBOUNCE_COUNT; ++i)
  {
    update_and_advance(20U);
  }
  SystemMonitor_GetState(&state);
  require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) == 0U,
              "disabled ADC software overcurrent stays clear after startup blank");
  require_int(fake_fault_stop == 0U, "disabled ADC software overcurrent keeps fault stop clear after blank");
}

static void test_drv_fault_is_not_suppressed_by_startup_blanking(void)
{
  system_monitor_state_t state;

  reset_fake_monitor();
  fake_motor_state.output_permille[MOTOR_ID_M2] = 50;
  fake_motor_state.fault_active[MOTOR_ID_M2] = 1U;

  SystemMonitor_Update();
  SystemMonitor_GetState(&state);

  require_int((state.latched_error_flags & SYSTEM_ERROR_DRV_FAULT) != 0U,
              "DRV fault latches during startup blank");
  require_int(fake_fault_stop != 0U, "DRV fault requests fault stop during startup blank");
}

static void test_adc_overcurrent_output_chatter_does_not_fault_when_software_fault_disabled(void)
{
  system_monitor_state_t state;

  reset_fake_monitor();
  fake_adc_state.current_a[MOTOR_ID_M2] = MOTOR_STALL_CURRENT_A + 1.0f;

  for (uint8_t i = 0U; i < MOTOR_OVERCURRENT_DEBOUNCE_COUNT + 20U; ++i)
  {
    fake_motor_state.output_permille[MOTOR_ID_M2] = ((i & 1U) == 0U) ? 1 : 0;
    update_and_advance(20U);
  }
  SystemMonitor_GetState(&state);

  require_int((state.latched_error_flags & SYSTEM_ERROR_M2_OVERCURRENT) == 0U,
              "disabled ADC software overcurrent ignores output chatter");
  require_int(fake_fault_stop == 0U, "disabled ADC software overcurrent chatter keeps fault stop clear");
}

int main(void)
{
  test_adc_overcurrent_does_not_fault_stop_when_software_fault_disabled();
  test_drv_fault_is_not_suppressed_by_startup_blanking();
  test_adc_overcurrent_output_chatter_does_not_fault_when_software_fault_disabled();
  return 0;
}
