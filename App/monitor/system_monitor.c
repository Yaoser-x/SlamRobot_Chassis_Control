#include "system_monitor.h"

#include "adc_monitor.h"
#include "chassis_config.h"
#include "chassis_layout.h"
#include "control_manager.h"
#include "cmsis_os2.h"
#include "encoder_driver.h"
#include "main.h"
#include "motor_driver.h"

static system_monitor_state_t monitor_state;
static uint8_t overcurrent_count[MOTOR_ID_COUNT];
static uint8_t motor_output_active[MOTOR_ID_COUNT];
static uint8_t startup_blank_armed[MOTOR_ID_COUNT];
static uint32_t overcurrent_blank_until_ms[MOTOR_ID_COUNT];
static uint32_t inactive_since_ms[MOTOR_ID_COUNT];

static const uint32_t overcurrent_flags[MOTOR_ID_COUNT] = {
  SYSTEM_ERROR_M1_OVERCURRENT,
  SYSTEM_ERROR_M2_OVERCURRENT,
  SYSTEM_ERROR_M3_OVERCURRENT,
  SYSTEM_ERROR_M4_OVERCURRENT,
};

static uint8_t SystemMonitor_CurrentBelowFaultThreshold(float current_a)
{
  return (current_a <= MOTOR_STALL_CURRENT_A) ? 1U : 0U;
}

static uint8_t SystemMonitor_TimeReached(uint32_t now_ms, uint32_t deadline_ms)
{
  return (((int32_t)(now_ms - deadline_ms)) >= 0) ? 1U : 0U;
}

static void SystemMonitor_UpdateOvercurrentCounters(const adc_monitor_state_t *adc_state,
                                                    const uint8_t blanked[MOTOR_ID_COUNT],
                                                    const uint8_t previous_count[MOTOR_ID_COUNT],
                                                    uint8_t next_count[MOTOR_ID_COUNT],
                                                    uint32_t *new_latched_flags)
{
  if (MOTOR_ADC_OVERCURRENT_FAULT_ENABLED == 0U)
  {
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
      next_count[i] = 0U;
    }
    return;
  }

  if (adc_state->current_valid == 0U)
  {
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
      next_count[i] = 0U;
    }
    return;
  }

  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    next_count[i] = previous_count[i];
    if (ChassisLayout_MotorEnabled((motor_id_t)i) == 0U)
    {
      next_count[i] = 0U;
      continue;
    }
    if (blanked[i] != 0U)
    {
      next_count[i] = 0U;
      continue;
    }
    if (adc_state->current_a[i] > MOTOR_STALL_CURRENT_A)
    {
      if (next_count[i] < MOTOR_OVERCURRENT_DEBOUNCE_COUNT)
      {
        next_count[i]++;
      }
      if (next_count[i] >= MOTOR_OVERCURRENT_DEBOUNCE_COUNT && new_latched_flags != 0)
      {
        *new_latched_flags |= overcurrent_flags[i];
      }
    }
    else
    {
      next_count[i] = 0U;
    }
  }
}

void SystemMonitor_Init(void)
{
  monitor_state = (system_monitor_state_t){0};
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    overcurrent_count[i] = 0U;
    motor_output_active[i] = 0U;
    startup_blank_armed[i] = 1U;
    overcurrent_blank_until_ms[i] = 0U;
    inactive_since_ms[i] = 0U;
  }
}

void SystemMonitor_Update(void)
{
  adc_monitor_state_t adc_state;
  encoder_state_t encoder_state;
  motor_driver_state_t motor_state;
  uint8_t estop_active;
  uint8_t fault_stop_active;
  uint8_t active_source;
  system_monitor_state_t next_state;
  uint8_t previous_overcurrent_count[MOTOR_ID_COUNT];
  uint8_t next_overcurrent_count[MOTOR_ID_COUNT];
  uint8_t overcurrent_blanked[MOTOR_ID_COUNT];
  uint32_t new_latched_flags = 0U;
  uint32_t latched_after_commit;
  uint8_t request_fault_stop = 0U;
  uint32_t now_ms;
  uint32_t primask;

  now_ms = osKernelGetTickCount();
  AdcMonitor_Update();
  MotorDriver_UpdateFaults();
  AdcMonitor_GetState(&adc_state);
  EncoderDriver_GetState(&encoder_state);
  MotorDriver_GetState(&motor_state);
  estop_active = ControlManager_IsEmergencyStop();
  fault_stop_active = ControlManager_IsFaultStop();
  active_source = ControlManager_GetActiveSource();

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    previous_overcurrent_count[i] = overcurrent_count[i];
  }
  __set_PRIMASK(primask);

  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    uint8_t active = (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U &&
                      motor_state.output_permille[i] != 0) ? 1U : 0U;
    if (active != 0U)
    {
      inactive_since_ms[i] = now_ms;
      if (motor_output_active[i] == 0U && startup_blank_armed[i] != 0U)
      {
        overcurrent_blank_until_ms[i] = now_ms + MOTOR_OVERCURRENT_STARTUP_BLANK_MS;
        startup_blank_armed[i] = 0U;
      }
      motor_output_active[i] = 1U;
    }
    else
    {
      if (motor_output_active[i] != 0U)
      {
        inactive_since_ms[i] = now_ms;
      }
      motor_output_active[i] = 0U;
      if (SystemMonitor_TimeReached(now_ms, inactive_since_ms[i] + MOTOR_OVERCURRENT_STARTUP_REARM_MS) != 0U)
      {
        startup_blank_armed[i] = 1U;
        overcurrent_blank_until_ms[i] = 0U;
      }
    }
    overcurrent_blanked[i] = (startup_blank_armed[i] == 0U &&
                              SystemMonitor_TimeReached(now_ms, overcurrent_blank_until_ms[i]) == 0U) ? 1U : 0U;
  }

  next_state = (system_monitor_state_t){0};
  next_state.battery_voltage = adc_state.battery_voltage;
  next_state.left_current_a = adc_state.left_current_a;
  next_state.right_current_a = adc_state.right_current_a;
  next_state.control_mode = active_source;
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    next_state.motor_current_a[i] = adc_state.current_a[i];
  }

  SystemMonitor_UpdateOvercurrentCounters(&adc_state,
                                          overcurrent_blanked,
                                          previous_overcurrent_count,
                                          next_overcurrent_count,
                                          &new_latched_flags);
  if (ADC_MONITOR_CALIBRATION_ENABLED != 0U &&
      BATTERY_LOW_MONITOR_ENABLED != 0U &&
      next_state.battery_voltage > 0.1f &&
      next_state.battery_voltage < BATTERY_LOW_WARN_V)
  {
    next_state.error_flags |= SYSTEM_ERROR_LOW_BATTERY;
  }
  if (estop_active != 0U)
  {
    next_state.error_flags |= SYSTEM_ERROR_ESTOP;
  }
  if (fault_stop_active != 0U)
  {
    next_state.error_flags |= SYSTEM_ERROR_FAULT_STOP;
  }
  if (encoder_state.speed_valid_all == 0U)
  {
    next_state.error_flags |= SYSTEM_ERROR_ENCODER_INVALID;
  }
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U &&
        motor_state.fault_active[i] != 0U)
    {
      new_latched_flags |= SYSTEM_ERROR_DRV_FAULT;
    }
  }

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    overcurrent_count[i] = next_overcurrent_count[i];
  }
  monitor_state.battery_voltage = next_state.battery_voltage;
  monitor_state.left_current_a = next_state.left_current_a;
  monitor_state.right_current_a = next_state.right_current_a;
  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    monitor_state.motor_current_a[i] = next_state.motor_current_a[i];
  }
  monitor_state.control_mode = next_state.control_mode;
  monitor_state.latched_error_flags |= new_latched_flags;
  latched_after_commit = monitor_state.latched_error_flags;
  monitor_state.error_flags = next_state.error_flags | latched_after_commit;
  if ((latched_after_commit &
       (SYSTEM_ERROR_LEFT_OVERCURRENT | SYSTEM_ERROR_RIGHT_OVERCURRENT | SYSTEM_ERROR_DRV_FAULT)) != 0U)
  {
    request_fault_stop = 1U;
    monitor_state.error_flags |= SYSTEM_ERROR_FAULT_STOP;
  }
  __set_PRIMASK(primask);

  if (request_fault_stop != 0U)
  {
    ControlManager_SetFaultStop(1U);
  }
}

void SystemMonitor_GetState(system_monitor_state_t *state)
{
  uint32_t primask;

  if (state == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *state = monitor_state;
  __set_PRIMASK(primask);
}

void SystemMonitor_ClearLatchedFaults(uint32_t mask)
{
  uint32_t clearable = mask;
  system_monitor_state_t snapshot;
  motor_driver_state_t motor_state;
  uint8_t clear_fault_stop = 0U;
  uint32_t latched_after_clear;
  uint32_t primask;

  MotorDriver_GetState(&motor_state);

  primask = __get_PRIMASK();
  __disable_irq();
  snapshot = monitor_state;
  __set_PRIMASK(primask);

  for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U &&
        (mask & overcurrent_flags[i]) != 0U &&
        SystemMonitor_CurrentBelowFaultThreshold(snapshot.motor_current_a[i]) == 0U)
    {
      clearable &= ~overcurrent_flags[i];
    }
  }
  if ((mask & SYSTEM_ERROR_DRV_FAULT) != 0U)
  {
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
      if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U &&
          motor_state.fault_active[i] != 0U)
      {
        clearable &= ~SYSTEM_ERROR_DRV_FAULT;
      }
    }
  }

  primask = __get_PRIMASK();
  __disable_irq();
  monitor_state.latched_error_flags &= ~clearable;
  latched_after_clear = monitor_state.latched_error_flags;
  if ((latched_after_clear &
       (SYSTEM_ERROR_LEFT_OVERCURRENT | SYSTEM_ERROR_RIGHT_OVERCURRENT | SYSTEM_ERROR_DRV_FAULT)) == 0U)
  {
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
      overcurrent_count[i] = 0U;
    }
    clear_fault_stop = 1U;
  }

  __set_PRIMASK(primask);

  if (clear_fault_stop != 0U)
  {
    ControlManager_SetFaultStop(0U);
  }
}

uint8_t SystemMonitor_HasLatchedFault(void)
{
  uint8_t has_fault;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  has_fault = (monitor_state.latched_error_flags != 0U) ? 1U : 0U;
  __set_PRIMASK(primask);
  return has_fault;
}
