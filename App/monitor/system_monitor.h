#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include <stdint.h>

#include "chassis_task_timing.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SYSTEM_ERROR_LOW_BATTERY      (1UL << 0)
#define SYSTEM_ERROR_M1_OVERCURRENT   (1UL << 1)
#define SYSTEM_ERROR_M2_OVERCURRENT   (1UL << 2)
#define SYSTEM_ERROR_M3_OVERCURRENT   (1UL << 3)
#define SYSTEM_ERROR_M4_OVERCURRENT   (1UL << 4)
#define SYSTEM_ERROR_LEFT_OVERCURRENT (SYSTEM_ERROR_M1_OVERCURRENT | SYSTEM_ERROR_M2_OVERCURRENT)
#define SYSTEM_ERROR_RIGHT_OVERCURRENT (SYSTEM_ERROR_M3_OVERCURRENT | SYSTEM_ERROR_M4_OVERCURRENT)
#define SYSTEM_ERROR_ESTOP            (1UL << 5)
#define SYSTEM_ERROR_FAULT_STOP       (1UL << 6)
#define SYSTEM_ERROR_ENCODER_INVALID  (1UL << 7)
#define SYSTEM_ERROR_DRV_FAULT        (1UL << 8)
#define SYSTEM_ERROR_TIM_BREAK        (1UL << 9)
#define SYSTEM_ERROR_ENCODER_FEEDBACK_LOST (1UL << 17)
#define SYSTEM_ERROR_BATTERY_CRITICAL (1UL << 18)

typedef struct
{
  float battery_voltage;
  float motor_current_a[4];
  float left_current_a;
  float right_current_a;
  uint32_t current_observe_over_limit_count[4];
  uint32_t current_fault_would_latch_count[4];
  uint32_t error_flags;
  uint32_t latched_error_flags;
  uint32_t task_last_heartbeat_ms[CHASSIS_TASK_TIMING_COUNT];
  uint32_t task_timeout_count[CHASSIS_TASK_TIMING_COUNT];
  uint8_t task_timed_out[CHASSIS_TASK_TIMING_COUNT];
  uint8_t control_mode;
  uint8_t current_control_valid;
  uint8_t current_control_valid_mask;
} system_monitor_state_t;

void SystemMonitor_Init(void);
void SystemMonitor_Update(void);
void SystemMonitor_GetState(system_monitor_state_t *state);
void SystemMonitor_ClearLatchedFaults(uint32_t mask);
uint8_t SystemMonitor_HasLatchedFault(void);
void SystemMonitor_LatchEncoderFeedbackFault(void);

#ifdef __cplusplus
}
#endif

#endif
