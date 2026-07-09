#ifndef CURRENT_GUARD_H
#define CURRENT_GUARD_H

#include <stdint.h>

#include "adc_monitor.h"
#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t observe_over_limit[MOTOR_ID_COUNT];
  uint8_t soft_limit_would_apply[MOTOR_ID_COUNT];
  uint8_t soft_limit_applied[MOTOR_ID_COUNT];
  uint8_t fault_would_latch[MOTOR_ID_COUNT];
  uint8_t control_valid[MOTOR_ID_COUNT];
  int16_t applied_permille[MOTOR_ID_COUNT];
  uint32_t observe_over_limit_count[MOTOR_ID_COUNT];
  uint32_t fault_would_latch_count[MOTOR_ID_COUNT];
} current_guard_state_t;

void CurrentGuard_Init(void);
int16_t CurrentGuard_ApplyMotorLimit(motor_id_t motor,
                                     int16_t requested_permille,
                                     const adc_monitor_state_t *adc_state,
                                     uint32_t now_ms,
                                     uint8_t *limited);
void CurrentGuard_GetState(current_guard_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
