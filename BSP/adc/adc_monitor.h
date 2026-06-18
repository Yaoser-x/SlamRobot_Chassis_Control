#ifndef ADC_MONITOR_H
#define ADC_MONITOR_H

#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint16_t raw_current[MOTOR_ID_COUNT];
  uint16_t current_zero_raw[MOTOR_ID_COUNT];
  uint16_t raw_battery;
  uint16_t raw_left_current;
  uint16_t raw_right_current;
  float current_a[MOTOR_ID_COUNT];
  float battery_voltage;
  float left_current_a;
  float right_current_a;
  uint16_t current_zero_sample_count;
  uint8_t samples_ready;
  uint8_t current_zero_valid;
  uint8_t current_valid;
} adc_monitor_state_t;

void AdcMonitor_Init(void);
void AdcMonitor_Update(void);
void AdcMonitor_GetState(adc_monitor_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
