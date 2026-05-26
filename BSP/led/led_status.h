#ifndef LED_STATUS_H
#define LED_STATUS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  LED_STATUS_NORMAL = 0,
  LED_STATUS_UPPER_LINK = 1,
  LED_STATUS_LOW_BATTERY = 2,
  LED_STATUS_FAULT = 3,
  LED_STATUS_ESTOP = 4
} led_status_mode_t;

void LedStatus_Init(void);
void LedStatus_SetMode(led_status_mode_t mode);
void LedStatus_TaskStep(uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif
