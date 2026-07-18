#ifndef STATUS_LED_ADAPTER_H
#define STATUS_LED_ADAPTER_H

#include <stdint.h>

typedef enum
{
    STATUS_LED_ADAPTER_NORMAL      = 0,
    STATUS_LED_ADAPTER_UPPER_LINK  = 1,
    STATUS_LED_ADAPTER_LOW_BATTERY = 2,
    STATUS_LED_ADAPTER_FAULT       = 3,
    STATUS_LED_ADAPTER_ESTOP       = 4,
    STATUS_LED_ADAPTER_CAL_RUNNING = 5,
    STATUS_LED_ADAPTER_CAL_OK      = 6,
    STATUS_LED_ADAPTER_CAL_APPLIED = 7,
    STATUS_LED_ADAPTER_CAL_FAILED  = 8
} status_led_adapter_mode_t;

/** @brief Set the LED pattern mode through the Adapter. */
void StatusLedAdapter_SetMode(status_led_adapter_mode_t mode);
/** @brief Advance the BSP-owned status LED pattern by one task period. */
void StatusLedAdapter_TaskStep(uint32_t elapsed_ms);

#endif /* STATUS_LED_ADAPTER_H */
