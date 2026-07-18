#include "status_led_adapter.h"

#include "status_led_driver.h"

_Static_assert((int)STATUS_LED_ADAPTER_NORMAL == (int)STATUS_LED_DRIVER_NORMAL
                   && (int)STATUS_LED_ADAPTER_UPPER_LINK == (int)STATUS_LED_DRIVER_UPPER_LINK
                   && (int)STATUS_LED_ADAPTER_LOW_BATTERY == (int)STATUS_LED_DRIVER_LOW_BATTERY
                   && (int)STATUS_LED_ADAPTER_FAULT == (int)STATUS_LED_DRIVER_FAULT
                   && (int)STATUS_LED_ADAPTER_ESTOP == (int)STATUS_LED_DRIVER_ESTOP
                   && (int)STATUS_LED_ADAPTER_CAL_RUNNING == (int)STATUS_LED_DRIVER_CAL_RUNNING
                   && (int)STATUS_LED_ADAPTER_CAL_OK == (int)STATUS_LED_DRIVER_CAL_OK
                   && (int)STATUS_LED_ADAPTER_CAL_APPLIED == (int)STATUS_LED_DRIVER_CAL_APPLIED,
               "status_led_adapter_mode_t values must match status_led_driver_mode_t");

void StatusLedAdapter_SetMode(status_led_adapter_mode_t mode)
{
    StatusLedDriver_SetMode((status_led_driver_mode_t)mode);
}

void StatusLedAdapter_TaskStep(uint32_t elapsed_ms)
{
    StatusLedDriver_TaskStep(elapsed_ms);
}
