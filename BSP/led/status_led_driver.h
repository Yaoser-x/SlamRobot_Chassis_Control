#ifndef STATUS_LED_DRIVER_H
#define STATUS_LED_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        STATUS_LED_DRIVER_NORMAL      = 0,
        STATUS_LED_DRIVER_UPPER_LINK  = 1,
        STATUS_LED_DRIVER_LOW_BATTERY = 2,
        STATUS_LED_DRIVER_FAULT       = 3,
        STATUS_LED_DRIVER_ESTOP       = 4,
        STATUS_LED_DRIVER_CAL_RUNNING = 5,
        STATUS_LED_DRIVER_CAL_OK      = 6,
        STATUS_LED_DRIVER_CAL_APPLIED = 7,
        STATUS_LED_DRIVER_CAL_FAILED  = 8
    } status_led_driver_mode_t;

    void StatusLedDriver_Init(void);
    void StatusLedDriver_SetMode(status_led_driver_mode_t mode);
    void StatusLedDriver_TaskStep(uint32_t period_ms);

#ifdef __cplusplus
}
#endif

#endif
