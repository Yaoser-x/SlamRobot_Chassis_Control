#include "status_led_adapter.h"

#include "status_led_driver.h"

void StatusLedAdapter_TaskStep(uint32_t elapsed_ms)
{
    StatusLedDriver_TaskStep(elapsed_ms);
}
