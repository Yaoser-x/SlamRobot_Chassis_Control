#include "platform_time.h"

#include "cmsis_os2.h"
#include "stm32f4xx_hal.h"

uint32_t PlatformTime_NowMs(void)
{
    return HAL_GetTick();
}

uint32_t PlatformTime_TaskNowMs(void)
{
    return osKernelGetTickCount();
}

void PlatformTime_DelayUntil(uint32_t *next_wake_ms, uint32_t period_ms, uint8_t *deadline_missed)
{
    uint32_t now_ms;
    uint32_t target_ms;

    if (next_wake_ms == 0)
    {
        return;
    }

    now_ms    = osKernelGetTickCount();
    target_ms = *next_wake_ms + period_ms;
    if ((int32_t)(now_ms - target_ms) >= 0)
    {
        target_ms = now_ms + period_ms;
        if (deadline_missed != 0)
        {
            *deadline_missed = 1U;
        }
    }
    else if (deadline_missed != 0)
    {
        *deadline_missed = 0U;
    }

    *next_wake_ms = target_ms;
    (void)osDelayUntil(target_ms);
}
