#include "platform_critical.h"
#include "platform_time.h"

#include <stdint.h>

static uint32_t host_critical_depth;

#if defined(__GNUC__)
__attribute__((weak)) uint32_t osKernelGetTickCount(void)
{
    return 0U;
}

__attribute__((weak)) int32_t osDelayUntil(uint32_t ticks)
{
    (void)ticks;
    return 0;
}

__attribute__((weak)) uint32_t HAL_GetTick(void)
{
    return osKernelGetTickCount();
}
#endif

platform_critical_state_t PlatformCritical_Enter(void)
{
    platform_critical_state_t previous = host_critical_depth;
    host_critical_depth++;
    return previous;
}

void PlatformCritical_Exit(platform_critical_state_t state)
{
    host_critical_depth = state;
}

uint8_t HostPlatform_CriticalActive(void)
{
    return (host_critical_depth != 0UL) ? 1U : 0U;
}

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
