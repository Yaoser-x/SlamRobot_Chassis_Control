#include "platform_watchdog.h"

#include "iwdg.h"

void PlatformWatchdog_Init(void)
{
    MX_IWDG_Init();
}

void PlatformWatchdog_Feed(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}
