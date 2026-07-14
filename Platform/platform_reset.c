#include "platform_reset.h"

#include "main.h"

_Noreturn void PlatformReset_FatalStop(void)
{
    __disable_irq();
    for (;;)
    {
    }
}

void PlatformReset_SystemReset(void)
{
    NVIC_SystemReset();
}
