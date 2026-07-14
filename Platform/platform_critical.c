#include "platform_critical.h"

#include "main.h"

platform_critical_state_t PlatformCritical_Enter(void)
{
    platform_critical_state_t state = __get_PRIMASK();
    __disable_irq();
    return state;
}

void PlatformCritical_Exit(platform_critical_state_t state)
{
    __set_PRIMASK(state);
}
