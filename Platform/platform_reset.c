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

uint32_t PlatformReset_ReadReasonFlags(void)
{
    return RCC->CSR;
}

void PlatformReset_ClearReasonFlags(void)
{
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

uint8_t PlatformReset_ReasonFlagSet(uint32_t flags, platform_reset_reason_t reason)
{
    uint32_t mask;

    switch (reason)
    {
        case PLATFORM_RESET_REASON_BROWNOUT:
            mask = RCC_CSR_BORRSTF;
            break;
        case PLATFORM_RESET_REASON_POWER_ON:
            mask = RCC_CSR_PORRSTF;
            break;
        case PLATFORM_RESET_REASON_PIN:
            mask = RCC_CSR_PINRSTF;
            break;
        case PLATFORM_RESET_REASON_SOFTWARE:
            mask = RCC_CSR_SFTRSTF;
            break;
        case PLATFORM_RESET_REASON_INDEPENDENT_WATCHDOG:
            mask = RCC_CSR_IWDGRSTF;
            break;
        case PLATFORM_RESET_REASON_WINDOW_WATCHDOG:
            mask = RCC_CSR_WWDGRSTF;
            break;
        case PLATFORM_RESET_REASON_LOW_POWER:
            mask = RCC_CSR_LPWRRSTF;
            break;
        default:
            return 0U;
    }
    return ((flags & mask) != 0U) ? 1U : 0U;
}
