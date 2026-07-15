#ifndef PLATFORM_RESET_H
#define PLATFORM_RESET_H

#include <stdint.h>

typedef enum
{
    PLATFORM_RESET_REASON_BROWNOUT = 0,
    PLATFORM_RESET_REASON_POWER_ON,
    PLATFORM_RESET_REASON_PIN,
    PLATFORM_RESET_REASON_SOFTWARE,
    PLATFORM_RESET_REASON_INDEPENDENT_WATCHDOG,
    PLATFORM_RESET_REASON_WINDOW_WATCHDOG,
    PLATFORM_RESET_REASON_LOW_POWER
} platform_reset_reason_t;

_Noreturn void PlatformReset_FatalStop(void);
void           PlatformReset_SystemReset(void);
/** Read the MCU reset-cause register without clearing it. */
uint32_t PlatformReset_ReadReasonFlags(void);
/** Clear the MCU reset-cause flags. */
void PlatformReset_ClearReasonFlags(void);
/** Test a normalized reset cause against a raw reset-cause register value. */
uint8_t PlatformReset_ReasonFlagSet(uint32_t flags, platform_reset_reason_t reason);

#endif
