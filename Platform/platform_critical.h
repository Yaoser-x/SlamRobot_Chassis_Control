#ifndef PLATFORM_CRITICAL_H
#define PLATFORM_CRITICAL_H

#include <stdint.h>

typedef uint32_t platform_critical_state_t;

platform_critical_state_t PlatformCritical_Enter(void);
void                      PlatformCritical_Exit(platform_critical_state_t state);

#endif
