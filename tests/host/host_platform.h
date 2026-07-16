#ifndef HOST_PLATFORM_H
#define HOST_PLATFORM_H

#include <stdint.h>

/** Return non-zero while a host-test Platform critical section is active. */
uint8_t HostPlatform_CriticalActive(void);

#endif /* HOST_PLATFORM_H */
