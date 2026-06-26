#ifndef HOST_CMSIS_OS2_H
#define HOST_CMSIS_OS2_H

#include <stdint.h>

typedef int32_t osStatus_t;

uint32_t osKernelGetTickCount(void);
osStatus_t osDelayUntil(uint32_t ticks);

#endif
