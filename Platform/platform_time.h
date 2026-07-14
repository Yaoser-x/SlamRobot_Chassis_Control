#ifndef PLATFORM_TIME_H
#define PLATFORM_TIME_H

#include <stdint.h>

uint32_t PlatformTime_NowMs(void);
uint32_t PlatformTime_TaskNowMs(void);
void     PlatformTime_DelayUntil(uint32_t *next_wake_ms, uint32_t period_ms, uint8_t *deadline_missed);

#endif
