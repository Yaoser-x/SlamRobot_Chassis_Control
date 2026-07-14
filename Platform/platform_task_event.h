#ifndef PLATFORM_TASK_EVENT_H
#define PLATFORM_TASK_EVENT_H

#include <stdint.h>

typedef enum
{
    PLATFORM_TASK_EVENT_IMU_DRDY = 0,
    PLATFORM_TASK_EVENT_COUNT
} platform_task_event_t;

void     PlatformTaskEvent_Bind(platform_task_event_t event, void *task_handle);
void     PlatformTaskEvent_SetFromIsr(platform_task_event_t event);
uint32_t PlatformTaskEvent_Wait(platform_task_event_t event, uint32_t timeout_ms);

#endif
