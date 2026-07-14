#include "platform_task_event.h"

#include "cmsis_os2.h"

static osThreadId_t task_handles[PLATFORM_TASK_EVENT_COUNT];

void PlatformTaskEvent_Bind(platform_task_event_t event, void *task_handle)
{
    if ((uint32_t)event < PLATFORM_TASK_EVENT_COUNT)
    {
        task_handles[event] = (osThreadId_t)task_handle;
    }
}

void PlatformTaskEvent_SetFromIsr(platform_task_event_t event)
{
    if ((uint32_t)event < PLATFORM_TASK_EVENT_COUNT && task_handles[event] != 0)
    {
        (void)osThreadFlagsSet(task_handles[event], 1UL << (uint32_t)event);
    }
}

uint32_t PlatformTaskEvent_Wait(platform_task_event_t event, uint32_t timeout_ms)
{
    if ((uint32_t)event >= PLATFORM_TASK_EVENT_COUNT)
    {
        return osFlagsErrorParameter;
    }
    return osThreadFlagsWait(1UL << (uint32_t)event, osFlagsWaitAny, timeout_ms);
}
