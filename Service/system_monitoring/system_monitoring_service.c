#include "system_monitoring_service.h"

#include "platform_critical.h"
#include "platform_time.h"

static system_monitoring_config_t system_config;
static system_monitoring_status_t system_status;
static system_monitoring_status_t system_next;
static uint8_t                    system_initialized;

static uint8_t SystemMonitoring_ValidTask(system_monitoring_task_id_t task)
{
    return ((uint32_t)task < (uint32_t)SYSTEM_MONITORING_TASK_COUNT) ? 1U : 0U;
}

uint8_t SystemMonitoring_Init(const system_monitoring_config_t *config, uint32_t reset_reason_flags)
{
    system_monitoring_status_t next = {0};
    platform_critical_state_t  critical;

    if (config == 0)
    {
        return 0U;
    }
    for (uint32_t index = 0U; index < SYSTEM_MONITORING_TASK_COUNT; ++index)
    {
        if (config->task_timeout_ms[index] == 0UL)
        {
            return 0U;
        }
    }
    next.reset_reason_flags = reset_reason_flags;
    next.generation         = 1UL;
    critical                = PlatformCritical_Enter();
    system_config           = *config;
    system_status           = next;
    system_initialized      = 1U;
    PlatformCritical_Exit(critical);
    return 1U;
}

uint8_t SystemMonitoring_IsInitialized(void)
{
    return system_initialized;
}

uint32_t SystemMonitoring_NextWake(uint32_t previous_wake_ms, uint32_t now_ms, uint32_t period_ms, uint8_t *missed)
{
    uint32_t next_wake = previous_wake_ms + period_ms;

    if (missed != 0)
    {
        *missed = 0U;
    }
    if ((int32_t)(now_ms - next_wake) >= 0)
    {
        next_wake = now_ms + period_ms;
        if (missed != 0)
        {
            *missed = 1U;
        }
    }
    return next_wake;
}

void SystemMonitoring_DelayUntil(system_monitoring_task_id_t task, uint32_t *next_wake_ms, uint32_t period_ms)
{
    uint8_t                   missed = 0U;
    uint32_t                  now_ms;
    platform_critical_state_t critical;

    if (next_wake_ms == 0 || period_ms == 0U)
    {
        return;
    }
    now_ms = PlatformTime_TaskNowMs();
    SystemMonitoring_Heartbeat(task, now_ms);
    PlatformTime_DelayUntil(next_wake_ms, period_ms, &missed);
    if (missed != 0U && SystemMonitoring_ValidTask(task) != 0U)
    {
        critical    = PlatformCritical_Enter();
        system_next = system_status;
        system_next.task_health.missed_count[task]++;
        system_next.generation++;
        system_status = system_next;
        PlatformCritical_Exit(critical);
    }
}

void SystemMonitoring_Heartbeat(system_monitoring_task_id_t task, uint32_t now_ms)
{
    platform_critical_state_t critical;

    if (SystemMonitoring_ValidTask(task) == 0U)
    {
        return;
    }
    critical                                        = PlatformCritical_Enter();
    system_next                                     = system_status;
    system_next.task_health.last_heartbeat_ms[task] = now_ms;
    system_next.task_health.timed_out[task]         = 0U;
    system_next.generation++;
    system_status = system_next;
    PlatformCritical_Exit(critical);
}

void SystemMonitoring_UpdateTimeouts(uint32_t now_ms)
{
    platform_critical_state_t critical;

    critical    = PlatformCritical_Enter();
    system_next = system_status;
    for (uint32_t index = 0U; index < SYSTEM_MONITORING_TASK_COUNT; ++index)
    {
        uint32_t heartbeat = system_next.task_health.last_heartbeat_ms[index];
        if (heartbeat == 0UL)
        {
            continue;
        }
        if ((uint32_t)(now_ms - heartbeat) > system_config.task_timeout_ms[index])
        {
            if (system_next.task_health.timed_out[index] == 0U)
            {
                system_next.task_health.timeout_count[index]++;
            }
            system_next.task_health.timed_out[index] = 1U;
        }
    }
    system_next.generation++;
    system_status = system_next;
    PlatformCritical_Exit(critical);
}

void SystemMonitoring_ResetTaskHealth(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    system_next             = system_status;
    system_next.task_health = (system_monitoring_task_health_t){0};
    system_next.generation++;
    system_status = system_next;
    PlatformCritical_Exit(critical);
}

uint32_t SystemMonitoring_GetMissedCount(system_monitoring_task_id_t task)
{
    platform_critical_state_t critical;
    uint32_t                  count;

    if (SystemMonitoring_ValidTask(task) == 0U)
    {
        return 0UL;
    }
    critical = PlatformCritical_Enter();
    count    = system_status.task_health.missed_count[task];
    PlatformCritical_Exit(critical);
    return count;
}

uint16_t SystemMonitoring_GetTimeoutMask(void)
{
    platform_critical_state_t critical;
    uint16_t                  mask = 0U;

    critical = PlatformCritical_Enter();
    for (uint32_t index = 0U; index < SYSTEM_MONITORING_TASK_COUNT; ++index)
    {
        if (system_status.task_health.timed_out[index] != 0U)
        {
            mask |= (uint16_t)(1U << index);
        }
    }
    PlatformCritical_Exit(critical);
    return mask;
}

uint32_t SystemMonitoring_GetTaskHealth(system_monitoring_task_health_t *health)
{
    platform_critical_state_t critical;
    uint32_t                  generation;

    if (health == 0)
    {
        return 0UL;
    }
    critical   = PlatformCritical_Enter();
    *health    = system_status.task_health;
    generation = system_status.generation;
    PlatformCritical_Exit(critical);
    return generation;
}

uint32_t SystemMonitoring_GetStatus(system_monitoring_status_t *status)
{
    platform_critical_state_t critical;

    if (status == 0)
    {
        return 0UL;
    }
    critical = PlatformCritical_Enter();
    *status  = system_status;
    PlatformCritical_Exit(critical);
    return status->generation;
}

void SystemMonitoring_SetModuleHealth(const system_monitoring_module_health_t *modules)
{
    platform_critical_state_t critical;

    if (modules == 0)
    {
        return;
    }
    critical            = PlatformCritical_Enter();
    system_next         = system_status;
    system_next.modules = *modules;
    system_next.generation++;
    system_status = system_next;
    PlatformCritical_Exit(critical);
}

void SystemMonitoring_CaptureResetReason(uint32_t flags)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    system_next                    = system_status;
    system_next.reset_reason_flags = flags;
    system_next.generation++;
    system_status = system_next;
    PlatformCritical_Exit(critical);
}

uint32_t SystemMonitoring_GetResetReason(void)
{
    platform_critical_state_t critical;
    uint32_t                  flags;

    critical = PlatformCritical_Enter();
    flags    = system_status.reset_reason_flags;
    PlatformCritical_Exit(critical);
    return flags;
}
