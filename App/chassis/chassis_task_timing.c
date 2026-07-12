#include "chassis_task_timing.h"

#include "chassis_config.h"
#include "cmsis_os2.h"

static uint32_t missed_count[CHASSIS_TASK_TIMING_COUNT];
static uint32_t heartbeat_ms[CHASSIS_TASK_TIMING_COUNT];
static uint32_t timeout_count[CHASSIS_TASK_TIMING_COUNT];
static uint8_t  timed_out[CHASSIS_TASK_TIMING_COUNT];

static const uint32_t task_timeout_ms[CHASSIS_TASK_TIMING_COUNT] = {
    CHASSIS_ADC_PERIOD_MS * 4U,
    CHASSIS_CONTROL_PERIOD_MS * 4U,
    UPPER_UART_TASK_PERIOD_MS * 8U,
    CHASSIS_IMU_PERIOD_MS * 8U,
    CHASSIS_LINE_PERIOD_MS * 8U,
    CHASSIS_ESP12F_PERIOD_MS * 8U,
    CHASSIS_PS2_PERIOD_MS * 4U,
    CHASSIS_LED_PERIOD_MS * 4U,
    OLED_TASK_PERIOD_MS * 4U,
};

uint32_t ChassisTaskTiming_NextWake(uint32_t previous_wake_ms, uint32_t now_ms, uint32_t period_ms, uint8_t *missed)
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

void ChassisTaskTiming_DelayUntil(chassis_task_timing_id_t task, uint32_t *next_wake_ms, uint32_t period_ms)
{
    uint8_t  missed = 0U;
    uint32_t now_ms;

    if (next_wake_ms == 0 || period_ms == 0U)
    {
        return;
    }

    now_ms = osKernelGetTickCount();
    ChassisTaskTiming_Heartbeat(task, now_ms);
    *next_wake_ms = ChassisTaskTiming_NextWake(*next_wake_ms, now_ms, period_ms, &missed);
    if (missed != 0U && (uint32_t)task < (uint32_t)CHASSIS_TASK_TIMING_COUNT)
    {
        missed_count[task]++;
    }
    (void)osDelayUntil(*next_wake_ms);
}

uint32_t ChassisTaskTiming_GetMissedCount(chassis_task_timing_id_t task)
{
    if ((uint32_t)task >= (uint32_t)CHASSIS_TASK_TIMING_COUNT)
    {
        return 0U;
    }
    return missed_count[task];
}

void ChassisTaskTiming_Heartbeat(chassis_task_timing_id_t task, uint32_t now_ms)
{
    uint32_t index = (uint32_t)task;

    if (index >= (uint32_t)CHASSIS_TASK_TIMING_COUNT)
    {
        return;
    }
    heartbeat_ms[index] = now_ms;
    timed_out[index]    = 0U;
}

void ChassisTaskTiming_UpdateTimeouts(uint32_t now_ms)
{
    for (uint32_t i = 0U; i < (uint32_t)CHASSIS_TASK_TIMING_COUNT; ++i)
    {
        if (heartbeat_ms[i] == 0U || task_timeout_ms[i] == 0U)
        {
            continue;
        }
        if ((uint32_t)(now_ms - heartbeat_ms[i]) > task_timeout_ms[i])
        {
            if (timed_out[i] == 0U)
            {
                timeout_count[i]++;
            }
            timed_out[i] = 1U;
        }
    }
}

void ChassisTaskTiming_GetHealth(chassis_task_health_t *health)
{
    if (health == 0)
    {
        return;
    }
    for (uint32_t i = 0U; i < (uint32_t)CHASSIS_TASK_TIMING_COUNT; ++i)
    {
        health->last_heartbeat_ms[i] = heartbeat_ms[i];
        health->timeout_count[i]     = timeout_count[i];
        health->timed_out[i]         = timed_out[i];
    }
}

uint16_t ChassisTaskTiming_GetTimeoutMask(void)
{
    uint16_t mask = 0U;

    for (uint32_t i = 0U; i < (uint32_t)CHASSIS_TASK_TIMING_COUNT; ++i)
    {
        if (timed_out[i] != 0U)
        {
            mask |= (uint16_t)(1U << i);
        }
    }
    return mask;
}

void ChassisTaskTiming_Reset(void)
{
    for (uint32_t i = 0U; i < (uint32_t)CHASSIS_TASK_TIMING_COUNT; ++i)
    {
        missed_count[i]  = 0U;
        heartbeat_ms[i]  = 0U;
        timeout_count[i] = 0U;
        timed_out[i]     = 0U;
    }
}
