#include "chassis_task_timing.h"

#include "cmsis_os2.h"

static uint32_t missed_count[CHASSIS_TASK_TIMING_COUNT];

uint32_t ChassisTaskTiming_NextWake(uint32_t previous_wake_ms,
                                    uint32_t now_ms,
                                    uint32_t period_ms,
                                    uint8_t *missed)
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

void ChassisTaskTiming_DelayUntil(chassis_task_timing_id_t task,
                                  uint32_t *next_wake_ms,
                                  uint32_t period_ms)
{
  uint8_t missed = 0U;
  uint32_t now_ms;

  if (next_wake_ms == 0 || period_ms == 0U)
  {
    return;
  }

  now_ms = osKernelGetTickCount();
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

void ChassisTaskTiming_Reset(void)
{
  for (uint32_t i = 0U; i < (uint32_t)CHASSIS_TASK_TIMING_COUNT; ++i)
  {
    missed_count[i] = 0U;
  }
}
