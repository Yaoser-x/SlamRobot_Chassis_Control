#ifndef CHASSIS_TASK_TIMING_H
#define CHASSIS_TASK_TIMING_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  CHASSIS_TASK_TIMING_SAFETY = 0,
  CHASSIS_TASK_TIMING_MOTOR,
  CHASSIS_TASK_TIMING_RPI,
  CHASSIS_TASK_TIMING_IMU,
  CHASSIS_TASK_TIMING_LINE,
  CHASSIS_TASK_TIMING_ESP,
  CHASSIS_TASK_TIMING_PS2,
  CHASSIS_TASK_TIMING_LED,
  CHASSIS_TASK_TIMING_OLED,
  CHASSIS_TASK_TIMING_COUNT
} chassis_task_timing_id_t;

typedef struct
{
  uint32_t last_heartbeat_ms[CHASSIS_TASK_TIMING_COUNT];
  uint32_t timeout_count[CHASSIS_TASK_TIMING_COUNT];
  uint8_t timed_out[CHASSIS_TASK_TIMING_COUNT];
} chassis_task_health_t;

uint32_t ChassisTaskTiming_NextWake(uint32_t previous_wake_ms,
                                    uint32_t now_ms,
                                    uint32_t period_ms,
                                    uint8_t *missed);
void ChassisTaskTiming_DelayUntil(chassis_task_timing_id_t task,
                                  uint32_t *next_wake_ms,
                                  uint32_t period_ms);
uint32_t ChassisTaskTiming_GetMissedCount(chassis_task_timing_id_t task);
void ChassisTaskTiming_Heartbeat(chassis_task_timing_id_t task, uint32_t now_ms);
void ChassisTaskTiming_UpdateTimeouts(uint32_t now_ms);
void ChassisTaskTiming_GetHealth(chassis_task_health_t *health);
uint16_t ChassisTaskTiming_GetTimeoutMask(void);
void ChassisTaskTiming_Reset(void);

#ifdef __cplusplus
}
#endif

#endif
