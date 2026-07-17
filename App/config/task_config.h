#ifndef TASK_CONFIG_H
#define TASK_CONFIG_H

#include <stdint.h>

typedef enum
{
    APP_TASK_SAFETY = 0,
    APP_TASK_MOTOR,
    APP_TASK_HOST,
    APP_TASK_IMU,
    APP_TASK_LINE,
    APP_TASK_ESP12F,
    APP_TASK_PS2,
    APP_TASK_LED,
    APP_TASK_OLED,
    APP_TASK_DEBUG,
    APP_TASK_COUNT
} app_task_id_t;

typedef enum
{
    APP_TASK_PRIORITY_LOW = 0,
    APP_TASK_PRIORITY_BELOW_NORMAL,
    APP_TASK_PRIORITY_NORMAL,
    APP_TASK_PRIORITY_ABOVE_NORMAL,
    APP_TASK_PRIORITY_HIGH
} app_task_priority_t;

/** @brief Beta5 frozen task attributes.
 *
 * Task period, stack size, priority, and event-driven flag are frozen in Beta5.
 * freertos.c creates tasks with hardcoded osThreadAttr_t values and every task
 * loop reads RobotConfig_GetDefault()-&gt;tasks[...].period_ms.  The values in
 * robot_config_t.tasks[] are documentation of the frozen intent; they are
 * NOT runtime-configurable parameters.  RobotConfig_TasksValid() enforces
 * exact equality with the defaults. */
typedef struct
{
    uint32_t            period_ms;
    uint32_t            stack_size_bytes;
    app_task_priority_t priority;
    uint8_t             event_driven;
} app_task_config_t;

#endif /* TASK_CONFIG_H */
