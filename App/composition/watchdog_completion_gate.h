#ifndef WATCHDOG_COMPLETION_GATE_H
#define WATCHDOG_COMPLETION_GATE_H

#include <stdint.h>

typedef struct
{
    uint32_t generation;
    uint32_t completed_at_ms;
} task_completion_t;

typedef struct
{
    uint32_t motor_timeout_ms;
    uint32_t safety_timeout_ms;
    uint32_t hardware_timeout_ms;
} watchdog_gate_config_t;

typedef struct
{
    uint32_t consumed_motor_generation;
    uint32_t consumed_safety_generation;
} watchdog_gate_state_t;

typedef enum
{
    WATCHDOG_GATE_DENY = 0,
    WATCHDOG_GATE_ALLOW,
    WATCHDOG_GATE_INVALID_CONFIG
} watchdog_gate_result_t;

watchdog_gate_result_t WatchdogCompletionGate_Evaluate(uint32_t                      now_ms,
                                                       const task_completion_t      *motor,
                                                       const task_completion_t      *safety,
                                                       const watchdog_gate_config_t *config,
                                                       const watchdog_gate_state_t  *state);
void                   WatchdogCompletionGate_CommitFeed(watchdog_gate_state_t   *state,
                                                         const task_completion_t *motor,
                                                         const task_completion_t *safety);

#endif /* WATCHDOG_COMPLETION_GATE_H */
