#include "watchdog_completion_gate.h"

static uint8_t WatchdogCompletionGate_ConfigValid(const watchdog_gate_config_t *config)
{
    return (config != 0 && config->motor_timeout_ms != 0UL && config->motor_timeout_ms < config->safety_timeout_ms
            && config->safety_timeout_ms < config->hardware_timeout_ms)
               ? 1U
               : 0U;
}

watchdog_gate_result_t WatchdogCompletionGate_Evaluate(uint32_t                      now_ms,
                                                       const task_completion_t      *motor,
                                                       const task_completion_t      *safety,
                                                       const watchdog_gate_config_t *config,
                                                       const watchdog_gate_state_t  *state)
{
    if (motor == 0 || safety == 0 || state == 0 || WatchdogCompletionGate_ConfigValid(config) == 0U)
    {
        return WATCHDOG_GATE_INVALID_CONFIG;
    }
    if (motor->generation == 0UL || safety->generation == 0UL || motor->generation == state->consumed_motor_generation
        || safety->generation == state->consumed_safety_generation)
    {
        return WATCHDOG_GATE_DENY;
    }
    if ((uint32_t)(now_ms - motor->completed_at_ms) > config->motor_timeout_ms
        || (uint32_t)(now_ms - safety->completed_at_ms) > config->safety_timeout_ms)
    {
        return WATCHDOG_GATE_DENY;
    }
    return WATCHDOG_GATE_ALLOW;
}

void WatchdogCompletionGate_CommitFeed(watchdog_gate_state_t   *state,
                                       const task_completion_t *motor,
                                       const task_completion_t *safety)
{
    if (state == 0 || motor == 0 || safety == 0)
    {
        return;
    }
    state->consumed_motor_generation  = motor->generation;
    state->consumed_safety_generation = safety->generation;
}
