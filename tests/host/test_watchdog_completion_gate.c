#include "watchdog_completion_gate.h"

#include <assert.h>
#include <stdint.h>

static const watchdog_gate_config_t config = {
    .motor_timeout_ms    = 20U,
    .safety_timeout_ms   = 40U,
    .hardware_timeout_ms = 800U,
};

static void test_both_tasks_must_advance(void)
{
    task_completion_t     motor  = {1U, 100U};
    task_completion_t     safety = {1U, 100U};
    watchdog_gate_state_t state  = {0};

    assert(WatchdogCompletionGate_Evaluate(100U, &motor, &safety, &config, &state) == WATCHDOG_GATE_ALLOW);
    WatchdogCompletionGate_CommitFeed(&state, &motor, &safety);
    motor.generation++;
    motor.completed_at_ms = 110U;
    assert(WatchdogCompletionGate_Evaluate(110U, &motor, &safety, &config, &state) == WATCHDOG_GATE_DENY);
    safety.generation++;
    safety.completed_at_ms = 120U;
    assert(WatchdogCompletionGate_Evaluate(120U, &motor, &safety, &config, &state) == WATCHDOG_GATE_ALLOW);
}

static void test_evaluate_does_not_consume(void)
{
    task_completion_t     motor  = {7U, 200U};
    task_completion_t     safety = {9U, 200U};
    watchdog_gate_state_t state  = {0};

    assert(WatchdogCompletionGate_Evaluate(200U, &motor, &safety, &config, &state) == WATCHDOG_GATE_ALLOW);
    assert(WatchdogCompletionGate_Evaluate(200U, &motor, &safety, &config, &state) == WATCHDOG_GATE_ALLOW);
    assert(state.consumed_motor_generation == 0U);
    assert(state.consumed_safety_generation == 0U);
}

static void test_age_boundaries_and_time_wrap(void)
{
    task_completion_t     motor  = {1U, UINT32_MAX - 9U};
    task_completion_t     safety = {1U, UINT32_MAX - 29U};
    watchdog_gate_state_t state  = {0};

    assert(WatchdogCompletionGate_Evaluate(10U, &motor, &safety, &config, &state) == WATCHDOG_GATE_ALLOW);
    motor.completed_at_ms--;
    assert(WatchdogCompletionGate_Evaluate(10U, &motor, &safety, &config, &state) == WATCHDOG_GATE_DENY);
}

static void test_generation_wrap_and_stalls(void)
{
    task_completion_t     motor  = {UINT32_MAX, 300U};
    task_completion_t     safety = {UINT32_MAX, 300U};
    watchdog_gate_state_t state  = {UINT32_MAX, UINT32_MAX};

    motor.generation  = 1U;
    safety.generation = 1U;
    assert(WatchdogCompletionGate_Evaluate(300U, &motor, &safety, &config, &state) == WATCHDOG_GATE_ALLOW);
    motor.completed_at_ms  = 279U;
    safety.completed_at_ms = 259U;
    assert(WatchdogCompletionGate_Evaluate(300U, &motor, &safety, &config, &state) == WATCHDOG_GATE_DENY);
}

static void test_config_invariant(void)
{
    task_completion_t      completion = {1U, 0U};
    watchdog_gate_state_t  state      = {0};
    watchdog_gate_config_t invalid    = config;

    invalid.motor_timeout_ms = invalid.safety_timeout_ms;
    assert(WatchdogCompletionGate_Evaluate(0U, &completion, &completion, &invalid, &state)
           == WATCHDOG_GATE_INVALID_CONFIG);
    invalid                   = config;
    invalid.safety_timeout_ms = invalid.hardware_timeout_ms;
    assert(WatchdogCompletionGate_Evaluate(0U, &completion, &completion, &invalid, &state)
           == WATCHDOG_GATE_INVALID_CONFIG);
}

int main(void)
{
    test_both_tasks_must_advance();
    test_evaluate_does_not_consume();
    test_age_boundaries_and_time_wrap();
    test_generation_wrap_and_stalls();
    test_config_invariant();
    return 0;
}
