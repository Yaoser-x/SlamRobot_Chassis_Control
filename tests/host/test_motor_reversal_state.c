#include <assert.h>
#include <stdio.h>

#include "motor_reversal_state_machine.h"

static const motor_reversal_config_t config = {
    .rise_step_per_cycle         = 15U,
    .fall_step_per_cycle         = 25U,
    .fixed_brake_cycles          = 2U,
    .feedback_brake_cycles       = 20U,
    .reverse_speed_threshold_mps = 0.02f,
};

static motor_reversal_output_t Step(motor_reversal_state_machine_t *state, int16_t requested)
{
    motor_reversal_input_t input = {.requested_pwm = requested};
    return MotorReversalStateMachine_Step(state, &config, &input);
}

static void TestStartRampAndStop(void)
{
    motor_reversal_state_machine_t state;
    motor_reversal_output_t        output;

    MotorReversalStateMachine_Init(&state);
    output = Step(&state, 300);
    assert(output.phase_changed != 0U);
    assert(output.current_ph_dir == 1);
    assert(output.applied_pwm == 15);
    assert(output.phase == MOTOR_DRIVER_PHASE_RUN);
    for (uint8_t count = 1U; count < 20U; ++count)
    {
        output = Step(&state, 300);
    }
    assert(output.applied_pwm == 300);
    output = Step(&state, 0);
    assert(output.applied_pwm == 275);
    assert(output.current_ph_dir == 1);
    assert(output.phase == MOTOR_DRIVER_PHASE_RAMP_DOWN);
}

static void TestReverseWaitsForZeroAndPhaseSettle(void)
{
    motor_reversal_state_machine_t state;
    motor_reversal_output_t        output;

    MotorReversalStateMachine_Init(&state);
    for (uint8_t count = 0U; count < 20U; ++count)
    {
        (void)Step(&state, 300);
    }
    for (uint8_t count = 0U; count < 12U; ++count)
    {
        output = Step(&state, -300);
    }
    assert(output.applied_pwm == 0);
    assert(output.current_ph_dir == 1);
    assert(output.phase == MOTOR_DRIVER_PHASE_REVERSE_BRAKE);
    output = Step(&state, -300);
    assert(output.phase == MOTOR_DRIVER_PHASE_REVERSE_BRAKE);
    output = Step(&state, -300);
    assert(output.phase == MOTOR_DRIVER_PHASE_PH_SETTLE);
    assert(output.phase_changed == 0U);
    output = Step(&state, -300);
    assert(output.phase_changed != 0U);
    assert(output.current_ph_dir == -1);
    assert(output.applied_pwm == 0);
    assert(output.phase == MOTOR_DRIVER_PHASE_RAMP_UP);
    output = Step(&state, -300);
    assert(output.applied_pwm == -15);
    assert(output.phase == MOTOR_DRIVER_PHASE_RUN);
}

static void TestSpeedFeedbackCanEndBrakeEarly(void)
{
    motor_reversal_state_machine_t state;
    motor_reversal_input_t         input = {
                .requested_pwm            = -300,
                .speed_feedback_available = 1U,
                .speed_mps                = 0.0f,
    };

    MotorReversalStateMachine_Init(&state);
    state.applied_pwm       = 0;
    state.current_ph_dir    = 1;
    state.pending_dir       = -1;
    state.phase             = MOTOR_DRIVER_PHASE_REVERSE_BRAKE;
    state.wait_cycles       = config.feedback_brake_cycles;
    state.phase_initialized = 1U;
    assert(MotorReversalStateMachine_Step(&state, &config, &input).phase == MOTOR_DRIVER_PHASE_PH_SETTLE);
}

int main(void)
{
    TestStartRampAndStop();
    TestReverseWaitsForZeroAndPhaseSettle();
    TestSpeedFeedbackCanEndBrakeEarly();
    puts("motor reversal state tests passed");
    return 0;
}
