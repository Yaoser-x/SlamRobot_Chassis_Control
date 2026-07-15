#include "motor_reversal_state.h"

static int8_t MotorReversalState_Sign(int16_t value)
{
    if (value > 0)
    {
        return 1;
    }
    if (value < 0)
    {
        return -1;
    }
    return 0;
}

static uint16_t MotorReversalState_Abs(int16_t value)
{
    return (value < 0) ? (uint16_t)(-value) : (uint16_t)value;
}

static uint16_t MotorReversalState_Ramp(uint16_t current, uint16_t target, uint16_t rise_step, uint16_t fall_step)
{
    if (current < target)
    {
        uint16_t next = (uint16_t)(current + rise_step);
        return (next > target) ? target : next;
    }
    if (current > target)
    {
        return ((uint16_t)(current - target) <= fall_step) ? target : (uint16_t)(current - fall_step);
    }
    return current;
}

static motor_reversal_output_t MotorReversalState_Output(const motor_reversal_state_t *state, uint8_t phase_changed)
{
    motor_reversal_output_t output = {0};

    if (state != 0)
    {
        output.applied_pwm    = state->applied_pwm;
        output.current_ph_dir = state->current_ph_dir;
        output.pending_dir    = state->pending_dir;
        output.phase          = state->phase;
        output.phase_changed  = phase_changed;
    }
    return output;
}

void MotorReversalState_Init(motor_reversal_state_t *state)
{
    MotorReversalState_Disable(state);
}

void MotorReversalState_ClearOutput(motor_reversal_state_t *state)
{
    if (state == 0)
    {
        return;
    }
    state->requested_pwm = 0;
    state->applied_pwm   = 0;
    state->pending_dir   = 0;
    state->phase         = MOTOR_DRIVER_PHASE_IDLE_BRAKE;
    state->wait_cycles   = 0U;
}

void MotorReversalState_Disable(motor_reversal_state_t *state)
{
    if (state != 0)
    {
        *state = (motor_reversal_state_t){
            .current_ph_dir = -1,
            .phase          = MOTOR_DRIVER_PHASE_IDLE_BRAKE,
        };
    }
}

motor_reversal_output_t MotorReversalState_Step(motor_reversal_state_t        *state,
                                                const motor_reversal_config_t *config,
                                                const motor_reversal_input_t  *input)
{
    int8_t   target_dir;
    int8_t   applied_dir;
    uint16_t target_mag;
    uint16_t applied_mag;
    uint16_t next_mag;
    uint8_t  phase_changed = 0U;

    if (state == 0 || config == 0 || input == 0)
    {
        return (motor_reversal_output_t){0};
    }
    state->requested_pwm = input->requested_pwm;
    target_dir           = MotorReversalState_Sign(input->requested_pwm);
    target_mag           = MotorReversalState_Abs(input->requested_pwm);
    applied_dir          = MotorReversalState_Sign(state->applied_pwm);
    applied_mag          = MotorReversalState_Abs(state->applied_pwm);

    if (target_dir == 0)
    {
        next_mag = MotorReversalState_Ramp(applied_mag, 0U, config->rise_step_per_cycle, config->fall_step_per_cycle);
        state->applied_pwm = (applied_dir != 0) ? (int16_t)((int16_t)next_mag * applied_dir) : 0;
        state->pending_dir = 0;
        state->wait_cycles = 0U;
        state->phase       = (state->applied_pwm == 0) ? MOTOR_DRIVER_PHASE_IDLE_BRAKE : MOTOR_DRIVER_PHASE_RAMP_DOWN;
        return MotorReversalState_Output(state, 0U);
    }

    if (state->applied_pwm != 0 && applied_dir != target_dir)
    {
        next_mag = MotorReversalState_Ramp(applied_mag, 0U, config->rise_step_per_cycle, config->fall_step_per_cycle);
        state->applied_pwm = (int16_t)((int16_t)next_mag * applied_dir);
        state->pending_dir = target_dir;
        if (state->applied_pwm == 0)
        {
            state->phase = MOTOR_DRIVER_PHASE_REVERSE_BRAKE;
            state->wait_cycles =
                (input->speed_feedback_available != 0U) ? config->feedback_brake_cycles : config->fixed_brake_cycles;
        }
        else
        {
            state->phase = MOTOR_DRIVER_PHASE_RAMP_DOWN;
        }
        return MotorReversalState_Output(state, 0U);
    }

    if (state->applied_pwm == 0 && state->current_ph_dir != target_dir)
    {
        state->pending_dir = target_dir;
        if (state->phase == MOTOR_DRIVER_PHASE_REVERSE_BRAKE)
        {
            float   speed_abs = (input->speed_mps < 0.0f) ? -input->speed_mps : input->speed_mps;
            uint8_t ready =
                (input->speed_feedback_available != 0U && speed_abs < config->reverse_speed_threshold_mps) ? 1U : 0U;
            if (ready == 0U && state->wait_cycles > 1U)
            {
                state->wait_cycles--;
            }
            else
            {
                state->wait_cycles = 0U;
                state->phase       = MOTOR_DRIVER_PHASE_PH_SETTLE;
            }
            return MotorReversalState_Output(state, 0U);
        }

        state->current_ph_dir = target_dir;
        state->pending_dir    = 0;
        state->phase          = MOTOR_DRIVER_PHASE_RAMP_UP;
        if (state->phase_initialized != 0U)
        {
            return MotorReversalState_Output(state, 1U);
        }
        state->phase_initialized = 1U;
        phase_changed            = 1U;
    }

    if (state->phase == MOTOR_DRIVER_PHASE_PH_SETTLE)
    {
        state->current_ph_dir    = target_dir;
        state->pending_dir       = 0;
        state->phase             = MOTOR_DRIVER_PHASE_RAMP_UP;
        state->phase_initialized = 1U;
        return MotorReversalState_Output(state, 1U);
    }

    state->phase_initialized = 1U;
    next_mag                 = MotorReversalState_Ramp(MotorReversalState_Abs(state->applied_pwm),
                                       target_mag,
                                       config->rise_step_per_cycle,
                                       config->fall_step_per_cycle);
    state->applied_pwm       = (int16_t)((int16_t)next_mag * target_dir);
    state->pending_dir       = 0;
    state->phase             = (state->applied_pwm == 0) ? MOTOR_DRIVER_PHASE_IDLE_BRAKE : MOTOR_DRIVER_PHASE_RUN;
    return MotorReversalState_Output(state, phase_changed);
}
