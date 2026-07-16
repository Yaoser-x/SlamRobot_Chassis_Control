#ifndef MOTOR_REVERSAL_STATE_H
#define MOTOR_REVERSAL_STATE_H

#include <stdint.h>

#include "motor_state_types.h"

typedef struct
{
    uint16_t rise_step_per_cycle;
    uint16_t fall_step_per_cycle;
    uint8_t  fixed_brake_cycles;
    uint8_t  feedback_brake_cycles;
    float    reverse_speed_threshold_mps;
} motor_reversal_config_t;

typedef struct
{
    int16_t              requested_pwm;
    int16_t              applied_pwm;
    int8_t               current_ph_dir;
    int8_t               pending_dir;
    motor_driver_phase_t phase;
    uint8_t              wait_cycles;
    uint8_t              phase_initialized;
} motor_reversal_state_machine_t;

typedef struct
{
    int16_t requested_pwm;
    uint8_t speed_feedback_available;
    float   speed_mps;
} motor_reversal_input_t;

typedef struct
{
    int16_t              applied_pwm;
    int8_t               current_ph_dir;
    int8_t               pending_dir;
    motor_driver_phase_t phase;
    uint8_t              phase_changed;
} motor_reversal_output_t;

/** Initialize one motor reversal state in low-side-brake idle. */
void MotorReversalStateMachine_Init(motor_reversal_state_machine_t *state);

/** Clear requested and applied PWM while preserving the current PH direction. */
void MotorReversalStateMachine_ClearOutput(motor_reversal_state_machine_t *state);

/** Reset one disabled motor to the hardware-safe default direction. */
void MotorReversalStateMachine_Disable(motor_reversal_state_machine_t *state);

/** Advance one pure motor PWM ramp and reversal-protection step. */
motor_reversal_output_t MotorReversalStateMachine_Step(motor_reversal_state_machine_t *state,
                                                       const motor_reversal_config_t  *config,
                                                       const motor_reversal_input_t   *input);

#endif
