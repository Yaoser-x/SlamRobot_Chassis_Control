#ifndef MOTOR_STATE_TYPES_H
#define MOTOR_STATE_TYPES_H

/** @brief Hardware driver phase reported for one motor channel. */
typedef enum
{
    MOTOR_DRIVER_PHASE_IDLE_BRAKE = 0,
    MOTOR_DRIVER_PHASE_RUN,
    MOTOR_DRIVER_PHASE_RAMP_DOWN,
    MOTOR_DRIVER_PHASE_REVERSE_BRAKE,
    MOTOR_DRIVER_PHASE_PH_SETTLE,
    MOTOR_DRIVER_PHASE_RAMP_UP
} motor_driver_phase_t;

#endif
