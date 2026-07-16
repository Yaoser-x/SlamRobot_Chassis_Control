#ifndef CHASSIS_SPEED_LOOP_H
#define CHASSIS_SPEED_LOOP_H

#include <stdint.h>

#include "motor_driver.h"
#include "motion_control_config.h"
#include "parameter_management_status.h"
#include "wheel_speed_pid_controller.h"

typedef struct
{
    pid_state_t pid_motor[MOTOR_ID_COUNT];
    float       last_target_mps[MOTOR_ID_COUNT];
    float       correction_limit;
    float       stop_epsilon_mps;
    float       direction_epsilon_mps;
} wheel_speed_control_loop_t;

typedef struct
{
    int16_t permille;
    float   error_mps;
    uint8_t pid_active;
} wheel_speed_control_loop_result_t;

/** Initialize all chassis speed-loop controllers. */
void WheelSpeedControlLoop_Init(wheel_speed_control_loop_t *loop, const motion_control_config_t *config);

/** Apply one runtime parameter model to all speed-loop controllers. */
void WheelSpeedControlLoop_SetParams(wheel_speed_control_loop_t *loop, const param_model_t *params);

/** Reset all speed-loop integrators and target history. */
void WheelSpeedControlLoop_Reset(wheel_speed_control_loop_t *loop);

/** Reset one motor speed-loop integrator and target history. */
void WheelSpeedControlLoop_ResetMotor(wheel_speed_control_loop_t *loop, motor_id_t motor);

/** Reset target history without changing PID integrators. */
void WheelSpeedControlLoop_ResetTargets(wheel_speed_control_loop_t *loop);

/** Step one motor speed loop with actuator-limit feedback. */
wheel_speed_control_loop_result_t WheelSpeedControlLoop_StepMotor(wheel_speed_control_loop_t *loop,
                                                                  motor_id_t                  motor,
                                                                  float                       target_mps,
                                                                  float                       actual_mps,
                                                                  uint8_t                     speed_valid,
                                                                  float                       dt_s,
                                                                  int8_t                      actuator_limit_direction,
                                                                  int16_t                     base_permille,
                                                                  motor_driver_phase_t        phase);

#endif
