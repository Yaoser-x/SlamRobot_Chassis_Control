#ifndef CHASSIS_SPEED_LOOP_H
#define CHASSIS_SPEED_LOOP_H

#include <stdint.h>

#include "motor_driver.h"
#include "param_service.h"
#include "pid_controller.h"

typedef struct
{
    pid_state_t pid_motor[MOTOR_ID_COUNT];
    float       last_target_mps[MOTOR_ID_COUNT];
} chassis_speed_loop_t;

typedef struct
{
    int16_t permille;
    float   error_mps;
    uint8_t pid_active;
} chassis_speed_loop_result_t;

/** Initialize all chassis speed-loop controllers. */
void ChassisSpeedLoop_Init(chassis_speed_loop_t *loop);

/** Apply one runtime parameter model to all speed-loop controllers. */
void ChassisSpeedLoop_SetParams(chassis_speed_loop_t *loop, const param_model_t *params);

/** Reset all speed-loop integrators and target history. */
void ChassisSpeedLoop_Reset(chassis_speed_loop_t *loop);

/** Reset one motor speed-loop integrator and target history. */
void ChassisSpeedLoop_ResetMotor(chassis_speed_loop_t *loop, motor_id_t motor);

/** Reset target history without changing PID integrators. */
void ChassisSpeedLoop_ResetTargets(chassis_speed_loop_t *loop);

/** Step one motor speed loop with actuator-limit feedback. */
chassis_speed_loop_result_t ChassisSpeedLoop_StepMotor(chassis_speed_loop_t *loop,
                                                       motor_id_t            motor,
                                                       float                 target_mps,
                                                       float                 actual_mps,
                                                       uint8_t               speed_valid,
                                                       float                 dt_s,
                                                       int8_t                actuator_limit_direction,
                                                       int16_t               base_permille,
                                                       motor_driver_phase_t  phase);

#endif
