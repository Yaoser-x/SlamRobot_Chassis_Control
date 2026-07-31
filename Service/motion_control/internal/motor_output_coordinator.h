#ifndef CHASSIS_OUTPUT_SERVICE_H
#define CHASSIS_OUTPUT_SERVICE_H

#include <stdint.h>

#include "motion_control_config.h"
#include "motion_control_status.h"
#include "motor_types.h"
#include "parameter_management_types.h"
#include "power_management_status.h"

void MotorOutputCoordinator_Init(const motion_control_config_t *config);

/** Clamp a signed motor command to the chassis permille limit. */
int16_t MotorOutputCoordinator_Clamp(int32_t permille);

/** Convert one wheel-speed target to open-loop permille. */
int16_t MotorOutputCoordinator_MpsToPermille(float target_mps);

/** Stop one motor without consulting mutable Service state. */
void MotorOutputCoordinator_StopMotor(motion_control_status_t *snapshot, motor_id_t motor);

/** Apply current limiting from one shared ADC snapshot and send one final motor output. */
void MotorOutputCoordinator_SetMotorWithPower(motion_control_status_t         *snapshot,
                                              motor_id_t                       motor,
                                              int16_t                          permille,
                                              const power_management_status_t *power_status,
                                              const param_model_t             *params);

/** Apply current limiting without writing hardware. */
int16_t MotorOutputCoordinator_ApplyCurrentLimit(motor_id_t                       motor,
                                                 int16_t                          permille,
                                                 const power_management_status_t *power_status,
                                                 const param_model_t             *params,
                                                 uint8_t                         *limited);

/** Return non-zero when any enabled motor snapshot output is active. */
uint8_t MotorOutputCoordinator_AnyActive(const motion_control_status_t *snapshot);

#endif
