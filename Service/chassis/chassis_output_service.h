#ifndef CHASSIS_OUTPUT_SERVICE_H
#define CHASSIS_OUTPUT_SERVICE_H

#include <stdint.h>

#include "adc_monitor.h"
#include "chassis_service.h"

/** Clamp a signed motor command to the chassis permille limit. */
int16_t ChassisOutputService_Clamp(int32_t permille);

/** Convert one wheel-speed target to open-loop permille. */
int16_t ChassisOutputService_MpsToPermille(float target_mps);

/** Apply current limiting and send one final motor output. */
void ChassisOutputService_SetMotor(chassis_service_snapshot_t *snapshot, motor_id_t motor, int16_t permille);

/** Apply current limiting from one shared ADC snapshot and send one final motor output. */
void ChassisOutputService_SetMotorWithAdc(chassis_service_snapshot_t *snapshot,
                                          motor_id_t                  motor,
                                          int16_t                     permille,
                                          const adc_monitor_state_t  *adc);

/** Apply current limiting without writing hardware. */
int16_t ChassisOutputService_ApplyCurrentLimit(motor_id_t                 motor,
                                               int16_t                    permille,
                                               const adc_monitor_state_t *adc,
                                               uint8_t                   *limited);

/** Return non-zero when any enabled motor snapshot output is active. */
uint8_t ChassisOutputService_AnyActive(const chassis_service_snapshot_t *snapshot);

#endif
