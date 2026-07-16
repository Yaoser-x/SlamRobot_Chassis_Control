#ifndef STATE_ESTIMATION_CALIBRATION_TYPES_H
#define STATE_ESTIMATION_CALIBRATION_TYPES_H

#include "parameter_imu_calibration_types.h"

typedef struct
{
    uint8_t (*get_motion_facts)(int16_t output_permille[4], uint8_t *enabled_mask);
    uint8_t (*begin_maintenance)(void);
    void (*end_maintenance)(void);
    uint8_t (*persist)(const imu_calibration_t *calibration, uint8_t persist_current_zero);
    void (*set_current_zero_persistence)(uint8_t enabled);
} state_estimation_calibration_ports_t;

#endif /* STATE_ESTIMATION_CALIBRATION_TYPES_H */
