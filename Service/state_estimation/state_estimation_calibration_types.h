#ifndef STATE_ESTIMATION_CALIBRATION_TYPES_H
#define STATE_ESTIMATION_CALIBRATION_TYPES_H

#include <stdint.h>

typedef struct
{
    int16_t output_permille[4];
    uint8_t enabled_mask;
} state_estimation_calibration_motion_facts_t;

#endif /* STATE_ESTIMATION_CALIBRATION_TYPES_H */
