#ifndef LINE_FOLLOWING_CALIBRATION_TYPES_H
#define LINE_FOLLOWING_CALIBRATION_TYPES_H

#include <stdint.h>

#define LINE_CALIBRATION_CHANNELS 8U

typedef enum
{
    LINE_CALIBRATION_SURFACE_FLOOR = 0,
    LINE_CALIBRATION_SURFACE_LINE  = 1
} line_sensor_calibration_surface_t;

typedef struct
{
    uint32_t sum[2][LINE_CALIBRATION_CHANNELS];
    uint16_t min[2][LINE_CALIBRATION_CHANNELS];
    uint16_t max[2][LINE_CALIBRATION_CHANNELS];
    uint16_t count[2];
    uint16_t target_samples;
    uint8_t  collecting;
    uint8_t  surface;
    uint8_t  ready_mask;
    uint8_t  fail_mask;
} line_sensor_calibration_t;

typedef enum
{
    LINE_CALIBRATION_APPLY_OK = 0,
    LINE_CALIBRATION_APPLY_INCOMPLETE,
    LINE_CALIBRATION_APPLY_LOW_SEPARATION,
    LINE_CALIBRATION_APPLY_PARAMETER_REJECTED
} line_calibration_apply_result_t;

#endif /* LINE_FOLLOWING_CALIBRATION_TYPES_H */
