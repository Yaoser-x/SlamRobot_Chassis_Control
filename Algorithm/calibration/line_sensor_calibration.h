#ifndef LINE_CALIBRATION_H
#define LINE_CALIBRATION_H

#include <stdint.h>

#define LINE_CALIBRATION_CHANNELS           8U
#define LINE_CALIBRATION_MIN_SEPARATION_RAW 50U

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

/** Reset all collected line calibration samples. */
void LineSensorCalibration_Init(line_sensor_calibration_t *calibration);
/** Start collecting one surface with a bounded sample count. */
uint8_t LineSensorCalibration_Begin(line_sensor_calibration_t        *calibration,
                                    line_sensor_calibration_surface_t surface,
                                    uint16_t                          samples);
/** Feed one eight-channel raw sample into the active collection. */
void LineSensorCalibration_Feed(line_sensor_calibration_t *calibration, const uint16_t raw[LINE_CALIBRATION_CHANNELS]);
/** Build per-channel thresholds and reject insufficient separation. */
uint8_t LineSensorCalibration_Apply(line_sensor_calibration_t *calibration,
                                    uint16_t                   thresholds[LINE_CALIBRATION_CHANNELS],
                                    uint8_t                   *active_low);

#endif /* LINE_CALIBRATION_H */
