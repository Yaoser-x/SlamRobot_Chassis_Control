#ifndef APP_LINE_CALIBRATION_H
#define APP_LINE_CALIBRATION_H

#include <stdint.h>

#include "line_sensor_calibration.h"

/** Start line calibration after acquiring the stationary maintenance gate. */
uint8_t AppLineSensorCalibration_Begin(line_sensor_calibration_surface_t surface, uint16_t samples);
/** Process at most one calibration request submitted by another Service. */
void AppLineSensorCalibration_ProcessRequest(void);
/** Explicitly persist the current Parameter-owned RAM calibration. */
uint8_t AppLineSensorCalibration_CommitToFlash(void);

#endif /* APP_LINE_CALIBRATION_H */
