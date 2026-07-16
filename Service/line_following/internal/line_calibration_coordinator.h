#ifndef LINE_CALIBRATION_COORDINATOR_H
#define LINE_CALIBRATION_COORDINATOR_H

#include <stdint.h>

#include "line_following_calibration_types.h"

/** Start line calibration after acquiring the stationary maintenance gate. */
void LineCalibrationCoordinator_SetPorts(const line_following_calibration_ports_t *ports);
uint8_t LineCalibrationCoordinator_Begin(line_sensor_calibration_surface_t surface, uint16_t samples);
/** Process at most one calibration request submitted by another Service. */
void LineCalibrationCoordinator_ProcessRequest(void);
/** Explicitly persist the current Parameter-owned RAM calibration. */
uint8_t LineCalibrationCoordinator_CommitToFlash(void);

#endif /* APP_LINE_CALIBRATION_H */
