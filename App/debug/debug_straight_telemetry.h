#ifndef DEBUG_STRAIGHT_TELEMETRY_H
#define DEBUG_STRAIGHT_TELEMETRY_H

#include <stddef.h>

#include "power_adc_driver.h"
#include "motion_control_service.h"

/** Format stable JSON fields shared by straight-line HIL logging. */
size_t DebugStraightTelemetry_FormatJson(char                           *buffer,
                                         size_t                          buffer_size,
                                         const motion_control_status_t  *chassis,
                                         const power_adc_driver_state_t *adc);

/** Format the CSV header suffix in the same field order as the data suffix. */
size_t DebugStraightTelemetry_FormatCsvHeader(char *buffer, size_t buffer_size);

/** Format stable CSV values from the same snapshots used by JSON logging. */
size_t DebugStraightTelemetry_FormatCsv(char                           *buffer,
                                        size_t                          buffer_size,
                                        const motion_control_status_t  *chassis,
                                        const power_adc_driver_state_t *adc);

#endif
