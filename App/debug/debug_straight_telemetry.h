#ifndef DEBUG_STRAIGHT_TELEMETRY_H
#define DEBUG_STRAIGHT_TELEMETRY_H

#include <stddef.h>

#include "adc_monitor.h"
#include "chassis_service.h"

/** Format stable JSON fields shared by straight-line HIL logging. */
size_t DebugStraightTelemetry_FormatJson(char                             *buffer,
                                         size_t                            buffer_size,
                                         const chassis_service_snapshot_t *chassis,
                                         const adc_monitor_state_t        *adc);

/** Format the CSV header suffix in the same field order as the data suffix. */
size_t DebugStraightTelemetry_FormatCsvHeader(char *buffer, size_t buffer_size);

/** Format stable CSV values from the same snapshots used by JSON logging. */
size_t DebugStraightTelemetry_FormatCsv(char                             *buffer,
                                        size_t                            buffer_size,
                                        const chassis_service_snapshot_t *chassis,
                                        const adc_monitor_state_t        *adc);

#endif
