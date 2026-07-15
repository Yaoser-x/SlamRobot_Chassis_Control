#ifndef DEBUG_TELEMETRY_CSV_H
#define DEBUG_TELEMETRY_CSV_H

#include <stdint.h>

#include "debug_telemetry_model.h"

/** Encode and write one full CSV telemetry frame. */
void DebugTelemetryCsv_Print(uint32_t now_ms, const debug_full_log_snapshot_t *snapshot);

#endif
