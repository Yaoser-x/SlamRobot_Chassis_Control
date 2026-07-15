#ifndef DEBUG_TELEMETRY_H
#define DEBUG_TELEMETRY_H

#include <stdint.h>

/** Initialize debug telemetry streaming state. */
void DebugTelemetry_Init(void);
/** Stop telemetry streaming and reset its sampling baseline. */
void DebugTelemetry_Stop(void);
/** Handle telemetry header, format, rate, and stream commands. */
uint8_t DebugTelemetry_TryHandle(char *line);
/** Emit a telemetry frame when the configured period has elapsed. */
void DebugTelemetry_Step(uint32_t now_ms);
/** Print the stable full CSV telemetry header. */
void DebugTelemetry_PrintHeader(void);

#endif
