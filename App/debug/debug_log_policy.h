#ifndef DEBUG_LOG_POLICY_H
#define DEBUG_LOG_POLICY_H

#include <stdint.h>

typedef enum
{
  DEBUG_LOG_FORMAT_CSV = 0,
  DEBUG_LOG_FORMAT_JSON = 1
} debug_log_format_t;

typedef struct
{
  uint32_t period_ms;
  debug_log_format_t format;
} debug_log_policy_t;

/** Initialize deterministic console logging defaults. */
void DebugLogPolicy_Init(debug_log_policy_t *policy);
/** Set a logging period in the supported 50-5000 ms range. */
uint8_t DebugLogPolicy_SetPeriod(debug_log_policy_t *policy, uint32_t period_ms);
/** Select CSV or newline-delimited JSON output. */
void DebugLogPolicy_SetFormat(debug_log_policy_t *policy, debug_log_format_t format);

#endif
