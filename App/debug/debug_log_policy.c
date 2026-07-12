#include "debug_log_policy.h"

void DebugLogPolicy_Init(debug_log_policy_t *policy)
{
  if (policy != 0)
  {
    policy->period_ms = 500U;
    policy->format = DEBUG_LOG_FORMAT_CSV;
  }
}

uint8_t DebugLogPolicy_SetPeriod(debug_log_policy_t *policy, uint32_t period_ms)
{
  if (policy == 0 || period_ms < 50U || period_ms > 5000U)
  {
    return 0U;
  }
  policy->period_ms = period_ms;
  return 1U;
}

void DebugLogPolicy_SetFormat(debug_log_policy_t *policy, debug_log_format_t format)
{
  if (policy != 0 && (format == DEBUG_LOG_FORMAT_CSV || format == DEBUG_LOG_FORMAT_JSON))
  {
    policy->format = format;
  }
}
