#include "reset_reason.h"

static uint32_t reset_reason_flags;

void ResetReason_Capture(uint32_t flags)
{
  reset_reason_flags = flags;
}

uint32_t ResetReason_GetFlags(void)
{
  return reset_reason_flags;
}
