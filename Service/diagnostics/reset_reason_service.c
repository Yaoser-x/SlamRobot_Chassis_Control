#include "reset_reason_service.h"

static uint32_t reset_reason_flags;

void ResetReasonService_Capture(uint32_t flags)
{
    reset_reason_flags = flags;
}

uint32_t ResetReasonService_GetFlags(void)
{
    return reset_reason_flags;
}
