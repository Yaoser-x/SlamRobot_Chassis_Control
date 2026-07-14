#ifndef RESET_REASON_H
#define RESET_REASON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** Capture the hardware reset flags before application initialization changes them. */
    void ResetReasonService_Capture(uint32_t flags);
    /** Return the reset flags captured during startup. */
    uint32_t ResetReasonService_GetFlags(void);

#ifdef __cplusplus
}
#endif

#endif /* RESET_REASON_H */
