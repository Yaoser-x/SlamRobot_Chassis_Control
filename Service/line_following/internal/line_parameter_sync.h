#ifndef LINE_PARAMETER_SYNC_H
#define LINE_PARAMETER_SYNC_H

#include <stdint.h>

typedef struct
{
    uint32_t applied_generation;
} line_parameter_sync_t;

/** Reset the applied parameter generation. */
void LineParameterSync_Init(line_parameter_sync_t *sync);
/** Apply line thresholds and polarity when the Parameter generation changes. */
uint8_t LineParameterSync_Update(line_parameter_sync_t *sync);

#endif /* LINE_PARAMETER_SYNC_H */
