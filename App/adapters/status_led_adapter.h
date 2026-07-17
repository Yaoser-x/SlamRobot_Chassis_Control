#ifndef STATUS_LED_ADAPTER_H
#define STATUS_LED_ADAPTER_H

#include <stdint.h>

/** @brief Advance the BSP-owned status LED pattern by one task period. */
void StatusLedAdapter_TaskStep(uint32_t elapsed_ms);

#endif /* STATUS_LED_ADAPTER_H */
