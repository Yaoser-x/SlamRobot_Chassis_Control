#ifndef ESP12F_BOOT_CONTROL_H
#define ESP12F_BOOT_CONTROL_H

#include <stdint.h>

/** Begin ROM download boot; call Update until it returns nonzero. Never waits. */
void Esp12fBootControl_EnterDownload(void);

/** Begin normal boot; call Update until it returns nonzero. Never waits. */
void Esp12fBootControl_EnterNormal(void);

/** Advance at most one pin transition using the HAL millisecond clock. */
uint8_t Esp12fBootControl_Update(uint32_t now_ms);

#endif
