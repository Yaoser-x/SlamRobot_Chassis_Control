#ifndef APP_HARDWARE_INIT_H
#define APP_HARDWARE_INIT_H

#include <stdint.h>

/** @brief Initialize status LED hardware. */
void AppHardware_InitStatusLed(void);
/** @brief Initialize the line sensor hardware with persisted thresholds. */
void AppHardware_InitLineSensor(const uint16_t threshold_raw[8], uint8_t active_low);
/** @brief Initialize the OLED display hardware. */
void AppHardware_InitDisplay(void);

#endif /* APP_HARDWARE_INIT_H */
