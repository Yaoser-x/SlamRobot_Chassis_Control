#ifndef OLED_PAGE_RUNTIME_H
#define OLED_PAGE_RUNTIME_H

#include <stdint.h>

#include "oled_ui_model.h"

/** Draw the runtime or IMU-calibration page from an offline model. */
void OLED_PageRuntime_Draw(const oled_ui_model_t *model, uint32_t selfcheck_errors, uint8_t blink_visible);

#endif
