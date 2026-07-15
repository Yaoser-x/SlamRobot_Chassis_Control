#ifndef OLED_PAGE_CALIBRATION_H
#define OLED_PAGE_CALIBRATION_H

#include <stdint.h>

#include "oled_ui_model.h"

/** Draw calibration content and return nonzero when it replaces the runtime page. */
uint8_t OLED_PageCalibration_Draw(const oled_ui_model_t *model);

#endif
