#ifndef OLED_PAGE_FAULT_H
#define OLED_PAGE_FAULT_H

#include <stdint.h>

#include "oled_ui_model.h"

/** Draw the runtime fault summary and active control source row. */
void OLED_PageFault_Draw(const oled_ui_model_t *model, uint32_t selfcheck_errors, uint8_t blink_visible);

#endif
