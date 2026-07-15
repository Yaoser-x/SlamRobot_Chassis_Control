#ifndef OLED_PAGE_SELFCHECK_H
#define OLED_PAGE_SELFCHECK_H

#include <stdint.h>

#define OLED_PAGE_SELFCHECK_ITEM_COUNT 8U

/** Draw one self-check item and its current result. */
void OLED_PageSelfcheck_Draw(uint8_t current, const uint8_t results[OLED_PAGE_SELFCHECK_ITEM_COUNT]);

#endif
