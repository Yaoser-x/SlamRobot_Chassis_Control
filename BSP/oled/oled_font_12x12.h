/*
 * Copyright 2026 Your Name. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Chinese 12x12 column-major font library
 * PCtoLCD params: column-major, bit0=top, left-to-right top-to-bottom
 * 24 bytes per glyph (12 cols x 2 pages)
 *
 * Character encoding mapping: oled_ui.c uses lookup table
 * chinese12_index[] to map Unicode to array index
 */

#ifndef OLED_FONT_12X12_H
#define OLED_FONT_12X12_H

#include <stdint.h>

#define OLED_FONT_12X12_WIDTH   12
#define OLED_FONT_12X12_HEIGHT  12
#define OLED_FONT_12X12_BYTES_PER_CHAR 24

/*
 * OLED_FONT_12X12_CHARS: null-terminated character list, one-to-one with DATA array
 * OLED_FONT_12X12_DATA: font bitmap data, 24 bytes per glyph
 */
extern const char    OLED_FONT_12X12_CHARS[];
extern const uint8_t OLED_FONT_12X12_COUNT;
extern const uint8_t OLED_FONT_12X12_DATA[];

#endif /* OLED_FONT_12X12_H */
