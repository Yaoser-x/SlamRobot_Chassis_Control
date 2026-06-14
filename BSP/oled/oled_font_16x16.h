/*
 * Copyright 2026 Your Name. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Chinese 16x16 column-major font library
 * PCtoLCD params: column-major, bit0=top, left-to-right top-to-bottom
 * 32 bytes per glyph (16 cols x 2 pages)
 */

#ifndef OLED_FONT_16X16_H
#define OLED_FONT_16X16_H

#include <stdint.h>

#define OLED_FONT_16X16_WIDTH   16
#define OLED_FONT_16X16_HEIGHT  16
#define OLED_FONT_16X16_BYTES_PER_CHAR 32

extern const char    OLED_FONT_16X16_CHARS[];
extern const uint8_t OLED_FONT_16X16_COUNT;
extern const uint8_t OLED_FONT_16X16_DATA[];

#endif /* OLED_FONT_16X16_H */
