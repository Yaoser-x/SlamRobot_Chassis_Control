/*
 * Copyright 2026 Your Name. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 *
 * ASCII 8x16 column-major font library
 * PCtoLCD params: column-major, bit0=top, left-to-right top-to-bottom
 * Covers: space (0x20) through '~' (0x7E), 95 glyphs
 */

#ifndef OLED_FONT_8X16_H
#define OLED_FONT_8X16_H

#include <stdint.h>

#define OLED_FONT_8X16_WIDTH          8
#define OLED_FONT_8X16_HEIGHT         16
#define OLED_FONT_8X16_START          0x20
#define OLED_FONT_8X16_COUNT          95
#define OLED_FONT_8X16_BYTES_PER_CHAR 16

extern const uint8_t OLED_FONT_8X16_DATA[OLED_FONT_8X16_COUNT * OLED_FONT_8X16_BYTES_PER_CHAR];

#endif /* OLED_FONT_8X16_H */
