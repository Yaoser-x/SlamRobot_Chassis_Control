#ifndef OLED_TEXT_RENDERER_H
#define OLED_TEXT_RENDERER_H

#include <stdint.h>

/** Draw a UTF-8 string from a three-byte Chinese glyph map. */
void OLED_TextRenderer_DrawChinese(uint8_t        x,
                                   uint8_t        page,
                                   const char    *text,
                                   const uint8_t *font_data,
                                   const char    *chars_map,
                                   uint8_t        font_width,
                                   uint8_t        font_height);

#endif
