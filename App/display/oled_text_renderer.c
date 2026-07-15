#include "oled_text_renderer.h"

#include "ssd1306.h"

void OLED_TextRenderer_DrawChinese(uint8_t        x,
                                   uint8_t        page,
                                   const char    *str,
                                   const uint8_t *font_data,
                                   const char    *chars_map,
                                   uint8_t        font_w,
                                   uint8_t        font_h)
{
    uint8_t cx = x;

    while (*str)
    {
        /* Search for this UTF-8 character in chars_map */
        const char *p     = chars_map;
        uint8_t     idx   = 0;
        uint8_t     found = 0;

        while (*p)
        {
            if (p[0] == str[0] && p[1] == str[1] && p[2] == str[2])
            {
                found = 1;
                break;
            }
            p += 3;
            idx++;
        }

        if (found)
        {
            uint8_t char_pages  = font_h / 8;
            uint8_t glyph_bytes = font_w * char_pages;

            for (uint8_t col = 0; col < font_w; col++)
            {
                uint8_t col_x = cx + col;
                if (col_x >= SSD1306_WIDTH)
                    break;

                for (uint8_t pg = 0; pg < char_pages; pg++)
                {
                    uint8_t page_y = page + pg;
                    if (page_y >= SSD1306_PAGES)
                        break;

                    uint16_t font_idx = (uint16_t)idx * glyph_bytes + (uint16_t)col * char_pages + pg;
                    uint8_t  byte_val = font_data[font_idx];

                    for (uint8_t bit = 0; bit < 8; bit++)
                    {
                        if (byte_val & (1U << bit))
                        {
                            SSD1306_SetPixel(col_x, page_y * 8 + bit, 1);
                        }
                    }
                }
            }
        }

        cx += font_w;

        /* Advance UTF-8 pointer */
        uint8_t byte0 = (uint8_t)*str;
        if ((byte0 & 0x80U) == 0U)
        {
            str += 1;
        }
        else if ((byte0 & 0xE0U) == 0xC0U)
        {
            str += 2;
        }
        else
        {
            str += 3;
        }
    }
}
