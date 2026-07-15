#include "oled_page_welcome.h"

#include "ssd1306.h"
#include "oled_font_8x16.h"
#include "oled_font_16x16.h"
#include "oled_text_renderer.h"

void OLED_PageWelcome_Draw(void)
{
    /* "F407 V2.0" large title (pages 0-1, y=0~15, centered ~ x=32) */
    SSD1306_DrawString(32, 0, "F407 V2.0", OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);

    /* Separator line at y=16 (page boundary between title and subtitle) */
    for (uint8_t cx = 16; cx < 112; cx++)
    {
        SSD1306_SetPixel(cx, 16, 1);
    }

    /* Subtitle: "四轮差速底盘控制" 16x16 Chinese, pages 2-3 (y=16~31) */
    {
        const char *line1 = "四轮差速底盘控制";
        uint8_t     cx    = (uint8_t)((128 - 8 * 16) / 2);
        OLED_TextRenderer_DrawChinese(cx, 2, line1, OLED_FONT_16X16_DATA, OLED_FONT_16X16_CHARS, 16, 16);
    }

    /* "STM32F407VET6" (pages 4-5, y=32~47, centered) */
    SSD1306_DrawString(24, 4, "STM32F407VET6", OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);

    /* "系统启动中..." (pages 6-7, y=48~63, centered) */
    {
        const char *line3 = "系统启动中";
        uint8_t     cx    = (uint8_t)((128 - 5 * 16) / 2);
        OLED_TextRenderer_DrawChinese(cx, 6, line3, OLED_FONT_16X16_DATA, OLED_FONT_16X16_CHARS, 16, 16);
    }
}

/* ================================================================
 *  Self-Check Screen
 * ================================================================ */
