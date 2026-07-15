#include "oled_page_selfcheck.h"

#include "oled_selfcheck.h"
#include "ssd1306.h"
#include "oled_font_8x16.h"
#include "oled_font_16x16.h"
#include "oled_text_renderer.h"

static const char *const sc_names[OLED_PAGE_SELFCHECK_ITEM_COUNT] =
    {"I2C ", "IMU ", "ADC ", "Motr", "Enc ", "RPI ", "Line", "ESP "};

void OLED_PageSelfcheck_Draw(uint8_t current, const uint8_t results[OLED_PAGE_SELFCHECK_ITEM_COUNT])
{
    /* Title: "系统自检中" 16x16 Chinese, pages 0-1 (y=0~15) */
    {
        const char *title = "系统自检中";
        uint8_t     cx    = (uint8_t)((128 - 5 * 16) / 2);
        OLED_TextRenderer_DrawChinese(cx, 0, title, OLED_FONT_16X16_DATA, OLED_FONT_16X16_CHARS, 16, 16);
    }

    /* Current module name — pages 2-3 (y=16~31), centered */
    uint8_t sci = current;
    if (sci >= OLED_PAGE_SELFCHECK_ITEM_COUNT)
        sci = 0;
    SSD1306_DrawString(48, 2, sc_names[sci], OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);

    /* Page indicator "[N/8]" — pages 4-5 (y=32~47), full 8x16 visible */
    {
        char    buf[8];
        uint8_t item = sci + 1U;
        buf[0]       = '[';
        buf[1]       = (char)('0' + item);
        buf[2]       = '/';
        buf[3]       = (char)('0' + OLED_PAGE_SELFCHECK_ITEM_COUNT);
        buf[4]       = ']';
        buf[5]       = '\0';
        SSD1306_DrawString(52, 4, buf, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
    }

    /* Result: OK or FAIL — pages 6-7 (y=48~63), centered */
    if (results[sci] == 1U)
    {
        SSD1306_DrawString(52, 6, " OK ", OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
    }
    else if (results[sci] == OLED_SELFCHECK_FAIL)
    {
        SSD1306_DrawString(48, 6, "FAIL", OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
    }
    else if (results[sci] == OLED_SELFCHECK_SKIP)
    {
        SSD1306_DrawString(48, 6, "SKIP", OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
    }
    else
    {
        SSD1306_DrawString(52, 6, " ...", OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
    }
}

/* ================================================================
 *  Normal Runtime Screen
 * ================================================================ */
