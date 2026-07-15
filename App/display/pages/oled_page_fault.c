#include "oled_page_fault.h"

#include "oled_font_8x16.h"
#include "ssd1306.h"

void OLED_PageFault_Draw(const oled_ui_model_t *model, uint32_t selfcheck_errors, uint8_t blink_visible)
{
    uint32_t err = model->error_flags | (selfcheck_errors & 0xFFFF0000UL);

    if (blink_visible != 0U || err == 0U)
    {
        char err_buf[7];

        if (model->tim_break_active != 0U)
        {
            err_buf[0] = 'B';
            err_buf[1] = 'K';
            err_buf[2] = 'I';
            err_buf[3] = 'N';
            err_buf[4] = '!';
            err_buf[5] = ' ';
        }
        else
        {
            err_buf[0] = '0';
            err_buf[1] = 'x';
            for (uint8_t nibble_index = 0U; nibble_index < 4U; ++nibble_index)
            {
                uint8_t nibble = (uint8_t)((err >> (12U - nibble_index * 4U)) & 0x0FU);
                if (nibble < 10U)
                {
                    err_buf[2U + nibble_index] = (char)('0' + (int)nibble);
                }
                else
                {
                    err_buf[2U + nibble_index] = (char)('A' + (int)nibble - 10);
                }
            }
        }
        err_buf[6] = '\0';
        SSD1306_DrawString(0, 4, err_buf, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
    }
    {
        static const char *source_names[] = {"NONE", "RPI ", "PS2 ", "ESP ", "DBG ", "LINE"};
        uint8_t            source         = model->control_source;
        const char        *name           = (source <= 5U) ? source_names[source] : "????";
        char               buffer[12];

        buffer[0] = 'M';
        buffer[1] = ':';
        buffer[2] = name[0];
        buffer[3] = name[1];
        buffer[4] = name[2];
        buffer[5] = name[3];
        buffer[6] = ' ';
        buffer[7] = '\0';
        SSD1306_DrawString(56, 4, buffer, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
    }
}
