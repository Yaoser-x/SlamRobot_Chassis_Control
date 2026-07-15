#include "oled_page_runtime.h"
#include "oled_page_calibration.h"
#include "oled_page_fault.h"

#include "ssd1306.h"
#include "oled_font_8x16.h"
#include "oled_font_16x16.h"
#include "oled_text_renderer.h"

void OLED_PageRuntime_Draw(const oled_ui_model_t *model, uint32_t selfcheck_errors, uint8_t blink_visible)
{
    uint32_t now = model->timestamp_ms;

    if (OLED_PageCalibration_Draw(model) != 0U)
    {
        return;
    }

    /* Row 0: Runtime (pages 0~1) */
    {
        uint32_t sec = now / 1000U;
        uint8_t  hh  = (uint8_t)((sec / 3600U) % 100U);
        uint8_t  mm  = (uint8_t)((sec / 60U) % 60U);
        uint8_t  ss  = (uint8_t)(sec % 60U);

        char time_buf[9];
        time_buf[0] = (char)('0' + hh / 10);
        time_buf[1] = (char)('0' + hh % 10);
        time_buf[2] = ':';
        time_buf[3] = (char)('0' + mm / 10);
        time_buf[4] = (char)('0' + mm % 10);
        time_buf[5] = ':';
        time_buf[6] = (char)('0' + ss / 10);
        time_buf[7] = (char)('0' + ss % 10);
        time_buf[8] = '\0';

        SSD1306_DrawString(0, 0, time_buf, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);

        /* "运行时间" 16x16 Chinese, x=64 (4×16=64px, ends at x=128) */
        OLED_TextRenderer_DrawChinese(64, 0, "运行时间", OLED_FONT_16X16_DATA, OLED_FONT_16X16_CHARS, 16, 16);
    }

    /* Row 1: Battery voltage (pages 2~3) */
    {
        char     volt_buf[8];
        uint16_t v_int = (uint16_t)model->battery_voltage;
        uint16_t v_dec = (uint16_t)((model->battery_voltage - (float)v_int) * 10.0f + 0.5f);
        volt_buf[0]    = (char)('0' + v_int / 10);
        volt_buf[1]    = (char)('0' + v_int % 10);
        volt_buf[2]    = '.';
        volt_buf[3]    = (char)('0' + v_dec);
        volt_buf[4]    = 'V';
        volt_buf[5]    = '\0';

        SSD1306_DrawString(0, 2, volt_buf, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);

        /* "电池" 16x16 Chinese, x=88 */
        OLED_TextRenderer_DrawChinese(88, 2, "电池", OLED_FONT_16X16_DATA, OLED_FONT_16X16_CHARS, 16, 16);
    }

    OLED_PageFault_Draw(model, selfcheck_errors, blink_visible);
}
