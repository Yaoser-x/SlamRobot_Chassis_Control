#include "oled_page_calibration.h"

#include "oled_font_8x16.h"
#include "ssd1306.h"

#include <stdio.h>

uint8_t OLED_PageCalibration_Draw(const oled_ui_model_t *model)
{
    char progress[16];

    if (model == 0 || model->calibration.visible == 0U)
    {
        return 0U;
    }
    SSD1306_DrawString(0, 0, model->calibration.title, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
    SSD1306_DrawString(0, 2, model->calibration.detail, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
    (void)snprintf(progress, sizeof(progress), "PROGRESS %3u%%", model->calibration.progress_percent);
    SSD1306_DrawString(0, 4, progress, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
    return 1U;
}
