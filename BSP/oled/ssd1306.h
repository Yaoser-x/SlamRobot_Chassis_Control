/*
 * Copyright 2026 Your Name. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_PAGES  (SSD1306_HEIGHT / 8)

    void     SSD1306_Init(void);
    void     SSD1306_Clear(void);
    void     SSD1306_Refresh(void);
    void     SSD1306_SetPixel(uint8_t x, uint8_t y, uint8_t on);
    void     SSD1306_DrawChar(uint8_t        x,
                              uint8_t        page,
                              char           ch,
                              const uint8_t *font,
                              uint8_t        font_w,
                              uint8_t        font_h,
                              uint8_t        font_start);
    void     SSD1306_DrawString(uint8_t        x,
                                uint8_t        page,
                                const char    *str,
                                const uint8_t *font,
                                uint8_t        font_w,
                                uint8_t        font_h,
                                uint8_t        font_start);
    void     SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t on);
    void     SSD1306_DrawProgressBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t percent);
    uint32_t SSD1306_GetI2cErrorCount(void);
    uint32_t SSD1306_GetI2cRecoveryCount(void);

#ifdef __cplusplus
}
#endif

#endif /* SSD1306_H */
