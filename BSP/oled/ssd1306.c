/*
 * Copyright 2026 Your Name. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "ssd1306.h"
#include "chassis_config.h"
#include "i2c.h"
#include <string.h>

static uint8_t framebuffer[SSD1306_WIDTH * SSD1306_PAGES];

/* --- Internal I2C command write --- */
static void SSD1306_WriteCmd(uint8_t cmd)
{
  HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(OLED_I2C_ADDR << 1),
                    0x00, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 10);
}

/* --- Internal I2C data write --- */
static void SSD1306_WriteData(uint8_t data)
{
  HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(OLED_I2C_ADDR << 1),
                    0x40, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

/* --- Initialization sequence --- */
void SSD1306_Init(void)
{
  /* Wait for power-up stability */
  HAL_Delay(100);

  /* Command sequence per SSD1306 datasheet Rev 2.2, Section "Initialization" */
  SSD1306_WriteCmd(0xAE); /* display off */

  SSD1306_WriteCmd(0xD5); /* set display clock divide ratio/oscillator frequency */
  SSD1306_WriteCmd(0x80);

  SSD1306_WriteCmd(0xA8); /* set multiplex ratio */
  SSD1306_WriteCmd(0x3F); /* 64 */

  SSD1306_WriteCmd(0xD3); /* set display offset */
  SSD1306_WriteCmd(0x00);

  SSD1306_WriteCmd(0x40); /* set display start line */

  SSD1306_WriteCmd(0x8D); /* charge pump setting */
  SSD1306_WriteCmd(0x14); /* enable charge pump */

  SSD1306_WriteCmd(0x20); /* memory addressing mode */
  SSD1306_WriteCmd(0x00); /* horizontal */

  SSD1306_WriteCmd(0xA1); /* segment re-map (column 127 mapped to SEG0) */
  SSD1306_WriteCmd(0xC8); /* COM output scan direction (remapped) */

  SSD1306_WriteCmd(0xDA); /* set COM pins hardware configuration */
  SSD1306_WriteCmd(0x12);

  SSD1306_WriteCmd(0x81); /* set contrast control */
  SSD1306_WriteCmd(0xCF);

  SSD1306_WriteCmd(0xD9); /* set pre-charge period */
  SSD1306_WriteCmd(0xF1);

  SSD1306_WriteCmd(0xDB); /* set VCOMH deselect level */
  SSD1306_WriteCmd(0x40);

  SSD1306_WriteCmd(0xA4); /* entire display on follows RAM content */
  SSD1306_WriteCmd(0xA6); /* normal display (not inverted) */

  SSD1306_WriteCmd(0x2E); /* deactivate scroll */

  SSD1306_Clear();
  SSD1306_Refresh();

  SSD1306_WriteCmd(0xAF); /* display on */
}

/* --- Clear framebuffer --- */
void SSD1306_Clear(void)
{
  memset(framebuffer, 0x00, sizeof(framebuffer));
}

/* --- Full screen refresh (page-mode bulk write, 8 transactions) --- */
void SSD1306_Refresh(void)
{
  uint8_t page;

  for (page = 0; page < SSD1306_PAGES; page++)
  {
    /* Set page address (0xB0~0xB7) */
    SSD1306_WriteCmd((uint8_t)(0xB0 | page));
    /* Set column address to 0 */
    SSD1306_WriteCmd(0x00); /* column low nibble */
    SSD1306_WriteCmd(0x10); /* column high nibble = 0 */

    /* Send 128 bytes for this page in ONE I2C transaction */
    (void)HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(OLED_I2C_ADDR << 1),
                            0x40, I2C_MEMADD_SIZE_8BIT,
                            &framebuffer[(uint16_t)page * SSD1306_WIDTH],
                            SSD1306_WIDTH, 20);
  }
}

/* --- Set single pixel --- */
void SSD1306_SetPixel(uint8_t x, uint8_t y, uint8_t on)
{
  if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

  uint16_t idx = (uint16_t)(y / 8) * SSD1306_WIDTH + x;
  if (on)
    framebuffer[idx] |= (uint8_t)(1U << (y & 0x07));
  else
    framebuffer[idx] &= (uint8_t)(~(1U << (y & 0x07)));
}

/* --- Draw character (page-major font format) --- */
void SSD1306_DrawChar(uint8_t x, uint8_t page, char ch,
                      const uint8_t *font, uint8_t font_w, uint8_t font_h,
                      uint8_t font_start)
{
  uint8_t char_pages = font_h / 8;

  for (uint8_t col = 0; col < font_w; col++)
  {
    uint8_t col_x = x + col;
    if (col_x >= SSD1306_WIDTH) break;

    for (uint8_t pg = 0; pg < char_pages; pg++)
    {
      uint8_t page_y = page + pg;
      if (page_y >= SSD1306_PAGES) break;

      /* Row-major (page-major): glyph's (pg)-th page, (col)-th column */
      uint16_t font_idx = (uint16_t)(ch - font_start) * (uint16_t)(font_w * char_pages)
                        + (uint16_t)pg * font_w + col;
      uint16_t fb_idx = (uint16_t)page_y * SSD1306_WIDTH + col_x;

      framebuffer[fb_idx] = font[font_idx];
    }
  }
}

/* --- Draw ASCII string (8x16 font default) --- */
void SSD1306_DrawString(uint8_t x, uint8_t page, const char *str,
                        const uint8_t *font, uint8_t font_w, uint8_t font_h,
                        uint8_t font_start)
{
  uint8_t cur_x = x;
  while (*str)
  {
    SSD1306_DrawChar(cur_x, page, *str, font, font_w, font_h, font_start);
    cur_x += font_w;
    str++;
  }
}

/* --- Fill rectangle --- */
void SSD1306_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t on)
{
  for (uint8_t dy = 0; dy < h; dy++)
  {
    for (uint8_t dx = 0; dx < w; dx++)
    {
      SSD1306_SetPixel(x + dx, y + dy, on);
    }
  }
}

/* --- Progress bar --- */
void SSD1306_DrawProgressBar(uint8_t x, uint8_t y, uint8_t w, uint8_t h,
                             uint8_t percent)
{
  if (percent > 100) percent = 100;

  uint8_t fill_w = (uint8_t)(((uint16_t)w * percent) / 100U);

  /* Outer frame */
  SSD1306_FillRect(x, y, w, h, 0);
  for (uint8_t dx = 0; dx < w; dx++)
  {
    SSD1306_SetPixel(x + dx, y, 1);
    SSD1306_SetPixel(x + dx, y + h - 1, 1);
  }
  for (uint8_t dy = 0; dy < h; dy++)
  {
    SSD1306_SetPixel(x, y + dy, 1);
    SSD1306_SetPixel(x + w - 1, y + dy, 1);
  }

  /* Fill */
  if (fill_w > 2)
  {
    SSD1306_FillRect(x + 1, y + 1, fill_w - 2, h - 2, 1);
  }
}
