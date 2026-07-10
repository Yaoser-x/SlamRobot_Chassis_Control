/*
 * Copyright 2026 Your Name. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "oled_ui.h"

#include "ssd1306.h"
#include "oled_font_8x16.h"
#include "oled_font_12x12.h"
#include "oled_font_16x16.h"
#include "i2c.h"
#include "main.h"
#include "cmsis_os2.h"
#include "chassis_config.h"
#include "system_monitor.h"
#include "control_manager.h"
#include "ps2_control.h"
#include "imu_bmi270.h"
#include "encoder_driver.h"
#include "motor_driver.h"
#include "line_uart.h"
#include "esp12f_comm.h"
#include "adc_monitor.h"
#include "upper_uart.h"
#include "oled_selfcheck.h"

/* ================================================================
 *  Self-Check Item Definitions
 * ================================================================ */
typedef enum
{
  SC_I2C = 0,
  SC_IMU,
  SC_ADC,
  SC_MOTOR,
  SC_ENCODER,
  SC_UART3_RPI,
  SC_UART4_LINE,
  SC_ESP12F,
  SC_COUNT
} selfcheck_item_t;

static const char *const sc_names[SC_COUNT] = {
  "I2C ",
  "IMU ",
  "ADC ",
  "Motr",
  "Enc ",
  "RPI ",
  "Line",
  "ESP "
};

static const uint32_t sc_error_bits[SC_COUNT] = {
  OLED_SC_ERROR_I2C,
  OLED_SC_ERROR_IMU,
  OLED_SC_ERROR_ADC,
  OLED_SC_ERROR_MOTOR,
  OLED_SC_ERROR_ENCODER,
  OLED_SC_ERROR_UART3_RPI,
  OLED_SC_ERROR_UART4_LINE,
  OLED_SC_ERROR_ESP12F
};

/* ================================================================
 *  UI State Variables
 * ================================================================ */
static oled_phase_t  ui_phase;
static uint32_t      phase_start_tick;
static uint32_t      selfcheck_errors;
static uint8_t       selfcheck_results[SC_COUNT];
static uint8_t       selfcheck_current;
static uint32_t      selfcheck_item_tick;
static uint8_t       selfcheck_done_flag;

/* Module online status (updated each frame) */
static uint8_t mod_rpi_online;
static uint8_t mod_ps2_online;
static uint8_t mod_imu_online;
static uint8_t mod_line_online;
static uint8_t mod_enc_online;
static uint8_t mod_esp_online;
static uint8_t mod_motr_online;
static uint8_t mod_adc_online;

/* Error code blink */
static uint32_t blink_tick;
static uint8_t  blink_visible;

/* ================================================================
 *  Chinese String Drawing Helper
 *
 *  Parses UTF-8 input, searches chars_map[] for a matching entry,
 *  and renders the glyph bitmap via SSD1306_SetPixel.
 *  Font format: column-major, bit0=top, left-to-right top-to-bottom.
 * ================================================================ */
static void OLED_UI_DrawChineseStr(uint8_t x, uint8_t page, const char *str,
                                   const uint8_t *font_data,
                                   const char *chars_map,
                                   uint8_t font_w, uint8_t font_h)
{
  uint8_t cx = x;

  while (*str)
  {
    /* Search for this UTF-8 character in chars_map */
    const char *p = chars_map;
    uint8_t      idx = 0;
    uint8_t      found = 0;

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
      uint8_t char_pages = font_h / 8;
      uint8_t glyph_bytes = font_w * char_pages;

      for (uint8_t col = 0; col < font_w; col++)
      {
        uint8_t col_x = cx + col;
        if (col_x >= SSD1306_WIDTH) break;

        for (uint8_t pg = 0; pg < char_pages; pg++)
        {
          uint8_t page_y = page + pg;
          if (page_y >= SSD1306_PAGES) break;

          uint16_t font_idx = (uint16_t)idx * glyph_bytes
                            + (uint16_t)col * char_pages + pg;
          uint8_t byte_val = font_data[font_idx];

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
    if ((byte0 & 0x80U) == 0U)           { str += 1; }
    else if ((byte0 & 0xE0U) == 0xC0U)   { str += 2; }
    else                                 { str += 3; }
  }
}

/* ================================================================
 *  Self-Check Execution
 * ================================================================ */
static uint8_t OLED_UI_RunSelfCheck(selfcheck_item_t item)
{
  switch (item)
  {
    case SC_I2C:
    {
      HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1,
                                   (uint16_t)(OLED_I2C_ADDR << 1), 2, 10);
      return (status == HAL_OK) ? 1U : 2U;
    }

    case SC_IMU:
    {
      imu_bmi270_state_t imu_state;
      ImuBmi270_GetState(&imu_state);
      return (imu_state.chip_id == 0x24U) ? 1U : 2U;
    }

    case SC_ADC:
    {
      system_monitor_state_t mon;
      SystemMonitor_GetState(&mon);
      return (mon.battery_voltage > 6.0f) ? 1U : 2U;
    }

    case SC_MOTOR:
    {
      motor_driver_state_t motor_state;
      MotorDriver_GetState(&motor_state);
      for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
      {
        if (motor_state.fault_active[i] != 0U)
        {
          return 2U;
        }
      }
      return 1U;
    }

    case SC_ENCODER:
    {
      encoder_state_t enc_state;
      EncoderDriver_GetState(&enc_state);
      return enc_state.speed_valid_all ? 1U : 2U;
    }

    case SC_UART3_RPI:
    {
      uint32_t now = osKernelGetTickCount();
      return (uint8_t)OLED_SelfCheckRpi(now,
                                        UpperUart_GetLastRxTimestamp(),
                                        OLED_MODULE_TIMEOUT_RPI_MS);
    }

    case SC_UART4_LINE:
    {
      uint32_t now = osKernelGetTickCount();
      line_sensor_data_t sensor;
      uint8_t valid = LineUart_GetSensorData(&sensor);
      return (uint8_t)OLED_SelfCheckLine(now,
                                         valid,
                                         sensor.timestamp_ms,
                                         OLED_MODULE_TIMEOUT_LINE_MS);
    }

    case SC_ESP12F:
    {
      esp12f_comm_state_t esp_state;
      Esp12fComm_GetState(&esp_state);
      return (uint8_t)OLED_SelfCheckEsp12f(esp_state.rx_frames,
                                           esp_state.boot_mode_download);
    }

    default:
      return 2U;
  }
}

/* ================================================================
 *  Module Online Status Aggregation
 * ================================================================ */
static void OLED_UI_UpdateModuleStatus(void)
{
  uint32_t now = osKernelGetTickCount();

  /* RPI: UpperUart last RX timestamp */
  mod_rpi_online = ((now - UpperUart_GetLastRxTimestamp()) < OLED_MODULE_TIMEOUT_RPI_MS) ? 1U : 0U;

  /* PS2: controller online flag */
  {
    ps2_control_state_t ps2_state;
    Ps2Control_GetState(&ps2_state);
    mod_ps2_online = ps2_state.online;
  }

  /* IMU: sensor online flag */
  {
    imu_bmi270_state_t imu_state;
    ImuBmi270_GetState(&imu_state);
    mod_imu_online = imu_state.online;
  }

  /* Line: sensor data timestamp */
  {
    line_sensor_data_t sensor;
    if (LineUart_GetSensorData(&sensor))
    {
      mod_line_online = ((now - sensor.timestamp_ms) < OLED_MODULE_TIMEOUT_LINE_MS) ? 1U : 0U;
    }
  }

  /* Encoder: all enabled encoders valid */
  {
    encoder_state_t enc_state;
    EncoderDriver_GetState(&enc_state);
    mod_enc_online = enc_state.speed_valid_all;
  }

  /* ESP: communication active */
  {
    esp12f_comm_state_t esp_state;
    Esp12fComm_GetState(&esp_state);
    mod_esp_online = (esp_state.rx_frames > 0U) ? 1U : 0U;
  }

  /* Motor: nFAULT all normal */
  {
    system_monitor_state_t mon;
    SystemMonitor_GetState(&mon);
    mod_motr_online = ((mon.error_flags & SYSTEM_ERROR_DRV_FAULT) == 0U) ? 1U : 0U;
  }

  /* ADC: voltage sampling valid */
  {
    adc_monitor_state_t adc_state;
    AdcMonitor_GetState(&adc_state);
    mod_adc_online = adc_state.current_valid;
  }
}

/* ================================================================
 *  Welcome Screen
 * ================================================================ */
static void OLED_UI_DrawWelcome(void)
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
    uint8_t cx = (uint8_t)((128 - 8 * 16) / 2);
    OLED_UI_DrawChineseStr(cx, 2, line1, OLED_FONT_16X16_DATA,
                           OLED_FONT_16X16_CHARS, 16, 16);
  }

  /* "STM32F407VET6" (pages 4-5, y=32~47, centered) */
  SSD1306_DrawString(24, 4, "STM32F407VET6", OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);

  /* "系统启动中..." (pages 6-7, y=48~63, centered) */
  {
    const char *line3 = "系统启动中";
    uint8_t cx = (uint8_t)((128 - 5 * 16) / 2);
    OLED_UI_DrawChineseStr(cx, 6, line3, OLED_FONT_16X16_DATA,
                           OLED_FONT_16X16_CHARS, 16, 16);
  }
}

/* ================================================================
 *  Self-Check Screen
 * ================================================================ */
static void OLED_UI_DrawSelfCheck(void)
{
  /* Title: "系统自检中" 16x16 Chinese, pages 0-1 (y=0~15) */
  {
    const char *title = "系统自检中";
    uint8_t cx = (uint8_t)((128 - 5 * 16) / 2);
    OLED_UI_DrawChineseStr(cx, 0, title, OLED_FONT_16X16_DATA,
                           OLED_FONT_16X16_CHARS, 16, 16);
  }

  /* Current module name — pages 2-3 (y=16~31), centered */
  uint8_t sci = selfcheck_current;
  if (sci >= SC_COUNT) sci = 0;
  SSD1306_DrawString(48, 2, sc_names[sci], OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);

  /* Page indicator "[N/8]" — pages 4-5 (y=32~47), full 8x16 visible */
  {
    char buf[8];
    uint8_t item = sci + 1U;
    buf[0] = '[';
    buf[1] = (char)('0' + item);
    buf[2] = '/';
    buf[3] = (char)('0' + SC_COUNT);
    buf[4] = ']';
    buf[5] = '\0';
    SSD1306_DrawString(52, 4, buf, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
  }

  /* Result: OK or FAIL — pages 6-7 (y=48~63), centered */
  if (selfcheck_results[sci] == 1U)
  {
    SSD1306_DrawString(52, 6, " OK ", OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
  }
  else if (selfcheck_results[sci] == OLED_SELFCHECK_FAIL)
  {
    SSD1306_DrawString(48, 6, "FAIL", OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
  }
  else if (selfcheck_results[sci] == OLED_SELFCHECK_SKIP)
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
static void OLED_UI_DrawNormal(void)
{
  uint32_t now = osKernelGetTickCount();
  system_monitor_state_t mon;
  SystemMonitor_GetState(&mon);

  /* Row 0: Runtime (pages 0~1) */
  {
    uint32_t sec = now / 1000U;
    uint8_t hh = (uint8_t)((sec / 3600U) % 100U);
    uint8_t mm = (uint8_t)((sec / 60U) % 60U);
    uint8_t ss = (uint8_t)(sec % 60U);

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
    OLED_UI_DrawChineseStr(64, 0, "运行时间", OLED_FONT_16X16_DATA,
                           OLED_FONT_16X16_CHARS, 16, 16);
  }

  /* Row 1: Battery voltage (pages 2~3) */
  {
    char volt_buf[8];
    uint16_t v_int = (uint16_t)mon.battery_voltage;
    uint16_t v_dec = (uint16_t)((mon.battery_voltage - (float)v_int) * 10.0f + 0.5f);
    volt_buf[0] = (char)('0' + v_int / 10);
    volt_buf[1] = (char)('0' + v_int % 10);
    volt_buf[2] = '.';
    volt_buf[3] = (char)('0' + v_dec);
    volt_buf[4] = 'V';
    volt_buf[5] = '\0';

    SSD1306_DrawString(0, 2, volt_buf, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);

    /* "电池" 16x16 Chinese, x=88 */
    OLED_UI_DrawChineseStr(88, 2, "电池", OLED_FONT_16X16_DATA,
                           OLED_FONT_16X16_CHARS, 16, 16);
  }

  /* Row 2: Error code + control source — pages 4~5 (y=32~47)
   * Combined on one line, full 8x16 visible. */
  {
    /* Error code hex (blinks when non-zero): "0x0000" at x=0 */
    uint32_t err = mon.error_flags | (selfcheck_errors & 0xFFFF0000UL);
    if (blink_visible || err == 0U)
    {
      char err_buf[7];
      err_buf[0] = '0';
      err_buf[1] = 'x';
      for (uint8_t n = 0; n < 4; n++)
      {
        uint8_t nib = (uint8_t)((err >> (12 - n * 4)) & 0x0FU);
        err_buf[2 + n] = (char)(nib < 10 ? '0' + nib : 'A' + nib - 10);
      }
      err_buf[6] = '\0';
      SSD1306_DrawString(0, 4, err_buf, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
    }

    /* Control source: "M:SRC " at x=56 (6 chars × 8px = 48px → ends at 104) */
    uint8_t src = ControlManager_GetActiveSource();
    static const char *src_names[] = {"NONE","RPI ","PS2 ","ESP ","DBG ","LINE"};
    const char *name = (src <= 5) ? src_names[src] : "????";

    char buf[12];
    buf[0] = 'M';
    buf[1] = ':';
    buf[2] = name[0];
    buf[3] = name[1];
    buf[4] = name[2];
    buf[5] = name[3];
    buf[6] = ' ';
    buf[7] = '\0';
    SSD1306_DrawString(56, 4, buf, OLED_FONT_8X16_DATA, 8, 16, OLED_FONT_8X16_START);
  }
}

/* ================================================================
 *  Public API
 * ================================================================ */

void OLED_UI_Init(void)
{
  ui_phase = OLED_PHASE_WELCOME;
  phase_start_tick = osKernelGetTickCount();
  selfcheck_errors = 0U;
  selfcheck_done_flag = 0U;

  for (uint8_t i = 0; i < SC_COUNT; i++)
  {
    selfcheck_results[i] = 0U;
  }
  selfcheck_current = 0U;
  selfcheck_item_tick = 0U;

  mod_rpi_online = 0U;
  mod_ps2_online = 0U;
  mod_imu_online = 0U;
  mod_line_online = 0U;
  mod_enc_online = 0U;
  mod_esp_online = 0U;
  mod_motr_online = 0U;
  mod_adc_online = 0U;

  blink_tick = 0U;
  blink_visible = 1U;
}

void OLED_UI_Update(void)
{
  uint32_t now = osKernelGetTickCount();
  uint32_t elapsed = now - phase_start_tick;

  SSD1306_Clear();

  switch (ui_phase)
  {
    case OLED_PHASE_WELCOME:
    {
      OLED_UI_DrawWelcome();
      if (elapsed >= OLED_WELCOME_DURATION_MS)
      {
        ui_phase = OLED_PHASE_SELFCHECK;
        phase_start_tick = now;
        selfcheck_current = 0U;
        selfcheck_item_tick = now;
        selfcheck_errors = 0U;
        selfcheck_done_flag = 0U;
        for (uint8_t i = 0; i < SC_COUNT; i++)
        {
          selfcheck_results[i] = 0U;
        }
      }
      break;
    }

    case OLED_PHASE_SELFCHECK:
    {
      /* Run one self-check item every OLED_SELFCHECK_ITEM_MS */
      if ((now - selfcheck_item_tick) >= OLED_SELFCHECK_ITEM_MS)
      {
        uint8_t result = OLED_UI_RunSelfCheck((selfcheck_item_t)selfcheck_current);
        selfcheck_results[selfcheck_current] = result;
        if (result == OLED_SELFCHECK_FAIL)
        {
          selfcheck_errors |= sc_error_bits[selfcheck_current];
        }

        selfcheck_current++;
        selfcheck_item_tick = now;

        if (selfcheck_current >= SC_COUNT)
        {
          selfcheck_done_flag = 1U;
          ui_phase = OLED_PHASE_NORMAL;
          phase_start_tick = now;
        }
      }
      OLED_UI_DrawSelfCheck();
      break;
    }

    case OLED_PHASE_NORMAL:
    {
      /* Update module online status every frame */
      OLED_UI_UpdateModuleStatus();

      /* Error code blink: 500ms period */
      if ((now - blink_tick) >= OLED_ERROR_BLINK_PERIOD_MS)
      {
        blink_tick = now;
        blink_visible = (blink_visible == 0U) ? 1U : 0U;
      }

      OLED_UI_DrawNormal();
      break;
    }

    default:
      break;
  }

  SSD1306_Refresh();
}

oled_phase_t OLED_UI_GetPhase(void)
{
  return ui_phase;
}

uint8_t OLED_UI_SelfCheckDone(void)
{
  return selfcheck_done_flag;
}

uint32_t OLED_UI_GetSelfCheckErrors(void)
{
  return selfcheck_errors;
}
