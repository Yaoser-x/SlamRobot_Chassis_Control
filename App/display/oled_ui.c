/*
 * Copyright 2026 Your Name. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "oled_ui.h"

#include "robot_config.h"
#include "app_publish_model.h"
#include "oled_page_runtime.h"
#include "oled_page_selfcheck.h"
#include "oled_page_welcome.h"
#include "oled_selfcheck.h"
#include "oled_ui_model.h"
#include "ssd1306.h"
#include "cmsis_os2.h"

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

static const uint32_t sc_error_bits[SC_COUNT] = {OLED_SC_ERROR_I2C,
                                                 OLED_SC_ERROR_IMU,
                                                 OLED_SC_ERROR_ADC,
                                                 OLED_SC_ERROR_MOTOR,
                                                 OLED_SC_ERROR_ENCODER,
                                                 OLED_SC_ERROR_UART3_RPI,
                                                 OLED_SC_ERROR_UART4_LINE,
                                                 OLED_SC_ERROR_ESP12F};

/* ================================================================
 *  UI State Variables
 * ================================================================ */
static oled_phase_t ui_phase;
static uint32_t     phase_start_tick;
static uint32_t     selfcheck_errors;
static uint8_t      selfcheck_results[SC_COUNT];
static uint8_t      selfcheck_current;
static uint32_t     selfcheck_item_tick;
static uint8_t      selfcheck_done_flag;

/* Error code blink */
static uint32_t blink_tick;
static uint8_t  blink_visible;
static uint8_t  last_calibration_state;
static uint32_t calibration_terminal_since_ms;

/* ================================================================
 *  Self-Check Execution
 * ================================================================ */
static uint8_t OLED_UI_RunSelfCheck(selfcheck_item_t item, const oled_ui_model_t *model)
{
    switch (item)
    {
        case SC_I2C:
            return (SSD1306_IsReady() != 0U) ? OLED_SELFCHECK_OK : OLED_SELFCHECK_FAIL;
        case SC_IMU:
            return (model->imu_chip_id == 0x24U) ? OLED_SELFCHECK_OK : OLED_SELFCHECK_FAIL;
        case SC_ADC:
            return (model->battery_voltage > 6.0f) ? OLED_SELFCHECK_OK : OLED_SELFCHECK_FAIL;
        case SC_MOTOR:
            return (model->motor_fault_mask == 0U) ? OLED_SELFCHECK_OK : OLED_SELFCHECK_FAIL;
        case SC_ENCODER:
            return (model->modules.encoder_online != 0U) ? OLED_SELFCHECK_OK : OLED_SELFCHECK_FAIL;
        case SC_UART3_RPI:
            return (model->modules.upper_online != 0U) ? OLED_SELFCHECK_OK : OLED_SELFCHECK_FAIL;
        case SC_UART4_LINE:
            return (model->modules.line_online != 0U) ? OLED_SELFCHECK_OK : OLED_SELFCHECK_FAIL;
        case SC_ESP12F:
            if (model->esp_download_mode != 0U)
            {
                return OLED_SELFCHECK_SKIP;
            }
            return (model->modules.esp12f_online != 0U) ? OLED_SELFCHECK_OK : OLED_SELFCHECK_FAIL;
        default:
            return OLED_SELFCHECK_FAIL;
    }
}

/* ================================================================
 *  Welcome Screen
 * ================================================================ */
/* ================================================================
 *  Public API
 * ================================================================ */

void OLED_UI_Init(void)
{
    ui_phase            = OLED_PHASE_WELCOME;
    phase_start_tick    = osKernelGetTickCount();
    selfcheck_errors    = 0U;
    selfcheck_done_flag = 0U;

    for (uint8_t i = 0; i < SC_COUNT; i++)
    {
        selfcheck_results[i] = 0U;
    }
    selfcheck_current   = 0U;
    selfcheck_item_tick = 0U;

    blink_tick                    = 0U;
    blink_visible                 = 1U;
    last_calibration_state        = SYSTEM_IMU_CAL_DISABLED;
    calibration_terminal_since_ms = 0U;
}

void OLED_UI_Update(void)
{
    communication_publish_model_t snapshot;
    const app_display_config_t   *display = &RobotConfig_GetDefault()->display;
    oled_ui_model_t               model;
    uint32_t                      now     = osKernelGetTickCount();
    uint32_t                      elapsed = now - phase_start_tick;

    (void)AppPublishModel_Get(&snapshot);
    if (snapshot.imu.calibration_state != last_calibration_state)
    {
        last_calibration_state        = snapshot.imu.calibration_state;
        calibration_terminal_since_ms = (snapshot.imu.calibration_state == SYSTEM_IMU_CAL_DONE
                                         || snapshot.imu.calibration_state == SYSTEM_IMU_CAL_FAILED)
                                            ? now
                                            : 0U;
    }
    OLED_UI_ModelBuild(&snapshot, now, calibration_terminal_since_ms, &model);

    SSD1306_Clear();

    switch (ui_phase)
    {
        case OLED_PHASE_WELCOME:
        {
            OLED_PageWelcome_Draw();
            if (elapsed >= display->welcome_duration_ms)
            {
                ui_phase            = OLED_PHASE_SELFCHECK;
                phase_start_tick    = now;
                selfcheck_current   = 0U;
                selfcheck_item_tick = now;
                selfcheck_errors    = 0U;
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
            if ((now - selfcheck_item_tick) >= display->selfcheck_item_ms)
            {
                uint8_t result = OLED_UI_RunSelfCheck((selfcheck_item_t)selfcheck_current, &model);
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
                    ui_phase            = OLED_PHASE_NORMAL;
                    phase_start_tick    = now;
                }
            }
            OLED_PageSelfcheck_Draw(selfcheck_current, selfcheck_results);
            break;
        }

        case OLED_PHASE_NORMAL:
        {
            if ((now - blink_tick) >= display->error_blink_period_ms)
            {
                blink_tick    = now;
                blink_visible = (blink_visible == 0U) ? 1U : 0U;
            }

            OLED_PageRuntime_Draw(&model, selfcheck_errors, blink_visible);
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
