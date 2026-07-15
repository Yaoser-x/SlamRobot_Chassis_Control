#ifndef OLED_UI_MODEL_H
#define OLED_UI_MODEL_H

#include <stdint.h>

#include "oled_calibration_view.h"
#include "system_snapshot_service.h"

typedef struct
{
    uint32_t                 timestamp_ms;
    float                    battery_voltage;
    uint32_t                 error_flags;
    uint8_t                  control_source;
    uint8_t                  esp_download_mode;
    uint8_t                  tim_break_active;
    uint8_t                  imu_chip_id;
    uint8_t                  motor_fault_mask;
    module_health_snapshot_t modules;
    uint8_t                  calibration_state;
    oled_calibration_view_t  calibration;
} oled_ui_model_t;

/** Convert one coherent system snapshot into the OLED presentation model. */
void OLED_UI_ModelBuild(const system_snapshot_t *snapshot,
                        uint32_t                 now_ms,
                        uint32_t                 calibration_terminal_since_ms,
                        oled_ui_model_t         *model);

#endif
