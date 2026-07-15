#include "oled_ui_model.h"

#include <stdio.h>
#include <stdlib.h>

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    system_snapshot_t snapshot = {0};
    oled_ui_model_t   model;

    snapshot.safety.battery_voltage             = 12.4f;
    snapshot.safety.error_flags                 = 0x1234U;
    snapshot.safety.tim_break_active            = 1U;
    snapshot.safety.motor_fault_mask            = 0x05U;
    snapshot.control.active_source              = 3U;
    snapshot.communication.esp12f.download_mode = 1U;
    snapshot.imu.chip_id                        = 0x24U;
    snapshot.imu.calibration_state              = SYSTEM_IMU_CAL_RUNNING;
    snapshot.imu.calibration_sample_count       = 125U;
    snapshot.modules.imu_online                 = 1U;
    snapshot.modules.encoder_online             = 1U;

    OLED_UI_ModelBuild(&snapshot, 7000U, 0U, &model);

    require_int(model.timestamp_ms == 7000U, "timestamp mapping");
    require_int(model.battery_voltage == 12.4f, "battery mapping");
    require_int(model.error_flags == 0x1234U, "fault mapping");
    require_int(model.control_source == 3U, "control source mapping");
    require_int(model.esp_download_mode == 1U, "download mode mapping");
    require_int(model.tim_break_active == 1U, "break mapping");
    require_int(model.imu_chip_id == 0x24U, "chip id mapping");
    require_int(model.motor_fault_mask == 0x05U, "motor fault mapping");
    require_int(model.modules.encoder_online == 1U, "module health mapping");
    require_int(model.calibration.visible == 1U, "calibration visibility");
    require_int(model.calibration.progress_percent == 25U, "calibration progress");

    puts("PASS: OLED UI model");
    return 0;
}
