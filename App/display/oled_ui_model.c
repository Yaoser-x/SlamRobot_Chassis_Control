#include "oled_ui_model.h"

void OLED_UI_ModelBuild(const system_snapshot_t *snapshot,
                        uint32_t                 now_ms,
                        uint32_t                 calibration_terminal_since_ms,
                        oled_ui_model_t         *model)
{
    if (snapshot == 0 || model == 0)
    {
        return;
    }
    *model                   = (oled_ui_model_t){0};
    model->timestamp_ms      = now_ms;
    model->battery_voltage   = snapshot->safety.battery_voltage;
    model->error_flags       = snapshot->safety.error_flags;
    model->control_source    = snapshot->control.active_source;
    model->esp_download_mode = snapshot->communication.esp12f.download_mode;
    model->tim_break_active  = snapshot->safety.tim_break_active;
    model->imu_chip_id       = snapshot->imu.chip_id;
    model->motor_fault_mask  = snapshot->safety.motor_fault_mask;
    model->modules           = snapshot->modules;
    model->calibration_state = snapshot->imu.calibration_state;
    model->calibration       = OLED_CalibrationView_Build(snapshot->imu.calibration_state,
                                                    snapshot->imu.calibration_sample_count,
                                                    500U,
                                                    snapshot->imu.calibration_fail_reason,
                                                    now_ms,
                                                    calibration_terminal_since_ms);
}
