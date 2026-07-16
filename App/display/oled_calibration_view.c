#include "oled_calibration_view.h"

#include "communication_publish_model_types.h"

#define OLED_CALIBRATION_RETURN_MS 3000U

static const char *OLED_CalibrationView_FailReason(uint8_t reason)
{
    switch (reason)
    {
        case SYSTEM_IMU_CAL_FAIL_CONFIG:
            return "CONFIG ERROR";
        case SYSTEM_IMU_CAL_FAIL_READ:
            return "READ ERROR";
        case SYSTEM_IMU_CAL_FAIL_ABS:
            return "BIAS TOO HIGH";
        case SYSTEM_IMU_CAL_FAIL_SPAN:
            return "NOISE TOO HIGH";
        case SYSTEM_IMU_CAL_FAIL_MOTION:
            return "KEEP STILL";
        default:
            return "STILL GATE";
    }
}

oled_calibration_view_t OLED_CalibrationView_Build(uint8_t  auto_cal_state,
                                                   uint16_t samples,
                                                   uint16_t target_samples,
                                                   uint8_t  fail_reason,
                                                   uint32_t now_ms,
                                                   uint32_t terminal_since_ms)
{
    oled_calibration_view_t view = {0};

    switch (auto_cal_state)
    {
        case SYSTEM_IMU_CAL_WAIT:
        case SYSTEM_IMU_CAL_RETRY_WAIT:
            view.visible = 1U;
            view.title   = "IMU CAL WAIT";
            view.detail  = OLED_CalibrationView_FailReason(fail_reason);
            break;
        case SYSTEM_IMU_CAL_RUNNING:
            view.visible          = 1U;
            view.title            = "IMU CAL RUN";
            view.detail           = "SAMPLING";
            view.progress_percent = (target_samples != 0U && samples < target_samples)
                                        ? (uint8_t)(((uint32_t)samples * 100U) / target_samples)
                                        : 100U;
            break;
        case SYSTEM_IMU_CAL_DONE:
        case SYSTEM_IMU_CAL_FAILED:
            view.visible = 1U;
            view.title   = (auto_cal_state == SYSTEM_IMU_CAL_DONE) ? "IMU CAL DONE" : "IMU CAL FAILED";
            view.detail =
                (auto_cal_state == SYSTEM_IMU_CAL_DONE) ? "SAVED IN RAM" : OLED_CalibrationView_FailReason(fail_reason);
            if ((uint32_t)(now_ms - terminal_since_ms) >= OLED_CALIBRATION_RETURN_MS)
            {
                view.visible          = 0U;
                view.return_to_normal = 1U;
            }
            break;
        default:
            break;
    }
    return view;
}
