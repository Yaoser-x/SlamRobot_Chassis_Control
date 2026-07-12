#ifndef OLED_CALIBRATION_VIEW_H
#define OLED_CALIBRATION_VIEW_H

#include <stdint.h>

typedef struct
{
    uint8_t     visible;
    uint8_t     return_to_normal;
    const char *title;
    const char *detail;
    uint8_t     progress_percent;
} oled_calibration_view_t;

/** Map the IMU calibration state to a non-blocking display model. */
oled_calibration_view_t OLED_CalibrationView_Build(uint8_t  auto_cal_state,
                                                   uint16_t samples,
                                                   uint16_t target_samples,
                                                   uint8_t  fail_reason,
                                                   uint32_t now_ms,
                                                   uint32_t terminal_since_ms);

#endif
