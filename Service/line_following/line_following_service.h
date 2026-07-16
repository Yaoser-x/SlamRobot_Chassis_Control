#ifndef LINE_FOLLOWING_SERVICE_H
#define LINE_FOLLOWING_SERVICE_H

#include <stdint.h>

#include "line_sensor_calibration.h"
#include "line_following_config.h"
#include "line_following_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        line_sensor_calibration_surface_t surface;
        uint16_t                          samples;
    } line_following_calibration_request_t;

    uint8_t  LineFollowing_Init(const line_following_config_t *config);
    void     LineFollowing_Update(void);
    void     LineFollowing_Enable(uint8_t enabled);
    uint8_t  LineFollowing_IsEnabled(void);
    uint32_t LineFollowing_GetStatus(line_following_status_t *status);
    uint8_t  LineFollowing_RequestCalibration(line_sensor_calibration_surface_t surface, uint16_t samples);
    uint8_t  LineFollowing_TakeCalibrationRequest(line_following_calibration_request_t *request);
    uint8_t  LineFollowing_ResolveCalibrationRequest(uint8_t authorized);
    uint8_t  LineFollowing_CalibrationStart(line_sensor_calibration_surface_t surface, uint16_t samples);
    uint8_t  LineFollowing_CalibrationBuild(uint16_t thresholds[LINE_CALIBRATION_CHANNELS], uint8_t *active_low);
    uint8_t  LineFollowing_CalibrationApplyToRam(void);
    void     LineFollowing_CalibrationGet(line_sensor_calibration_t *calibration);
    void     LineFollowing_CalibrationCancel(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOWING_SERVICE_H */
