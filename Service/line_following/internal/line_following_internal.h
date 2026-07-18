#ifndef LINE_FOLLOWING_INTERNAL_H
#define LINE_FOLLOWING_INTERNAL_H

#include <stdint.h>

#include "line_following_calibration_types.h"
#include "line_following_maintenance.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** Internal: take a pending calibration request from the gateway. */
    uint8_t LineFollowingInternal_TakeCalibrationRequest(line_following_calibration_request_t *request);
    /** Internal: resolve a pending calibration request. */
    uint8_t LineFollowingInternal_ResolveCalibrationRequest(void);
    /** Internal: start calibration for the given surface and sample count. */
    uint8_t LineFollowingInternal_CalibrationStart(line_sensor_calibration_surface_t surface, uint16_t samples);
    /** Internal: build calibration thresholds from collected data. */
    uint8_t LineFollowingInternal_CalibrationBuild(uint16_t thresholds[LINE_CALIBRATION_CHANNELS], uint8_t *active_low);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOWING_INTERNAL_H */
