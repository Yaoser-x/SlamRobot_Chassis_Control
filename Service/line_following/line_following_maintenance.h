#ifndef LINE_FOLLOWING_MAINTENANCE_H
#define LINE_FOLLOWING_MAINTENANCE_H

#include <stdint.h>

#include "line_following_calibration_types.h"
#include "line_following_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        line_sensor_calibration_surface_t surface;
        uint16_t                          samples;
    } line_following_calibration_request_t;

    /** Request a calibration operation through the maintenance gateway. */
    uint8_t LineFollowing_RequestCalibration(line_sensor_calibration_surface_t surface, uint16_t samples);
    /** Read the current calibration snapshot. */
    void    LineFollowing_CalibrationGet(line_sensor_calibration_t *calibration);
    /** Apply the completed calibration to the runtime parameter store. */
    uint8_t LineFollowing_ApplyCalibration(void);
    /** Cancel any in-progress calibration and release resources. */
    void    LineFollowing_CalibrationCancel(void);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOWING_MAINTENANCE_H */