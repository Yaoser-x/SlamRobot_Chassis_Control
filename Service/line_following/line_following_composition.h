#ifndef LINE_FOLLOWING_COMPOSITION_H
#define LINE_FOLLOWING_COMPOSITION_H

#include <stdint.h>

#include "line_following_calibration_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** Configure the calibration ports needed during App initialization. */
    void LineFollowing_ConfigureCalibrationPorts(const line_following_calibration_ports_t *ports);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOWING_COMPOSITION_H */