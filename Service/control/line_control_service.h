#ifndef LINE_CONTROL_SERVICE_H
#define LINE_CONTROL_SERVICE_H

#include <stdint.h>
#include "line_calibration.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float    line_position;
        float    error;
        float    error_derivative;
        uint8_t  detected_count;
        uint8_t  sensor_state[8];
        uint16_t sensor_raw[8];
        uint16_t threshold_raw[8];
        float    linear_x;
        float    angular_z;
        uint8_t  tracking_active;
        uint8_t  globally_enabled;
        uint8_t  active_low;
        uint8_t  output_saturated;
        uint8_t  lost_reason;
    } line_control_service_state_t;

    void    LineControlService_Init(void);
    void    LineControlService_Update(void);
    void    LineControlService_Enable(uint8_t enable);
    uint8_t LineControlService_IsEnabled(void);
    void    LineControlService_GetState(line_control_service_state_t *state);
    uint8_t LineControlService_CalibrationBegin(line_calibration_surface_t surface, uint16_t samples);
    uint8_t LineControlService_CalibrationBuild(uint16_t thresholds[LINE_CALIBRATION_CHANNELS], uint8_t *active_low);
    uint8_t LineControlService_CalibrationApplyToRam(void);
    uint8_t LineControlService_CalibrationCommitToFlash(void);
    void    LineControlService_CalibrationGet(line_calibration_t *calibration);
    void    LineControlService_CalibrationCancel(void);

#ifdef __cplusplus
}
#endif

#endif
