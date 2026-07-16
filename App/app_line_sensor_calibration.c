#include "app_line_sensor_calibration.h"

#include "line_following_service.h"
#include "motion_control_service.h"
#include "parameter_management_service.h"

uint8_t AppLineSensorCalibration_Begin(line_sensor_calibration_surface_t surface, uint16_t samples)
{
    uint8_t started;

    if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
    {
        return 0U;
    }
    started = LineFollowing_CalibrationStart(surface, samples);
    MotionControl_EndMaintenance();
    return started;
}

void AppLineSensorCalibration_ProcessRequest(void)
{
    line_following_calibration_request_t request;

    if (LineFollowing_TakeCalibrationRequest(&request) != 0U)
    {
        if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
        {
            (void)LineFollowing_ResolveCalibrationRequest(0U);
            return;
        }
        (void)LineFollowing_ResolveCalibrationRequest(1U);
        MotionControl_EndMaintenance();
    }
}

uint8_t AppLineSensorCalibration_CommitToFlash(void)
{
    uint8_t saved;

    if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
    {
        return 0U;
    }
    saved = ParameterManagement_Save();
    MotionControl_EndMaintenance();
    return saved;
}
