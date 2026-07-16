#include "line_calibration_coordinator.h"

#include "line_following_service.h"

static line_following_calibration_ports_t calibration_ports;

void LineCalibrationCoordinator_SetPorts(const line_following_calibration_ports_t *ports)
{
    calibration_ports = (ports != 0) ? *ports : (line_following_calibration_ports_t){0};
}

uint8_t LineCalibrationCoordinator_Begin(line_sensor_calibration_surface_t surface, uint16_t samples)
{
    uint8_t started;

    if (calibration_ports.begin_maintenance == 0 || calibration_ports.end_maintenance == 0
        || calibration_ports.begin_maintenance() == 0U)
    {
        return 0U;
    }
    started = LineFollowing_CalibrationStart(surface, samples);
    calibration_ports.end_maintenance();
    return started;
}

void LineCalibrationCoordinator_ProcessRequest(void)
{
    line_following_calibration_request_t request;

    if (LineFollowing_TakeCalibrationRequest(&request) != 0U)
    {
        if (calibration_ports.begin_maintenance == 0 || calibration_ports.end_maintenance == 0
            || calibration_ports.begin_maintenance() == 0U)
        {
            (void)LineFollowing_ResolveCalibrationRequest(0U);
            return;
        }
        (void)LineFollowing_ResolveCalibrationRequest(1U);
        calibration_ports.end_maintenance();
    }
}

uint8_t LineCalibrationCoordinator_CommitToFlash(void)
{
    uint8_t saved;

    if (calibration_ports.begin_maintenance == 0 || calibration_ports.end_maintenance == 0
        || calibration_ports.save_parameters == 0 || calibration_ports.begin_maintenance() == 0U)
    {
        return 0U;
    }
    saved = calibration_ports.save_parameters();
    calibration_ports.end_maintenance();
    return saved;
}
