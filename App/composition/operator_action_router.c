#include "operator_action_router.h"

#include "line_calibration_orchestrator.h"
#include "line_following_service.h"
#include "control_mode_coordinator.h"

void OperatorActionRouter_Handle(const teleoperation_action_t *action)
{
    if (action == 0 || action->type == TELEOPERATION_ACTION_NONE)
    {
        return;
    }

    switch (action->type)
    {
        case TELEOPERATION_ACTION_TOGGLE_LINE:
            if (LineFollowing_IsEnabled() == 0U)
            {
                if (ControlModeCoordinator_Request(CONTROL_MODE_LINE) != 0U)
                {
                    (void)LineFollowing_Enable(0U);
                    (void)LineFollowing_Enable(1U);
                }
            }
            else
            {
                (void)LineFollowing_Enable(0U);
                (void)ControlModeCoordinator_Request(CONTROL_MODE_AUTO);
            }
            break;

        case TELEOPERATION_ACTION_CALIBRATE_LINE_FLOOR:
            (void)LineCalibrationOrchestrator_Request(APP_LINE_CALIBRATION_MODE_AUTOMATIC,
                                                      LINE_CALIBRATION_SURFACE_FLOOR,
                                                      100U);
            break;

        case TELEOPERATION_ACTION_CALIBRATE_LINE_SURFACE:
            (void)LineCalibrationOrchestrator_Request(APP_LINE_CALIBRATION_MODE_AUTOMATIC,
                                                      LINE_CALIBRATION_SURFACE_LINE,
                                                      100U);
            break;

        default:
            break;
    }
}
