#include "operator_action_router.h"

#include "line_calibration_orchestrator.h"
#include "line_following_service.h"
#include "command_management_service.h"

void OperatorActionRouter_Handle(const teleoperation_action_t *action)
{
    if (action == 0 || action->type == TELEOPERATION_ACTION_NONE)
    {
        return;
    }

    switch (action->type)
    {
        case TELEOPERATION_ACTION_TOGGLE_LINE:
            LineFollowing_Enable((LineFollowing_IsEnabled() == 0U) ? 1U : 0U);
            if (LineFollowing_IsEnabled() != 0U)
            {
                /* Yield PS2 command slot so lower-priority LINE can take over immediately. */
                CommandManagement_ClearSource(COMMAND_SOURCE_PS2);
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
