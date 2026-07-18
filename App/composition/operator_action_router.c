#include "operator_action_router.h"

#include "line_following_service.h"
#include "line_following_maintenance.h"
#include "command_management_service.h"

#include "status_led_adapter.h"

/* --- calibration completion tracking --- */
static uint8_t prev_ready_mask;

void OperatorActionRouter_Handle(const teleoperation_action_t *action)
{
    line_sensor_calibration_t cal_state;

    /* --- poll calibration completion (every PS2 cycle, regardless of action type) --- */
    LineFollowing_CalibrationGet(&cal_state);

    if (cal_state.collecting == 0U && cal_state.ready_mask != prev_ready_mask)
    {
        if ((cal_state.ready_mask & 0x03U) == 0x03U)
        {
            /* both surfaces complete → auto apply */
            if (LineFollowing_ApplyCalibration() != 0U)
            {
                StatusLedAdapter_SetMode(STATUS_LED_ADAPTER_CAL_APPLIED);
            }
            else
            {
                /* low separation → blink fast */
                StatusLedAdapter_SetMode(STATUS_LED_ADAPTER_CAL_RUNNING);
            }
            LineFollowing_CalibrationCancel();
            prev_ready_mask = 0U;
        }
        else
        {
            /* single surface done */
            StatusLedAdapter_SetMode(STATUS_LED_ADAPTER_CAL_OK);
            prev_ready_mask = cal_state.ready_mask;
        }
    }

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
            /* guard: don't restart if already collecting */
            if (cal_state.collecting == 0U)
            {
                (void)LineFollowing_RequestCalibration(LINE_CALIBRATION_SURFACE_FLOOR, 100U);
                StatusLedAdapter_SetMode(STATUS_LED_ADAPTER_CAL_RUNNING);
            }
            break;

        case TELEOPERATION_ACTION_CALIBRATE_LINE_SURFACE:
            if (cal_state.collecting == 0U)
            {
                (void)LineFollowing_RequestCalibration(LINE_CALIBRATION_SURFACE_LINE, 100U);
                StatusLedAdapter_SetMode(STATUS_LED_ADAPTER_CAL_RUNNING);
            }
            break;

        default:
            break;
    }
}
