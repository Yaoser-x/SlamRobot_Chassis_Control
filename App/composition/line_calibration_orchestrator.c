#include "line_calibration_orchestrator.h"

#include "line_following_maintenance.h"
#include "motion_control_maintenance.h"
#include "parameter_management_service.h"
#include "platform_critical.h"
#include "platform_time.h"

#define APP_LINE_CALIBRATION_DEFAULT_TIMEOUT_MS 60000U

typedef struct
{
    app_line_calibration_mode_t      mode;
    uint32_t                         no_progress_timeout_ms;
    uint32_t                         last_progress_ms;
    uint16_t                         last_count[2];
    uint8_t                          last_ready_mask;
    uint8_t                          maintenance_owned;
    uint8_t                          operation_busy;
    app_line_calibration_led_event_t led_event;
    app_line_calibration_result_t    last_result;
} app_line_calibration_session_t;

static app_line_calibration_session_t session;

static void LineCalibrationOrchestrator_RecordResult(app_line_calibration_result_t result)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    session.last_result = result;
    PlatformCritical_Exit(critical);
}

static void LineCalibrationOrchestrator_Finish(app_line_calibration_led_event_t event,
                                               app_line_calibration_result_t    result)
{
    uint8_t                   release_maintenance;
    platform_critical_state_t critical = PlatformCritical_Enter();

    release_maintenance       = session.maintenance_owned;
    session.mode              = 0;
    session.maintenance_owned = 0U;
    session.operation_busy    = 0U;
    session.last_progress_ms  = 0U;
    session.last_count[0]     = 0U;
    session.last_count[1]     = 0U;
    session.last_ready_mask   = 0U;
    session.led_event         = event;
    session.last_result       = result;
    PlatformCritical_Exit(critical);
    if (release_maintenance != 0U)
    {
        MotionControl_EndMaintenance();
    }
}

static app_line_calibration_result_t LineCalibrationOrchestrator_MapApplyResult(line_calibration_apply_result_t result)
{
    switch (result)
    {
        case LINE_CALIBRATION_APPLY_OK:
            return APP_LINE_CALIBRATION_RESULT_OK;
        case LINE_CALIBRATION_APPLY_INCOMPLETE:
            return APP_LINE_CALIBRATION_RESULT_INCOMPLETE;
        case LINE_CALIBRATION_APPLY_LOW_SEPARATION:
            return APP_LINE_CALIBRATION_RESULT_LOW_SEPARATION;
        case LINE_CALIBRATION_APPLY_PARAMETER_REJECTED:
        default:
            return APP_LINE_CALIBRATION_RESULT_PARAMETER_REJECTED;
    }
}

void LineCalibrationOrchestrator_Init(uint32_t no_progress_timeout_ms)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    session = (app_line_calibration_session_t){
        .no_progress_timeout_ms =
            (no_progress_timeout_ms != 0U) ? no_progress_timeout_ms : APP_LINE_CALIBRATION_DEFAULT_TIMEOUT_MS,
    };
    PlatformCritical_Exit(critical);
}

app_line_calibration_result_t LineCalibrationOrchestrator_Request(app_line_calibration_mode_t       mode,
                                                                  line_sensor_calibration_surface_t surface,
                                                                  uint16_t                          samples)
{
    line_sensor_calibration_t calibration;
    uint8_t                   new_session;
    uint32_t                  now_ms;
    platform_critical_state_t critical;

    if ((mode != APP_LINE_CALIBRATION_MODE_MANUAL && mode != APP_LINE_CALIBRATION_MODE_AUTOMATIC)
        || (uint8_t)surface > (uint8_t)LINE_CALIBRATION_SURFACE_LINE || samples < 4U || samples > 2000U)
    {
        LineCalibrationOrchestrator_RecordResult(APP_LINE_CALIBRATION_RESULT_REQUEST_REJECTED);
        return APP_LINE_CALIBRATION_RESULT_REQUEST_REJECTED;
    }
    LineFollowing_CalibrationGet(&calibration);
    critical = PlatformCritical_Enter();
    if (session.operation_busy != 0U || (session.mode != 0 && session.mode != mode) || calibration.collecting != 0U)
    {
        PlatformCritical_Exit(critical);
        LineCalibrationOrchestrator_RecordResult(APP_LINE_CALIBRATION_RESULT_BUSY);
        return APP_LINE_CALIBRATION_RESULT_BUSY;
    }
    new_session            = (session.mode == 0) ? 1U : 0U;
    session.operation_busy = 1U;
    PlatformCritical_Exit(critical);

    if (new_session != 0U)
    {
        if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
        {
            critical               = PlatformCritical_Enter();
            session.operation_busy = 0U;
            PlatformCritical_Exit(critical);
            LineCalibrationOrchestrator_RecordResult(APP_LINE_CALIBRATION_RESULT_MAINTENANCE_DENIED);
            return APP_LINE_CALIBRATION_RESULT_MAINTENANCE_DENIED;
        }
        critical                  = PlatformCritical_Enter();
        session.mode              = mode;
        session.maintenance_owned = 1U;
        PlatformCritical_Exit(critical);
    }
    if (LineFollowing_RequestCalibration(surface, samples) == 0U)
    {
        if (new_session != 0U)
        {
            LineCalibrationOrchestrator_Finish(APP_LINE_CALIBRATION_LED_FAILED,
                                               APP_LINE_CALIBRATION_RESULT_REQUEST_REJECTED);
        }
        else
        {
            critical               = PlatformCritical_Enter();
            session.operation_busy = 0U;
            PlatformCritical_Exit(critical);
        }
        LineCalibrationOrchestrator_RecordResult(APP_LINE_CALIBRATION_RESULT_REQUEST_REJECTED);
        return APP_LINE_CALIBRATION_RESULT_REQUEST_REJECTED;
    }

    now_ms                   = PlatformTime_TaskNowMs();
    critical                 = PlatformCritical_Enter();
    session.last_progress_ms = now_ms;
    session.last_count[0]    = calibration.count[0];
    session.last_count[1]    = calibration.count[1];
    session.last_ready_mask  = calibration.ready_mask;
    session.operation_busy   = 0U;
    session.led_event        = APP_LINE_CALIBRATION_LED_RUNNING;
    session.last_result      = APP_LINE_CALIBRATION_RESULT_OK;
    PlatformCritical_Exit(critical);
    return APP_LINE_CALIBRATION_RESULT_OK;
}

app_line_calibration_result_t LineCalibrationOrchestrator_ApplyManual(void)
{
    line_calibration_apply_result_t apply_result;
    app_line_calibration_result_t   result;
    uint8_t                         acquired_here = 0U;
    platform_critical_state_t       critical;

    critical = PlatformCritical_Enter();
    if (session.operation_busy != 0U || session.mode == APP_LINE_CALIBRATION_MODE_AUTOMATIC)
    {
        PlatformCritical_Exit(critical);
        return APP_LINE_CALIBRATION_RESULT_BUSY;
    }
    session.operation_busy = 1U;
    if (session.mode == 0)
    {
        acquired_here = 1U;
    }
    PlatformCritical_Exit(critical);

    if (acquired_here != 0U)
    {
        if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
        {
            critical               = PlatformCritical_Enter();
            session.operation_busy = 0U;
            PlatformCritical_Exit(critical);
            return APP_LINE_CALIBRATION_RESULT_MAINTENANCE_DENIED;
        }
        critical                  = PlatformCritical_Enter();
        session.mode              = APP_LINE_CALIBRATION_MODE_MANUAL;
        session.maintenance_owned = 1U;
        PlatformCritical_Exit(critical);
    }

    apply_result = LineFollowing_ApplyCalibration();
    result       = LineCalibrationOrchestrator_MapApplyResult(apply_result);
    if (result != APP_LINE_CALIBRATION_RESULT_OK)
    {
        LineFollowing_CalibrationAbort();
    }
    LineCalibrationOrchestrator_Finish((result == APP_LINE_CALIBRATION_RESULT_OK) ? APP_LINE_CALIBRATION_LED_APPLIED
                                                                                  : APP_LINE_CALIBRATION_LED_FAILED,
                                       result);
    return result;
}

void LineCalibrationOrchestrator_Cancel(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    if (session.operation_busy != 0U)
    {
        PlatformCritical_Exit(critical);
        return;
    }
    session.operation_busy = 1U;
    PlatformCritical_Exit(critical);
    LineFollowing_CalibrationCancel();
    LineCalibrationOrchestrator_Finish(APP_LINE_CALIBRATION_LED_FAILED, APP_LINE_CALIBRATION_RESULT_CANCELLED);
}

void LineCalibrationOrchestrator_Update(uint32_t now_ms)
{
    line_sensor_calibration_t   calibration;
    app_line_calibration_mode_t mode;
    uint32_t                    timeout_ms;
    uint32_t                    last_progress_ms;
    uint8_t                     busy;
    platform_critical_state_t   critical;

    critical         = PlatformCritical_Enter();
    mode             = session.mode;
    timeout_ms       = session.no_progress_timeout_ms;
    last_progress_ms = session.last_progress_ms;
    busy             = session.operation_busy;
    PlatformCritical_Exit(critical);
    if (mode == 0 || busy != 0U)
    {
        return;
    }

    LineFollowing_CalibrationGet(&calibration);
    critical = PlatformCritical_Enter();
    if (session.mode != mode || session.operation_busy != 0U)
    {
        PlatformCritical_Exit(critical);
        return;
    }
    if (calibration.count[0] != session.last_count[0] || calibration.count[1] != session.last_count[1]
        || calibration.ready_mask != session.last_ready_mask)
    {
        session.last_count[0]    = calibration.count[0];
        session.last_count[1]    = calibration.count[1];
        session.last_ready_mask  = calibration.ready_mask;
        session.last_progress_ms = now_ms;
        last_progress_ms         = now_ms;
        if (calibration.collecting == 0U && calibration.ready_mask != 0U)
        {
            session.led_event = APP_LINE_CALIBRATION_LED_SURFACE_READY;
        }
    }
    PlatformCritical_Exit(critical);

    if (!((timeout_ms != 0U && (uint32_t)(now_ms - last_progress_ms) >= timeout_ms)
          || (mode == APP_LINE_CALIBRATION_MODE_AUTOMATIC && calibration.collecting == 0U
              && calibration.ready_mask == 0x03U)))
    {
        return;
    }
    critical = PlatformCritical_Enter();
    if (session.mode != mode || session.operation_busy != 0U)
    {
        PlatformCritical_Exit(critical);
        return;
    }
    session.operation_busy = 1U;
    PlatformCritical_Exit(critical);

    if (timeout_ms != 0U && (uint32_t)(now_ms - last_progress_ms) >= timeout_ms)
    {
        LineFollowing_CalibrationAbort();
        LineCalibrationOrchestrator_Finish(APP_LINE_CALIBRATION_LED_FAILED, APP_LINE_CALIBRATION_RESULT_TIMEOUT);
        return;
    }
    if (mode == APP_LINE_CALIBRATION_MODE_AUTOMATIC && calibration.collecting == 0U && calibration.ready_mask == 0x03U)
    {
        param_model_t                   old_params;
        line_calibration_apply_result_t apply_result;

        (void)ParameterManagement_GetSnapshot(&old_params);
        apply_result = LineFollowing_ApplyCalibration();
        if (apply_result != LINE_CALIBRATION_APPLY_OK)
        {
            LineCalibrationOrchestrator_Finish(APP_LINE_CALIBRATION_LED_FAILED,
                                               LineCalibrationOrchestrator_MapApplyResult(apply_result));
            return;
        }
        if (ParameterManagement_Save() == 0U)
        {
            (void)ParameterManagement_Set(&old_params);
            LineCalibrationOrchestrator_Finish(APP_LINE_CALIBRATION_LED_FAILED,
                                               APP_LINE_CALIBRATION_RESULT_SAVE_FAILED);
            return;
        }
        LineFollowing_CalibrationCancel();
        LineCalibrationOrchestrator_Finish(APP_LINE_CALIBRATION_LED_APPLIED, APP_LINE_CALIBRATION_RESULT_OK);
    }
}

uint8_t LineCalibrationOrchestrator_TakeLedEvent(app_line_calibration_led_event_t *event)
{
    platform_critical_state_t critical;

    if (event == 0)
    {
        return 0U;
    }
    critical          = PlatformCritical_Enter();
    *event            = session.led_event;
    session.led_event = APP_LINE_CALIBRATION_LED_NONE;
    PlatformCritical_Exit(critical);
    return (*event != APP_LINE_CALIBRATION_LED_NONE) ? 1U : 0U;
}

app_line_calibration_result_t LineCalibrationOrchestrator_GetLastResult(void)
{
    app_line_calibration_result_t result;
    platform_critical_state_t     critical = PlatformCritical_Enter();

    result = session.last_result;
    PlatformCritical_Exit(critical);
    return result;
}
