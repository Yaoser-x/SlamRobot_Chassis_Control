#include "line_calibration_orchestrator.h"
#include "line_following_service.h"
#include "line_following_maintenance.h"

#include "command_management_service.h"
#include "line_sensor_driver.h"
#include "motion_control_service.h"
#include "motion_control_maintenance.h"
#include "parameter_management_service.h"
#include "safety_management_service.h"

#include <stdio.h>
#include <stdlib.h>

static uint32_t                            fake_revoke_generation;
static uint32_t                            fake_tick;
static uint32_t                            submitted_count;
static uint32_t                            clear_count;
static uint8_t                             fake_estop;
static uint8_t                             fake_fault_stop;
static uint8_t                             fake_maintenance;
static motion_control_maintenance_result_t fake_maintenance_result;
static uint8_t                             fake_save_success;
static uint8_t                             fake_parameter_set_success;
static uint8_t                             save_ready_mask;
static uint32_t                            maintenance_begin_count;
static uint32_t                            maintenance_end_count;
static uint32_t                            flash_save_count;
static command_velocity_t                  last_command;
static line_sensor_data_t                  fake_sensor;
static param_model_t                       fake_params;
static uint32_t                            fake_parameter_generation;
static uint32_t                            threshold_apply_count;
static uint16_t                            applied_threshold[LINE_SENSOR_CHANNELS];
static uint8_t                             applied_active_low;

static void require_int(int condition, const char *message);

void LineSensorDriver_Update(void)
{
}

void LineSensorDriver_RequestAnalog(void)
{
}

uint32_t ParameterManagement_GetSnapshot(param_model_t *params)
{
    *params = fake_params;
    return fake_parameter_generation;
}
uint8_t ParameterManagement_Set(const param_model_t *params)
{
    if (fake_parameter_set_success == 0U)
    {
        return 0U;
    }
    fake_params = *params;
    fake_parameter_generation++;
    return 1U;
}

uint8_t ParameterManagement_Save(void)
{
    line_sensor_calibration_t calibration;

    LineFollowing_CalibrationGet(&calibration);
    save_ready_mask = calibration.ready_mask;
    flash_save_count++;
    return fake_save_success;
}

motion_control_maintenance_result_t MotionControl_BeginMaintenance(void)
{
    maintenance_begin_count++;
    if (fake_maintenance_result != MOTION_CONTROL_MAINTENANCE_OK)
    {
        return fake_maintenance_result;
    }
    fake_maintenance = 1U;
    return MOTION_CONTROL_MAINTENANCE_OK;
}

void MotionControl_EndMaintenance(void)
{
    maintenance_end_count++;
    fake_maintenance = 0U;
}

void LineSensorDriver_SetThresholdConfig(const uint16_t threshold[LINE_SENSOR_CHANNELS], uint8_t active_low)
{
    for (uint8_t index = 0U; index < LINE_SENSOR_CHANNELS; ++index)
    {
        applied_threshold[index] = threshold[index];
    }
    applied_active_low = active_low;
    threshold_apply_count++;
}

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

uint32_t osKernelGetTickCount(void)
{
    return fake_tick;
}

uint32_t CommandManagement_GetMotionRevokeGeneration(void)
{
    return fake_revoke_generation;
}

uint8_t SafetyManagement_IsMotionAllowed(void)
{
    return (fake_estop == 0U && fake_fault_stop == 0U && fake_maintenance == 0U) ? 1U : 0U;
}

command_result_t CommandManagement_SetForGeneration(const command_velocity_t *command, uint32_t expected_generation)
{
    if (expected_generation != fake_revoke_generation)
    {
        return COMMAND_RESULT_REJECTED;
    }
    last_command = *command;
    submitted_count++;
    return COMMAND_RESULT_ACCEPTED;
}

void CommandManagement_ClearSource(command_source_t source)
{
    if (source == COMMAND_SOURCE_LINE)
    {
        clear_count++;
    }
}

uint8_t LineSensorDriver_GetSensorData(line_sensor_data_t *data)
{
    *data = fake_sensor;
    return 1U;
}

static void reset_fake(void)
{
    fake_revoke_generation                  = 10U;
    fake_tick                               = 100U;
    submitted_count                         = 0U;
    clear_count                             = 0U;
    fake_estop                              = 0U;
    fake_fault_stop                         = 0U;
    fake_maintenance                        = 0U;
    fake_maintenance_result                 = MOTION_CONTROL_MAINTENANCE_OK;
    fake_save_success                       = 1U;
    fake_parameter_set_success              = 1U;
    save_ready_mask                         = 0U;
    maintenance_begin_count                 = 0U;
    maintenance_end_count                   = 0U;
    flash_save_count                        = 0U;
    last_command                            = (command_velocity_t){0};
    fake_sensor                             = (line_sensor_data_t){0};
    fake_sensor.valid                       = 1U;
    fake_sensor.timestamp_ms                = 100U;
    fake_sensor.state[3]                    = 1U;
    fake_sensor.state[4]                    = 1U;
    fake_params                             = (param_model_t){0};
    fake_params.line_kp                     = 0.5f;
    fake_params.line_kd                     = 0.1f;
    fake_params.line_speed_mps              = 0.2f;
    fake_params.line_slowdown_gain          = 0.5f;
    fake_params.line_detect_debounce_frames = 2U;
    fake_params.line_lost_debounce_frames   = 2U;
    fake_parameter_generation               = 1UL;
    threshold_apply_count                   = 0UL;
    applied_active_low                      = 0U;
    for (uint8_t index = 0U; index < LINE_SENSOR_CHANNELS; ++index)
    {
        fake_params.line_threshold_raw[index] = 500U;
        applied_threshold[index]              = 0U;
    }
    const line_following_config_t config = {
        .angular_max_rps        = 2.0f,
        .sensor_timeout_ms      = 50U,
        .default_enabled        = 0U,
        .detect_threshold_count = 1U,
    };
    require_int(LineFollowing_Init(&config) != 0U, "line config accepted");
    require_int(threshold_apply_count == 1U && applied_threshold[0] == 500U,
                "line init applies current parameter generation");
    LineFollowing_Enable(1U);
    LineCalibrationOrchestrator_Init(60000U);
}

static void collect_calibration_surface(line_sensor_calibration_surface_t surface, uint16_t base)
{
    require_int(LineFollowing_RequestCalibration(surface, 4U) != 0U, "line calibration request accepted");
    for (uint8_t sample = 0U; sample < 4U; ++sample)
    {
        for (uint8_t channel = 0U; channel < LINE_SENSOR_CHANNELS; ++channel)
        {
            fake_sensor.analog[channel] = (uint16_t)(base + channel + sample);
        }
        fake_sensor.timestamp_ms++;
        fake_tick++;
        LineFollowing_Update();
    }
}

static void
collect_owned_surface(app_line_calibration_mode_t mode, line_sensor_calibration_surface_t surface, uint16_t base)
{
    require_int(LineCalibrationOrchestrator_Request(mode, surface, 4U) == APP_LINE_CALIBRATION_RESULT_OK,
                "App calibration session accepts surface");
    for (uint8_t sample = 0U; sample < 4U; ++sample)
    {
        for (uint8_t channel = 0U; channel < LINE_SENSOR_CHANNELS; ++channel)
        {
            fake_sensor.analog[channel] = (uint16_t)(base + channel + sample);
        }
        fake_sensor.timestamp_ms++;
        fake_tick++;
        LineFollowing_Update();
        LineCalibrationOrchestrator_Update(fake_tick);
    }
}

static void test_app_owned_calibration_sessions(void)
{
    line_sensor_calibration_t        calibration;
    app_line_calibration_led_event_t led_event;

    reset_fake();
    collect_owned_surface(APP_LINE_CALIBRATION_MODE_AUTOMATIC, LINE_CALIBRATION_SURFACE_FLOOR, 900U);
    require_int(fake_maintenance != 0U && maintenance_end_count == 0U,
                "automatic session holds maintenance while waiting for second surface");
    require_int(LineCalibrationOrchestrator_Request(APP_LINE_CALIBRATION_MODE_MANUAL, LINE_CALIBRATION_SURFACE_LINE, 4U)
                    == APP_LINE_CALIBRATION_RESULT_BUSY,
                "another owner cannot steal an active session");
    collect_owned_surface(APP_LINE_CALIBRATION_MODE_AUTOMATIC, LINE_CALIBRATION_SURFACE_LINE, 200U);
    LineFollowing_CalibrationGet(&calibration);
    require_int(flash_save_count == 1U && save_ready_mask == 0x03U && maintenance_begin_count == 1U
                    && maintenance_end_count == 1U,
                "automatic session saves once and releases one owned lease");
    require_int(calibration.ready_mask == 0U && fake_params.line_threshold_raw[0] == 551U,
                "successful automatic commit clears raw data after persistence");
    require_int(LineCalibrationOrchestrator_TakeLedEvent(&led_event) != 0U
                    && led_event == APP_LINE_CALIBRATION_LED_APPLIED,
                "successful automatic commit emits one applied LED event");

    reset_fake();
    collect_owned_surface(APP_LINE_CALIBRATION_MODE_MANUAL, LINE_CALIBRATION_SURFACE_FLOOR, 900U);
    collect_owned_surface(APP_LINE_CALIBRATION_MODE_MANUAL, LINE_CALIBRATION_SURFACE_LINE, 200U);
    LineFollowing_CalibrationGet(&calibration);
    require_int(flash_save_count == 0U && fake_maintenance != 0U && calibration.ready_mask == 0x03U,
                "manual session remains inspectable and never auto-saves");
    require_int(LineCalibrationOrchestrator_ApplyManual() == APP_LINE_CALIBRATION_RESULT_OK,
                "manual apply succeeds through the App owner");
    LineFollowing_CalibrationGet(&calibration);
    require_int(fake_maintenance == 0U && flash_save_count == 0U && calibration.ready_mask == 0x03U,
                "manual apply releases maintenance but preserves data for show and explicit save");
}

static void test_app_owned_failure_and_timeout_preserve_data(void)
{
    line_sensor_calibration_t        calibration;
    app_line_calibration_led_event_t led_event;

    reset_fake();
    collect_owned_surface(APP_LINE_CALIBRATION_MODE_AUTOMATIC, LINE_CALIBRATION_SURFACE_FLOOR, 900U);
    fake_save_success = 0U;
    collect_owned_surface(APP_LINE_CALIBRATION_MODE_AUTOMATIC, LINE_CALIBRATION_SURFACE_LINE, 200U);
    LineFollowing_Update();
    LineFollowing_CalibrationGet(&calibration);
    require_int(fake_params.line_threshold_raw[0] == 500U && calibration.ready_mask == 0x03U,
                "save failure rolls RAM back and preserves raw calibration");
    require_int(fake_maintenance == 0U && maintenance_end_count == 1U, "save failure releases the full-session lease");
    require_int(LineCalibrationOrchestrator_TakeLedEvent(&led_event) != 0U
                    && led_event == APP_LINE_CALIBRATION_LED_FAILED,
                "save failure emits one explicit failed LED event");
    require_int(LineCalibrationOrchestrator_GetLastResult() == APP_LINE_CALIBRATION_RESULT_SAVE_FAILED,
                "save failure remains available as a diagnostic result");

    reset_fake();
    require_int(
        LineCalibrationOrchestrator_Request(APP_LINE_CALIBRATION_MODE_MANUAL, LINE_CALIBRATION_SURFACE_FLOOR, 4U)
            == APP_LINE_CALIBRATION_RESULT_OK,
        "manual session starts for timeout test");
    LineCalibrationOrchestrator_Update(fake_tick + 60000U);
    LineFollowing_CalibrationGet(&calibration);
    require_int(fake_maintenance == 0U && calibration.collecting == 0U && maintenance_end_count == 1U,
                "no-progress timeout aborts collection and releases maintenance");
    require_int(LineCalibrationOrchestrator_GetLastResult() == APP_LINE_CALIBRATION_RESULT_TIMEOUT,
                "timeout remains available as a diagnostic result");

    reset_fake();
    collect_owned_surface(APP_LINE_CALIBRATION_MODE_AUTOMATIC, LINE_CALIBRATION_SURFACE_FLOOR, 500U);
    collect_owned_surface(APP_LINE_CALIBRATION_MODE_AUTOMATIC, LINE_CALIBRATION_SURFACE_LINE, 500U);
    LineFollowing_CalibrationGet(&calibration);
    require_int(calibration.ready_mask == 0x03U && calibration.fail_mask != 0U && flash_save_count == 0U,
                "low separation is explicit and retains diagnostics without saving");
    require_int(fake_maintenance == 0U, "low separation releases maintenance");
    require_int(LineCalibrationOrchestrator_GetLastResult() == APP_LINE_CALIBRATION_RESULT_LOW_SEPARATION,
                "low separation remains available as a diagnostic result");

    reset_fake();
    collect_owned_surface(APP_LINE_CALIBRATION_MODE_AUTOMATIC, LINE_CALIBRATION_SURFACE_FLOOR, 900U);
    fake_parameter_set_success = 0U;
    collect_owned_surface(APP_LINE_CALIBRATION_MODE_AUTOMATIC, LINE_CALIBRATION_SURFACE_LINE, 200U);
    LineFollowing_CalibrationGet(&calibration);
    require_int(calibration.ready_mask == 0x03U && flash_save_count == 0U && fake_maintenance == 0U,
                "parameter rejection retains samples, skips Flash, and releases maintenance");
    require_int(LineCalibrationOrchestrator_GetLastResult() == APP_LINE_CALIBRATION_RESULT_PARAMETER_REJECTED,
                "parameter rejection remains available as a diagnostic result");

    reset_fake();
    require_int(
        LineCalibrationOrchestrator_Request(APP_LINE_CALIBRATION_MODE_MANUAL, LINE_CALIBRATION_SURFACE_FLOOR, 4U)
            == APP_LINE_CALIBRATION_RESULT_OK,
        "manual session starts for explicit cancel");
    LineCalibrationOrchestrator_Cancel();
    LineFollowing_CalibrationGet(&calibration);
    require_int(calibration.ready_mask == 0U && calibration.collecting == 0U && fake_maintenance == 0U,
                "explicit cancel clears data and releases maintenance");
    require_int(LineCalibrationOrchestrator_GetLastResult() == APP_LINE_CALIBRATION_RESULT_CANCELLED,
                "explicit cancel remains available as a diagnostic result");
}

static void test_calibration_apply_is_explicit(void)
{
    uint16_t old_threshold;

    reset_fake();
    old_threshold = fake_params.line_threshold_raw[0];
    collect_calibration_surface(LINE_CALIBRATION_SURFACE_FLOOR, 900U);
    require_int(LineFollowing_ApplyCalibration() == LINE_CALIBRATION_APPLY_INCOMPLETE,
                "single surface cannot overwrite RAM parameters");
    require_int(fake_params.line_threshold_raw[0] == old_threshold, "rejected apply preserves prior parameters");
    collect_calibration_surface(LINE_CALIBRATION_SURFACE_LINE, 200U);
    require_int(LineFollowing_ApplyCalibration() == LINE_CALIBRATION_APPLY_OK, "two separated surfaces apply to RAM");
    require_int(fake_params.line_threshold_raw[0] == 551U && fake_params.line_active_low != 0U,
                "RAM apply updates thresholds and polarity");
    require_int(applied_threshold[0] == 551U && applied_active_low != 0U && threshold_apply_count == 2U,
                "RAM apply immediately updates the line driver");
}

static void test_parameter_generation_applies_before_next_sensor_frame(void)
{
    reset_fake();
    fake_params.line_threshold_raw[0] = 777U;
    fake_params.line_active_low       = 0U;
    fake_parameter_generation++;

    LineFollowing_Update();
    require_int(threshold_apply_count == 2U, "changed generation applies exactly once");
    require_int(applied_threshold[0] == 777U && applied_active_low == 0U,
                "next update uses the new threshold and polarity");
    LineFollowing_Update();
    require_int(threshold_apply_count == 2U, "unchanged generation is not applied twice");
}

static void test_abort_preserves_collected_data(void)
{
    line_sensor_calibration_t calibration;

    reset_fake();
    require_int(LineFollowing_RequestCalibration(LINE_CALIBRATION_SURFACE_FLOOR, 4U) != 0U,
                "calibration request accepted");
    LineFollowing_Update();
    LineFollowing_CalibrationGet(&calibration);
    require_int(calibration.collecting != 0U && calibration.count[0] == 1U, "request starts and captures a sample");
    LineFollowing_CalibrationAbort();
    LineFollowing_CalibrationGet(&calibration);
    require_int(calibration.collecting == 0U && calibration.count[0] == 1U,
                "abort stops collection while preserving raw data");
}

static void test_safety_state_rejects_line_rearm(void)
{
    reset_fake();
    require_int(LineFollowing_Enable(0U) == LINE_FOLLOWING_RESULT_APPLIED, "line disable reports applied");

    fake_maintenance = 1U;
    require_int(LineFollowing_Enable(1U) == LINE_FOLLOWING_RESULT_REJECTED,
                "maintenance rejection is Service-generated");
    require_int(LineFollowing_IsEnabled() == 0U, "maintenance rejects line rearm");

    fake_maintenance = 0U;
    fake_estop       = 1U;
    require_int(LineFollowing_Enable(1U) == LINE_FOLLOWING_RESULT_REJECTED, "ESTOP rejection is Service-generated");
    require_int(LineFollowing_IsEnabled() == 0U, "estop rejects line rearm");

    fake_estop      = 0U;
    fake_fault_stop = 1U;
    require_int(LineFollowing_Enable(1U) == LINE_FOLLOWING_RESULT_REJECTED,
                "fault-stop rejection is Service-generated");
    require_int(LineFollowing_IsEnabled() == 0U, "fault stop rejects line rearm");
}

static void test_safety_generation_revokes_old_line_enable(void)
{
    reset_fake();
    LineFollowing_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineFollowing_Update();
    require_int(submitted_count == 1U && last_command.source == COMMAND_SOURCE_LINE,
                "enabled line submits before safety revocation");

    fake_revoke_generation++;
    LineFollowing_Update();
    require_int(LineFollowing_IsEnabled() == 0U, "old line enable is revoked");
    require_int(submitted_count == 1U, "revoked line cannot resubmit stale motion");
    require_int(clear_count != 0U, "revoked line source is cleared");

    LineFollowing_Enable(1U);
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineFollowing_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineFollowing_Update();
    require_int(submitted_count == 2U, "new explicit line enable can move again");
}

static void test_pd_slowdown_and_lost_debounce(void)
{
    line_following_status_t state;
    reset_fake();
    fake_sensor.state[3] = 0U;
    fake_sensor.state[4] = 0U;
    fake_sensor.state[6] = 1U;
    fake_sensor.state[7] = 1U;
    LineFollowing_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineFollowing_Update();
    (void)LineFollowing_GetStatus(&state);
    require_int(state.tracking_active != 0U && state.error < -2.0f,
                "offset line tracked (right side → negative error)");
    require_int(state.linear_x < fake_params.line_speed_mps, "large error reduces speed");

    for (uint8_t i = 0U; i < 8U; ++i)
    {
        fake_sensor.state[i] = 0U;
    }
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineFollowing_Update();
    require_int(clear_count == 0U, "single lost frame debounced");
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineFollowing_Update();
    require_int(clear_count == 1U, "confirmed lost line clears source");
}

static void test_debounce_counts_unique_sensor_frames(void)
{
    reset_fake();
    LineFollowing_Update();
    LineFollowing_Update();
    require_int(submitted_count == 0U, "repeated timestamp does not satisfy detect debounce");
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineFollowing_Update();
    require_int(submitted_count == 1U, "second unique frame satisfies detect debounce");

    for (uint8_t i = 0U; i < 8U; ++i)
    {
        fake_sensor.state[i] = 0U;
    }
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineFollowing_Update();
    LineFollowing_Update();
    require_int(clear_count == 0U, "repeated lost timestamp does not satisfy debounce");
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineFollowing_Update();
    require_int(clear_count == 1U, "second unique lost frame clears source");
}

static void test_stale_sensor_clears_tracking_view_state(void)
{
    line_following_status_t state;
    reset_fake();
    LineFollowing_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineFollowing_Update();
    (void)LineFollowing_GetStatus(&state);
    require_int(state.tracking_active != 0U, "tracking active before stale sample");
    fake_sensor.valid = 0U;
    LineFollowing_Update();
    (void)LineFollowing_GetStatus(&state);
    require_int(state.tracking_active == 0U && state.lost_reason == 1U,
                "stale sample clears tracking state and reports stale reason");
}

static void test_status_generation_is_monotonic_on_whole_publish(void)
{
    line_following_status_t before;
    line_following_status_t after;

    reset_fake();
    (void)LineFollowing_GetStatus(&before);
    LineFollowing_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineFollowing_Update();
    (void)LineFollowing_GetStatus(&after);
    require_int(after.generation > before.generation, "line status generation advances on published frame");
    require_int(after.tracking_active != 0U && after.detected_count == 2U,
                "line status publishes one complete tracking frame");
}

int main(void)
{
    test_safety_generation_revokes_old_line_enable();
    test_safety_state_rejects_line_rearm();
    test_pd_slowdown_and_lost_debounce();
    test_debounce_counts_unique_sensor_frames();
    test_stale_sensor_clears_tracking_view_state();
    test_status_generation_is_monotonic_on_whole_publish();
    test_calibration_apply_is_explicit();
    test_parameter_generation_applies_before_next_sensor_frame();
    test_abort_preserves_collected_data();
    test_app_owned_calibration_sessions();
    test_app_owned_failure_and_timeout_preserve_data();
    (void)printf("PASS: line control safety generation tests\n");
    return 0;
}
