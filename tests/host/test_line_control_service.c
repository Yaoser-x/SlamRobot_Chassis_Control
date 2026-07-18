#include "line_calibration_coordinator.h"
#include "line_following_service.h"
#include "line_following_maintenance.h"
#include "line_following_composition.h"

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
static command_velocity_t                  last_command;
static line_sensor_data_t                  fake_sensor;
static param_model_t                       fake_params;
static motion_control_maintenance_result_t fake_maintenance_result;
static uint8_t                             fake_save_success;
static uint32_t                            maintenance_end_count;
static uint32_t                            flash_save_count;
static uint32_t                            fake_parameter_generation;
static uint32_t                            threshold_apply_count;
static uint16_t                            applied_threshold[LINE_SENSOR_CHANNELS];
static uint8_t                             applied_active_low;

static uint8_t fake_begin_maintenance_port(void)
{
    return (MotionControl_BeginMaintenance() == MOTION_CONTROL_MAINTENANCE_OK) ? 1U : 0U;
}

static const line_following_calibration_ports_t fake_calibration_ports = {
    .begin_maintenance = fake_begin_maintenance_port,
    .end_maintenance   = MotionControl_EndMaintenance,
    .save_parameters   = ParameterManagement_Save,
};

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
    fake_params = *params;
    fake_parameter_generation++;
    return 1U;
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

uint8_t ParameterManagement_Save(void)
{
    flash_save_count++;
    return fake_save_success;
}

motion_control_maintenance_result_t MotionControl_BeginMaintenance(void)
{
    if (fake_estop != 0U || fake_fault_stop != 0U || fake_maintenance != 0U)
    {
        return MOTION_CONTROL_MAINTENANCE_BUSY;
    }
    return fake_maintenance_result;
}
void MotionControl_EndMaintenance(void)
{
    maintenance_end_count++;
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
    fake_maintenance_result                 = MOTION_CONTROL_MAINTENANCE_OK;
    fake_save_success                       = 1U;
    maintenance_end_count                   = 0U;
    flash_save_count                        = 0U;
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
    LineFollowing_ConfigureCalibrationPorts(&fake_calibration_ports);
    LineFollowing_Enable(1U);
}

static void collect_calibration_surface(line_sensor_calibration_surface_t surface, uint16_t base)
{
    require_int(LineCalibrationCoordinator_Begin(surface, 4U) != 0U, "line calibration collection starts");
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

static void test_calibration_apply_and_commit_are_explicit(void)
{
    uint16_t old_threshold;

    reset_fake();
    old_threshold = fake_params.line_threshold_raw[0];
    collect_calibration_surface(LINE_CALIBRATION_SURFACE_FLOOR, 900U);
    require_int(LineFollowing_ApplyCalibration() == 0U, "single surface cannot overwrite RAM parameters");
    require_int(fake_params.line_threshold_raw[0] == old_threshold, "rejected apply preserves prior parameters");
    collect_calibration_surface(LINE_CALIBRATION_SURFACE_LINE, 200U);
    require_int(LineFollowing_ApplyCalibration() != 0U, "two separated surfaces apply to RAM");
    require_int(fake_params.line_threshold_raw[0] == 551U && fake_params.line_active_low != 0U,
                "RAM apply updates thresholds and polarity");
    require_int(applied_threshold[0] == 551U && applied_active_low != 0U && threshold_apply_count == 2U,
                "RAM apply immediately updates the line driver");
    require_int(flash_save_count == 0U, "RAM apply does not write flash");
    maintenance_end_count = 0U;

    require_int(LineCalibrationCoordinator_CommitToFlash() != 0U, "explicit flash commit succeeds");
    require_int(flash_save_count == 1U && maintenance_end_count == 1U,
                "flash commit saves once and releases maintenance");

    fake_maintenance_result = MOTION_CONTROL_MAINTENANCE_NOT_STATIONARY;
    require_int(LineCalibrationCoordinator_CommitToFlash() == 0U, "moving chassis rejects flash commit");
    require_int(flash_save_count == 1U && maintenance_end_count == 1U,
                "rejected commit neither saves nor releases an unowned lock");

    fake_maintenance_result = MOTION_CONTROL_MAINTENANCE_OK;
    fake_save_success       = 0U;
    require_int(LineCalibrationCoordinator_CommitToFlash() == 0U, "flash write failure is reported");
    require_int(flash_save_count == 2U && maintenance_end_count == 2U, "failed flash write still releases maintenance");
    require_int(applied_threshold[0] == 551U && applied_active_low != 0U,
                "failed flash write does not roll back RAM line parameters");
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

static void test_calibration_begin_requires_stationary_safe_chassis(void)
{
    line_sensor_calibration_t calibration;

    reset_fake();
    fake_maintenance_result = MOTION_CONTROL_MAINTENANCE_NOT_STATIONARY;
    require_int(LineCalibrationCoordinator_Begin(LINE_CALIBRATION_SURFACE_FLOOR, 4U) == 0U,
                "moving chassis rejects calibration collection");
    LineFollowing_CalibrationGet(&calibration);
    require_int(calibration.collecting == 0U, "rejected moving collection leaves calibration idle");

    fake_maintenance_result = MOTION_CONTROL_MAINTENANCE_OK;
    fake_estop              = 1U;
    require_int(LineCalibrationCoordinator_Begin(LINE_CALIBRATION_SURFACE_FLOOR, 4U) == 0U,
                "estop rejects calibration collection");
    fake_estop      = 0U;
    fake_fault_stop = 1U;
    require_int(LineCalibrationCoordinator_Begin(LINE_CALIBRATION_SURFACE_FLOOR, 4U) == 0U,
                "fault stop rejects calibration collection");
    fake_fault_stop  = 0U;
    fake_maintenance = 1U;
    require_int(LineCalibrationCoordinator_Begin(LINE_CALIBRATION_SURFACE_FLOOR, 4U) == 0U,
                "existing maintenance lock rejects calibration collection");
}

static void test_service_request_is_resolved_by_app_maintenance_gate(void)
{
    line_sensor_calibration_t calibration;

    reset_fake();
    fake_maintenance_result = MOTION_CONTROL_MAINTENANCE_NOT_STATIONARY;
    require_int(LineFollowing_RequestCalibration(LINE_CALIBRATION_SURFACE_FLOOR, 4U) != 0U,
                "service calibration request accepted");
    LineCalibrationCoordinator_ProcessRequest();
    LineFollowing_CalibrationGet(&calibration);
    require_int(calibration.collecting == 0U && maintenance_end_count == 0U,
                "denied request neither starts collection nor releases an unowned lock");

    fake_maintenance_result = MOTION_CONTROL_MAINTENANCE_OK;
    require_int(LineFollowing_RequestCalibration(LINE_CALIBRATION_SURFACE_FLOOR, 4U) != 0U,
                "denied request is cleared for retry");
    LineCalibrationCoordinator_ProcessRequest();
    LineFollowing_CalibrationGet(&calibration);
    require_int(calibration.collecting != 0U && maintenance_end_count == 1U,
                "authorized request starts collection and immediately releases maintenance");
}

static void test_safety_state_rejects_line_rearm(void)
{
    reset_fake();
    LineFollowing_Enable(0U);

    fake_maintenance = 1U;
    LineFollowing_Enable(1U);
    require_int(LineFollowing_IsEnabled() == 0U, "maintenance rejects line rearm");

    fake_maintenance = 0U;
    fake_estop       = 1U;
    LineFollowing_Enable(1U);
    require_int(LineFollowing_IsEnabled() == 0U, "estop rejects line rearm");

    fake_estop      = 0U;
    fake_fault_stop = 1U;
    LineFollowing_Enable(1U);
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
    test_calibration_apply_and_commit_are_explicit();
    test_parameter_generation_applies_before_next_sensor_frame();
    test_calibration_begin_requires_stationary_safe_chassis();
    test_service_request_is_resolved_by_app_maintenance_gate();
    (void)printf("PASS: line control safety generation tests\n");
    return 0;
}
