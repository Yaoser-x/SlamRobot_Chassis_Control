#include "line_control_service.h"

#include "control_service.h"
#include "chassis_maintenance_service.h"
#include "flash_param.h"
#include "imu_bmi270.h"
#include "line_uart.h"
#include "param_service.h"

#include <stdio.h>
#include <stdlib.h>

static uint32_t                             fake_revoke_generation;
static uint32_t                             fake_tick;
static uint32_t                             submitted_count;
static uint32_t                             clear_count;
static uint8_t                              fake_estop;
static uint8_t                              fake_fault_stop;
static uint8_t                              fake_maintenance;
static chassis_cmd_t                        last_command;
static line_sensor_data_t                   fake_sensor;
static param_model_t                        fake_params;
static chassis_maintenance_service_result_t fake_maintenance_result;
static flash_param_status_t                 fake_flash_status;
static uint32_t                             maintenance_end_count;
static uint32_t                             flash_save_count;

static void require_int(int condition, const char *message);

uint32_t ParamService_GetSnapshot(param_model_t *params)
{
    *params = fake_params;
    return 1U;
}
void ParamService_Get(param_model_t *params)
{
    *params = fake_params;
}
uint8_t ParamService_Set(const param_model_t *params)
{
    fake_params = *params;
    return 1U;
}

chassis_maintenance_service_result_t ChassisMaintenanceService_Begin(void)
{
    return fake_maintenance_result;
}
void ChassisMaintenanceService_End(void)
{
    maintenance_end_count++;
}
void ImuBmi270_GetCalibration(imu_bmi270_calibration_t *calibration)
{
    *calibration = (imu_bmi270_calibration_t){0};
}
flash_param_status_t FlashParam_SaveBundle(const flash_param_bundle_t *bundle)
{
    require_int(bundle != NULL, "flash commit provides a bundle");
    flash_save_count++;
    return fake_flash_status;
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

uint32_t ControlService_GetMotionRevokeGeneration(void)
{
    return fake_revoke_generation;
}

uint8_t ControlService_IsEmergencyStop(void)
{
    return fake_estop;
}
uint8_t ControlService_IsFaultStop(void)
{
    return fake_fault_stop;
}
uint8_t ControlService_IsMaintenanceLocked(void)
{
    return fake_maintenance;
}

control_command_result_t ControlService_SetCommand(const chassis_cmd_t *cmd)
{
    last_command = *cmd;
    submitted_count++;
    return CONTROL_COMMAND_ACCEPTED;
}

control_command_result_t ControlService_SetCommandForGeneration(const chassis_cmd_t *cmd, uint32_t expected_generation)
{
    if (expected_generation != fake_revoke_generation)
    {
        return CONTROL_COMMAND_REJECTED;
    }
    return ControlService_SetCommand(cmd);
}

void ControlService_ClearSource(uint8_t source)
{
    if (source == CONTROL_SOURCE_LINE)
    {
        clear_count++;
    }
}

uint8_t LineUart_GetSensorData(line_sensor_data_t *data)
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
    last_command                            = (chassis_cmd_t){0};
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
    fake_maintenance_result                 = CHASSIS_MAINTENANCE_SERVICE_OK;
    fake_flash_status                       = FLASH_PARAM_STATUS_OK;
    maintenance_end_count                   = 0U;
    flash_save_count                        = 0U;
    LineControlService_Init();
    LineControlService_Enable(1U);
}

static void collect_calibration_surface(line_calibration_surface_t surface, uint16_t base)
{
    require_int(LineControlService_CalibrationBegin(surface, 4U) != 0U, "line calibration collection starts");
    for (uint8_t sample = 0U; sample < 4U; ++sample)
    {
        for (uint8_t channel = 0U; channel < LINE_SENSOR_CHANNELS; ++channel)
        {
            fake_sensor.analog[channel] = (uint16_t)(base + channel + sample);
        }
        fake_sensor.timestamp_ms++;
        fake_tick++;
        LineControlService_Update();
    }
}

static void test_calibration_apply_and_commit_are_explicit(void)
{
    uint16_t old_threshold;

    reset_fake();
    old_threshold = fake_params.line_threshold_raw[0];
    collect_calibration_surface(LINE_CALIBRATION_SURFACE_FLOOR, 900U);
    require_int(LineControlService_CalibrationApplyToRam() == 0U, "single surface cannot overwrite RAM parameters");
    require_int(fake_params.line_threshold_raw[0] == old_threshold, "rejected apply preserves prior parameters");
    collect_calibration_surface(LINE_CALIBRATION_SURFACE_LINE, 200U);
    require_int(LineControlService_CalibrationApplyToRam() != 0U, "two separated surfaces apply to RAM");
    require_int(fake_params.line_threshold_raw[0] == 551U && fake_params.line_active_low != 0U,
                "RAM apply updates thresholds and polarity");
    require_int(flash_save_count == 0U, "RAM apply does not write flash");
    maintenance_end_count = 0U;

    require_int(LineControlService_CalibrationCommitToFlash() != 0U, "explicit flash commit succeeds");
    require_int(flash_save_count == 1U && maintenance_end_count == 1U,
                "flash commit saves once and releases maintenance");

    fake_maintenance_result = CHASSIS_MAINTENANCE_SERVICE_NOT_STATIONARY;
    require_int(LineControlService_CalibrationCommitToFlash() == 0U, "moving chassis rejects flash commit");
    require_int(flash_save_count == 1U && maintenance_end_count == 1U,
                "rejected commit neither saves nor releases an unowned lock");

    fake_maintenance_result = CHASSIS_MAINTENANCE_SERVICE_OK;
    fake_flash_status       = FLASH_PARAM_STATUS_WRITE_ERROR;
    require_int(LineControlService_CalibrationCommitToFlash() == 0U, "flash write failure is reported");
    require_int(flash_save_count == 2U && maintenance_end_count == 2U, "failed flash write still releases maintenance");
}

static void test_calibration_begin_requires_stationary_safe_chassis(void)
{
    line_calibration_t calibration;

    reset_fake();
    fake_maintenance_result = CHASSIS_MAINTENANCE_SERVICE_NOT_STATIONARY;
    require_int(LineControlService_CalibrationBegin(LINE_CALIBRATION_SURFACE_FLOOR, 4U) == 0U,
                "moving chassis rejects calibration collection");
    LineControlService_CalibrationGet(&calibration);
    require_int(calibration.collecting == 0U, "rejected moving collection leaves calibration idle");

    fake_maintenance_result = CHASSIS_MAINTENANCE_SERVICE_OK;
    fake_estop              = 1U;
    require_int(LineControlService_CalibrationBegin(LINE_CALIBRATION_SURFACE_FLOOR, 4U) == 0U,
                "estop rejects calibration collection");
    fake_estop      = 0U;
    fake_fault_stop = 1U;
    require_int(LineControlService_CalibrationBegin(LINE_CALIBRATION_SURFACE_FLOOR, 4U) == 0U,
                "fault stop rejects calibration collection");
    fake_fault_stop  = 0U;
    fake_maintenance = 1U;
    require_int(LineControlService_CalibrationBegin(LINE_CALIBRATION_SURFACE_FLOOR, 4U) == 0U,
                "existing maintenance lock rejects calibration collection");
}

static void test_safety_state_rejects_line_rearm(void)
{
    reset_fake();
    LineControlService_Enable(0U);

    fake_maintenance = 1U;
    LineControlService_Enable(1U);
    require_int(LineControlService_IsEnabled() == 0U, "maintenance rejects line rearm");

    fake_maintenance = 0U;
    fake_estop       = 1U;
    LineControlService_Enable(1U);
    require_int(LineControlService_IsEnabled() == 0U, "estop rejects line rearm");

    fake_estop      = 0U;
    fake_fault_stop = 1U;
    LineControlService_Enable(1U);
    require_int(LineControlService_IsEnabled() == 0U, "fault stop rejects line rearm");
}

static void test_safety_generation_revokes_old_line_enable(void)
{
    reset_fake();
    LineControlService_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControlService_Update();
    require_int(submitted_count == 1U && last_command.source == CONTROL_SOURCE_LINE,
                "enabled line submits before safety revocation");

    fake_revoke_generation++;
    LineControlService_Update();
    require_int(LineControlService_IsEnabled() == 0U, "old line enable is revoked");
    require_int(submitted_count == 1U, "revoked line cannot resubmit stale motion");
    require_int(clear_count != 0U, "revoked line source is cleared");

    LineControlService_Enable(1U);
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControlService_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControlService_Update();
    require_int(submitted_count == 2U, "new explicit line enable can move again");
}

static void test_pd_slowdown_and_lost_debounce(void)
{
    line_control_service_state_t state;
    reset_fake();
    fake_sensor.state[3] = 0U;
    fake_sensor.state[4] = 0U;
    fake_sensor.state[6] = 1U;
    fake_sensor.state[7] = 1U;
    LineControlService_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControlService_Update();
    LineControlService_GetState(&state);
    require_int(state.tracking_active != 0U && state.error < -2.0f,
                "offset line tracked (right side → negative error)");
    require_int(state.linear_x < fake_params.line_speed_mps, "large error reduces speed");

    for (uint8_t i = 0U; i < 8U; ++i)
    {
        fake_sensor.state[i] = 0U;
    }
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControlService_Update();
    require_int(clear_count == 0U, "single lost frame debounced");
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControlService_Update();
    require_int(clear_count == 1U, "confirmed lost line clears source");
}

static void test_debounce_counts_unique_sensor_frames(void)
{
    reset_fake();
    LineControlService_Update();
    LineControlService_Update();
    require_int(submitted_count == 0U, "repeated timestamp does not satisfy detect debounce");
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControlService_Update();
    require_int(submitted_count == 1U, "second unique frame satisfies detect debounce");

    for (uint8_t i = 0U; i < 8U; ++i)
    {
        fake_sensor.state[i] = 0U;
    }
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControlService_Update();
    LineControlService_Update();
    require_int(clear_count == 0U, "repeated lost timestamp does not satisfy debounce");
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControlService_Update();
    require_int(clear_count == 1U, "second unique lost frame clears source");
}

static void test_stale_sensor_clears_tracking_view_state(void)
{
    line_control_service_state_t state;
    reset_fake();
    LineControlService_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControlService_Update();
    LineControlService_GetState(&state);
    require_int(state.tracking_active != 0U, "tracking active before stale sample");
    fake_sensor.valid = 0U;
    LineControlService_Update();
    LineControlService_GetState(&state);
    require_int(state.tracking_active == 0U && state.lost_reason == 1U,
                "stale sample clears tracking state and reports stale reason");
}

int main(void)
{
    test_safety_generation_revokes_old_line_enable();
    test_safety_state_rejects_line_rearm();
    test_pd_slowdown_and_lost_debounce();
    test_debounce_counts_unique_sensor_frames();
    test_stale_sensor_clears_tracking_view_state();
    test_calibration_apply_and_commit_are_explicit();
    test_calibration_begin_requires_stationary_safe_chassis();
    (void)printf("PASS: line control safety generation tests\n");
    return 0;
}
