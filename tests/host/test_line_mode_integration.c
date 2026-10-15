#include "command_management_service.h"
#include "control_mode_coordinator.h"
#include "line_following_service.h"
#include "line_sensor_driver.h"
#include "parameter_management_service.h"
#include "safety_management_service.h"

#include <assert.h>
#include <stdio.h>

static uint32_t now_ms = 100U;
static param_model_t params;
static line_sensor_data_t sensor;

uint32_t osKernelGetTickCount(void) { return now_ms; }
uint32_t ParameterManagement_GetSnapshot(param_model_t *value) { *value = params; return 1UL; }
uint8_t ParameterManagement_Set(const param_model_t *value) { params = *value; return 1U; }
uint8_t SafetyManagement_IsMotionAllowed(void) { return CommandManagement_IsMotionGateOpen(); }
void LineSensorDriver_Update(void) {}
void LineSensorDriver_RequestAnalog(void) {}
uint8_t LineSensorDriver_GetSensorData(line_sensor_data_t *value) { *value = sensor; return 1U; }
void LineSensorDriver_SetThresholdConfig(const uint16_t threshold[LINE_SENSOR_CHANNELS], uint8_t active_low)
{ (void)threshold; (void)active_low; }

static void update_sensor(uint32_t time_ms)
{
    now_ms = time_ms;
    sensor.timestamp_ms = time_ms;
    LineFollowing_Update();
}

int main(void)
{
    params = (param_model_t){.max_linear_mps = 0.5f, .max_angular_rps = 10.0f, .track_width_m = 0.176f,
        .wheel_radius_m = 0.035f, .line_kp = 0.5f, .line_kd = 0.1f, .line_speed_mps = 0.2f,
        .line_detect_debounce_frames = 2U, .line_lost_debounce_frames = 2U};
    sensor = (line_sensor_data_t){.valid = 1U, .timestamp_ms = now_ms};
    sensor.state[3] = sensor.state[4] = 1U;
    const command_management_config_t command_config = {.host_timeout_ms = 200U, .ps2_timeout_ms = 500U,
        .esp12f_timeout_ms = 500U, .line_timeout_ms = 50U, .debug_timeout_ms = 2000U,
        .remote_max_lifetime_ms = 2000U};
    const control_mode_config_t mode_config = {.takeover_enter_threshold = 0.15f, .takeover_exit_threshold = 0.1f,
        .manual_neutral_restore_ms = 2000U, .takeover_confirm_samples = 3U};
    const line_following_config_t line_config = {.angular_max_rps = 2.0f, .sensor_timeout_ms = 50U,
        .default_enabled = 0U, .detect_threshold_count = 1U};
    assert(CommandManagement_Init(&command_config));
    assert(ControlModeCoordinator_Init(&mode_config));
    assert(ControlModeCoordinator_ApplyCapabilityMask(COMMAND_ALL_SOURCE_MASK & ~COMMAND_SOURCE_MASK_LINE));
    CommandManagement_SetMotionGate(1U, 100UL);
    assert(LineFollowing_Init(&line_config));
    LineFollowing_Update();
    line_following_status_t line;
    (void)LineFollowing_GetStatus(&line);
    assert(line.sensor_valid && !line.globally_enabled);

    /* Optional LINE availability must not revoke a live AUTO command. */
    assert(CommandManagement_QualifyRearm(COMMAND_SOURCE_HOST).outcome == COMMAND_OUTCOME_RELEASE_ACCEPTED);
    command_velocity_t host = {.linear_x = 0.1f, .enable = 1U, .source = COMMAND_SOURCE_HOST,
        .timestamp_ms = now_ms};
    assert(CommandManagement_Set(&host) == COMMAND_RESULT_ACCEPTED);
    uint32_t generation = CommandManagement_GetMotionRevokeGeneration();
    assert(ControlModeCoordinator_ApplyCapabilityMask(COMMAND_ALL_SOURCE_MASK));
    assert(generation == CommandManagement_GetMotionRevokeGeneration());
    assert(CommandManagement_GetActiveSource(now_ms) == COMMAND_SOURCE_HOST);

    /* The first explicit enable after the sensor fact is available must remain enabled. */
    assert(ControlModeCoordinator_Request(CONTROL_MODE_LINE));
    assert(LineFollowing_Enable(0U) == LINE_FOLLOWING_RESULT_APPLIED);
    assert(LineFollowing_Enable(1U) == LINE_FOLLOWING_RESULT_APPLIED);
    update_sensor(110U);
    update_sensor(120U);
    assert(ControlModeCoordinator_ApplyCapabilityMask(COMMAND_ALL_SOURCE_MASK));
    update_sensor(130U);
    (void)LineFollowing_GetStatus(&line);
    assert(line.globally_enabled && line.tracking_active);
    assert(CommandManagement_GetActiveSource(now_ms) == COMMAND_SOURCE_LINE);

    assert(LineFollowing_Enable(0U) == LINE_FOLLOWING_RESULT_APPLIED);
    now_ms = 181U;
    LineFollowing_Update();
    (void)LineFollowing_GetStatus(&line);
    assert(!line.globally_enabled && !line.sensor_valid);
    update_sensor(190U);
    (void)LineFollowing_GetStatus(&line);
    assert(line.sensor_valid && !line.globally_enabled);
    puts("PASS: LINE facts, capability and command integration");
    return 0;
}
