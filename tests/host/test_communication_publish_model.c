#include "communication_publish_model_service.h"
#include "power_on_self_test_service.h"

#include "command_management_service.h"
#include "wireless_communication_service.h"
#include "line_following_service.h"
#include "motion_control_service.h"
#include "platform_critical.h"
#include "power_management_service.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"
#include "system_monitoring_service.h"
#include "teleoperation_service.h"
#include "host_communication_service.h"

#include <assert.h>
#include <stdio.h>

static motion_control_status_t        fake_motion;
static state_estimation_status_t      fake_state;
static power_management_status_t      fake_power;
static safety_management_status_t     fake_safety;
static system_monitoring_status_t     fake_system;
static power_on_self_test_result_t    fake_post;
static host_communication_state_t     fake_upper;
static wireless_communication_state_t fake_esp;
static teleoperation_status_t         fake_teleoperation;
static line_following_status_t        fake_line;
static uint32_t                       fake_upper_last_rx_timestamp_ms;

platform_critical_state_t PlatformCritical_Enter(void)
{
    return 0U;
}
void PlatformCritical_Exit(platform_critical_state_t state)
{
    (void)state;
}
uint32_t MotionControl_GetStatus(motion_control_status_t *state)
{
    *state = fake_motion;
    return state->generation;
}
uint32_t StateEstimation_GetStatus(uint32_t now_ms, state_estimation_status_t *state)
{
    (void)now_ms;
    *state = fake_state;
    return state->wheel_generation + state->imu_generation;
}
uint32_t PowerManagement_GetStatus(power_management_status_t *state)
{
    *state = fake_power;
    return 1U;
}
uint32_t SafetyManagement_GetStatus(safety_management_status_t *state)
{
    *state = fake_safety;
    return state->generation;
}
uint32_t SystemMonitoring_GetStatus(system_monitoring_status_t *state)
{
    *state = fake_system;
    return state->generation;
}
void SystemMonitoring_SetModuleHealth(const system_monitoring_module_health_t *modules)
{
    fake_system.modules = *modules;
}
uint32_t LineFollowing_GetStatus(line_following_status_t *state)
{
    *state = fake_line;
    return state->generation;
}
void PowerOnSelfTest_GetResult(power_on_self_test_result_t *result)
{
    *result = fake_post;
}
void HostCommunication_GetState(host_communication_state_t *state)
{
    *state = fake_upper;
}
uint32_t HostCommunication_GetLastRxTimestamp(void)
{
    return fake_upper_last_rx_timestamp_ms;
}
void WirelessCommunication_GetState(wireless_communication_state_t *state)
{
    *state = fake_esp;
}
uint32_t Teleoperation_GetStatus(teleoperation_status_t *state)
{
    *state = fake_teleoperation;
    return state->generation;
}
command_source_t CommandManagement_GetActiveSource(uint32_t now_ms)
{
    (void)now_ms;
    return COMMAND_SOURCE_HOST;
}

static void SeedInputs(void)
{
    fake_motion                                    = (motion_control_status_t){0};
    fake_state                                     = (state_estimation_status_t){0};
    fake_power                                     = (power_management_status_t){0};
    fake_safety                                    = (safety_management_status_t){0};
    fake_system                                    = (system_monitoring_status_t){0};
    fake_post                                      = (power_on_self_test_result_t){0};
    fake_upper                                     = (host_communication_state_t){0};
    fake_esp                                       = (wireless_communication_state_t){0};
    fake_teleoperation                             = (teleoperation_status_t){0};
    fake_line                                      = (line_following_status_t){0};
    fake_upper_last_rx_timestamp_ms                = 800U;
    fake_motion.motor_target_mps[0]                = 0.5f;
    fake_motion.motor_actual_mps[0]                = 0.4f;
    fake_motion.motor_effective_output_permille[0] = 123;
    fake_motion.motor_enabled_mask                 = 0x0FU;
    fake_state.wheel.count[0]                      = 42;
    fake_state.wheel.speed_valid[0]                = 1U;
    fake_state.wheel.speed_valid_all               = 1U;
    fake_state.wheel.anomaly_count[1]              = 1U;
    fake_power.current_valid                       = 1U;
    fake_power.invalid_reason_flags                = 0x55UL;
    fake_state.imu.online                          = 1U;
    fake_state.imu.chip_id                         = 0x24U;
    fake_state.imu.gyro_calibrated                 = 1U;
    fake_state.imu.last_update_ms                  = 990U;
    fake_state.imu.sample_count                    = 7U;
    fake_safety.battery_voltage                    = 12.3f;
    fake_safety.motor_current_a[0]                 = 0.7f;
    fake_safety.control_mode                       = COMMAND_SOURCE_HOST;
    fake_safety.emergency_stop                     = 1U;
    fake_safety.motor_fault_mask                   = (1U << 1);
    fake_system.reset_reason_flags                 = 0x1234UL;
    fake_post.done                                 = 1U;
    fake_upper.last_valid_frame_ms                 = 800U;
    fake_esp.last_rx_timestamp_ms                  = 850U;
    fake_teleoperation.online                      = 1U;
    fake_line.globally_enabled                     = 1U;
    fake_line.sensor_valid                         = 1U;
    fake_line.sensor_timestamp_ms                  = 980U;
}

int main(void)
{
    communication_publish_model_t              snapshot;
    const communication_publish_model_config_t config = {
        .host_timeout_ms   = 500U,
        .esp12f_timeout_ms = 500U,
        .line_timeout_ms   = 50U,
    };

    SeedInputs();
    assert(CommunicationPublishModel_Init(&config) != 0U);
    assert(CommunicationPublishModel_Get(0) == 0U);
    assert(CommunicationPublishModel_Get(&snapshot) == 0U);
    CommunicationPublishModel_Update(1000U);
    assert(CommunicationPublishModel_Get(&snapshot) == 1U);
    assert(snapshot.generation == 1U);
    assert(snapshot.chassis.motor_target_mps[0] == 0.5f);
    assert(snapshot.chassis.motor_output_permille[0] == 123);
    assert(snapshot.encoder.count[0] == 42);
    assert(snapshot.encoder.anomaly_mask == (1U << 1));
    assert(snapshot.safety.battery_voltage == 12.3f);
    assert(snapshot.safety.motor_fault_mask == (1U << 1));
    assert(snapshot.imu.chip_id == 0x24U);
    assert(snapshot.imu.sample_count == 7U);
    assert(snapshot.control.emergency_stop == 1U);
    assert(snapshot.control.line_enabled == 1U);
    assert(snapshot.control.reset_reason_flags == 0x1234UL);
    assert(snapshot.modules.upper_online == 1U);
    assert(snapshot.modules.esp12f_online == 1U);
    assert(snapshot.modules.line_online == 1U);
    assert(snapshot.modules.ps2_online == 1U);

    fake_upper.last_valid_frame_ms  = 1U;
    fake_upper_last_rx_timestamp_ms = 1U;
    fake_esp.boot_mode_download     = 1U;
    fake_line.sensor_timestamp_ms   = 1U;
    CommunicationPublishModel_Update(2000U);
    assert(CommunicationPublishModel_Get(&snapshot) == 2U);
    assert(snapshot.modules.upper_online == 0U);
    assert(snapshot.modules.esp12f_online == 0U);
    assert(snapshot.modules.line_online == 0U);
    puts("app publish model tests passed");
    return 0;
}
