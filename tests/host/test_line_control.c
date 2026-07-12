#include "line_control.h"

#include "control_manager.h"
#include "line_uart.h"
#include "param_store.h"

#include <stdio.h>
#include <stdlib.h>

static uint32_t           fake_revoke_generation;
static uint32_t           fake_tick;
static uint32_t           submitted_count;
static uint32_t           clear_count;
static uint8_t            fake_estop;
static uint8_t            fake_fault_stop;
static uint8_t            fake_maintenance;
static chassis_cmd_t      last_command;
static line_sensor_data_t fake_sensor;
static param_store_t      fake_params;

uint32_t ParamStore_GetSnapshot(param_store_t *params)
{
    *params = fake_params;
    return 1U;
}
void ParamStore_Get(param_store_t *params)
{
    *params = fake_params;
}
uint8_t ParamStore_Set(const param_store_t *params)
{
    fake_params = *params;
    return 1U;
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

uint32_t ControlManager_GetMotionRevokeGeneration(void)
{
    return fake_revoke_generation;
}

uint8_t ControlManager_IsEmergencyStop(void)
{
    return fake_estop;
}
uint8_t ControlManager_IsFaultStop(void)
{
    return fake_fault_stop;
}
uint8_t ControlManager_IsMaintenanceLocked(void)
{
    return fake_maintenance;
}

control_command_result_t ControlManager_SetCommand(const chassis_cmd_t *cmd)
{
    last_command = *cmd;
    submitted_count++;
    return CONTROL_COMMAND_ACCEPTED;
}

control_command_result_t ControlManager_SetCommandForGeneration(const chassis_cmd_t *cmd, uint32_t expected_generation)
{
    if (expected_generation != fake_revoke_generation)
    {
        return CONTROL_COMMAND_REJECTED;
    }
    return ControlManager_SetCommand(cmd);
}

void ControlManager_ClearSource(uint8_t source)
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
    fake_params                             = (param_store_t){0};
    fake_params.line_kp                     = 0.5f;
    fake_params.line_kd                     = 0.1f;
    fake_params.line_speed_mps              = 0.2f;
    fake_params.line_slowdown_gain          = 0.5f;
    fake_params.line_detect_debounce_frames = 2U;
    fake_params.line_lost_debounce_frames   = 2U;
    LineControl_Init();
    LineControl_Enable(1U);
}

static void test_safety_state_rejects_line_rearm(void)
{
    reset_fake();
    LineControl_Enable(0U);

    fake_maintenance = 1U;
    LineControl_Enable(1U);
    require_int(LineControl_IsEnabled() == 0U, "maintenance rejects line rearm");

    fake_maintenance = 0U;
    fake_estop       = 1U;
    LineControl_Enable(1U);
    require_int(LineControl_IsEnabled() == 0U, "estop rejects line rearm");

    fake_estop      = 0U;
    fake_fault_stop = 1U;
    LineControl_Enable(1U);
    require_int(LineControl_IsEnabled() == 0U, "fault stop rejects line rearm");
}

static void test_safety_generation_revokes_old_line_enable(void)
{
    reset_fake();
    LineControl_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControl_Update();
    require_int(submitted_count == 1U && last_command.source == CONTROL_SOURCE_LINE,
                "enabled line submits before safety revocation");

    fake_revoke_generation++;
    LineControl_Update();
    require_int(LineControl_IsEnabled() == 0U, "old line enable is revoked");
    require_int(submitted_count == 1U, "revoked line cannot resubmit stale motion");
    require_int(clear_count != 0U, "revoked line source is cleared");

    LineControl_Enable(1U);
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControl_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControl_Update();
    require_int(submitted_count == 2U, "new explicit line enable can move again");
}

static void test_pd_slowdown_and_lost_debounce(void)
{
    line_control_state_t state;
    reset_fake();
    fake_sensor.state[3] = 0U;
    fake_sensor.state[4] = 0U;
    fake_sensor.state[6] = 1U;
    fake_sensor.state[7] = 1U;
    LineControl_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControl_Update();
    LineControl_GetState(&state);
    require_int(state.tracking_active != 0U && state.error < -2.0f,
                "offset line tracked (right side → negative error)");
    require_int(state.linear_x < fake_params.line_speed_mps, "large error reduces speed");

    for (uint8_t i = 0U; i < 8U; ++i)
    {
        fake_sensor.state[i] = 0U;
    }
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControl_Update();
    require_int(clear_count == 0U, "single lost frame debounced");
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControl_Update();
    require_int(clear_count == 1U, "confirmed lost line clears source");
}

static void test_debounce_counts_unique_sensor_frames(void)
{
    reset_fake();
    LineControl_Update();
    LineControl_Update();
    require_int(submitted_count == 0U, "repeated timestamp does not satisfy detect debounce");
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControl_Update();
    require_int(submitted_count == 1U, "second unique frame satisfies detect debounce");

    for (uint8_t i = 0U; i < 8U; ++i)
    {
        fake_sensor.state[i] = 0U;
    }
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControl_Update();
    LineControl_Update();
    require_int(clear_count == 0U, "repeated lost timestamp does not satisfy debounce");
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControl_Update();
    require_int(clear_count == 1U, "second unique lost frame clears source");
}

static void test_stale_sensor_clears_tracking_view_state(void)
{
    line_control_state_t state;
    reset_fake();
    LineControl_Update();
    fake_sensor.timestamp_ms++;
    fake_tick++;
    LineControl_Update();
    LineControl_GetState(&state);
    require_int(state.tracking_active != 0U, "tracking active before stale sample");
    fake_sensor.valid = 0U;
    LineControl_Update();
    LineControl_GetState(&state);
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
    (void)printf("PASS: line control safety generation tests\n");
    return 0;
}
