#include "ps2_control.h"

#include "chassis_config.h"
#include "control_manager.h"
#include "imu_bmi270.h"
#include "led_status.h"
#include "line_calibration.h"
#include "line_control.h"
#include "ps2_hw.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static uint32_t           fake_primask;
static uint32_t           fake_tick_ms;
static uint32_t           fake_hal_tick_ms;
static uint8_t            fake_read_ok;
static uint8_t            fake_line_enabled;
static uint8_t            fake_estop;
static uint8_t            fake_fault_stop;
static uint8_t            fake_maintenance;
static ps2_hw_sample_t    fake_sample;
static imu_bmi270_state_t fake_imu;
static chassis_cmd_t      last_command;
static uint32_t           set_command_count;
static uint32_t           clear_source_count;
static uint32_t           fake_motion_revoke_generation;
static uint8_t            fake_maintenance_crosses_on_query;
static line_calibration_t fake_cal;
static uint8_t            fake_cal_build_result;

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

uint32_t __get_PRIMASK(void)
{
    return fake_primask;
}
void __disable_irq(void)
{
    fake_primask = 1U;
}
void __set_PRIMASK(uint32_t value)
{
    fake_primask = value;
}
uint32_t osKernelGetTickCount(void)
{
    return fake_tick_ms;
}
uint32_t HAL_GetTick(void)
{
    return fake_hal_tick_ms;
}

void Ps2Hw_Init(void)
{
}
uint8_t Ps2Hw_ReadSample(ps2_hw_sample_t *sample)
{
    if (fake_read_ok != 0U)
    {
        *sample = fake_sample;
    }
    return fake_read_ok;
}
uint8_t Ps2Hw_IsAnalogMode(uint8_t mode)
{
    return (mode == 0x73U) ? 1U : 0U;
}

control_command_result_t ControlManager_SetCommand(const chassis_cmd_t *cmd)
{
    last_command = *cmd;
    set_command_count++;
    return CONTROL_COMMAND_ACCEPTED;
}
control_command_result_t ControlManager_SetCommandForGeneration(const chassis_cmd_t *cmd, uint32_t expected_generation)
{
    if (expected_generation != fake_motion_revoke_generation)
    {
        return CONTROL_COMMAND_REJECTED;
    }
    return ControlManager_SetCommand(cmd);
}
void ControlManager_ClearSource(uint8_t source)
{
    require_int(source == CONTROL_SOURCE_PS2, "PS2 clears only its own source");
    clear_source_count++;
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
    if (fake_maintenance_crosses_on_query != 0U)
    {
        fake_maintenance_crosses_on_query = 0U;
        fake_motion_revoke_generation++;
    }
    return fake_maintenance;
}
uint32_t ControlManager_GetMotionRevokeGeneration(void)
{
    return fake_motion_revoke_generation;
}

void LineControl_Enable(uint8_t enable)
{
    fake_line_enabled = (enable != 0U) ? 1U : 0U;
}
uint8_t LineControl_IsEnabled(void)
{
    return fake_line_enabled;
}
void LineControl_CalibrationGet(line_calibration_t *cal)
{
    if (cal)
        *cal = fake_cal;
}
uint8_t LineControl_CalibrationBegin(line_calibration_surface_t s, uint16_t n)
{
    (void)s;
    (void)n;
    fake_cal.collecting = 1U;
    return 1U;
}
uint8_t LineControl_CalibrationBuild(uint16_t thresh[8], uint8_t *al)
{
    (void)thresh;
    (void)al;
    return fake_cal_build_result;
}
void LineControl_CalibrationCancel(void)
{
    fake_cal = (line_calibration_t){0};
}
uint8_t LineControl_CalibrationApplyToRam(void)
{
    return fake_cal_build_result;
}
uint8_t LineControl_CalibrationCommitToFlash(void)
{
    return 1U;
}
void LedStatus_SetMode(led_status_mode_t mode)
{
    (void)mode;
}
void ImuBmi270_GetState(imu_bmi270_state_t *state)
{
    *state = fake_imu;
}

static void reset_fake(void)
{
    fake_primask      = 0U;
    fake_tick_ms      = 1000U;
    fake_hal_tick_ms  = fake_tick_ms;
    fake_read_ok      = 1U;
    fake_line_enabled = 0U;
    fake_estop        = 0U;
    fake_fault_stop   = 0U;
    fake_maintenance  = 0U;
    fake_sample       = (ps2_hw_sample_t){
              .mode    = 0x73U,
              .right_x = PS2_AXIS_CENTER,
              .right_y = PS2_AXIS_CENTER,
              .left_x  = PS2_AXIS_CENTER,
              .left_y  = PS2_AXIS_CENTER,
    };
    fake_imu                          = (imu_bmi270_state_t){0};
    fake_imu.enabled                  = 1U;
    fake_imu.online                   = 1U;
    fake_imu.last_error               = IMU_BMI270_ERROR_NONE;
    fake_imu.init_state               = IMU_BMI270_INIT_STATE_SAMPLING;
    fake_imu.sample_count             = 1UL;
    fake_imu.last_update_ms           = fake_hal_tick_ms;
    fake_imu.gyro_calibrated          = 1U;
    fake_imu.filter_initialized       = 1U;
    last_command                      = (chassis_cmd_t){0};
    set_command_count                 = 0UL;
    clear_source_count                = 0UL;
    fake_motion_revoke_generation     = 0UL;
    fake_maintenance_crosses_on_query = 0U;
    fake_cal                          = (line_calibration_t){0};
    fake_cal_build_result             = 0U;
    Ps2Control_Init();
}

static void test_heading_imu_freshness_uses_hal_tick_epoch(void)
{
    ps2_control_state_t state;

    reset_fake();
    fake_tick_ms            = 25U;
    fake_hal_tick_ms        = 5000U;
    fake_imu.last_update_ms = 4990U;
    fake_sample.btn2        = PS2_MACRO_L1_MASK;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_active != 0U, "fresh HAL-timestamped IMU starts heading despite RTOS epoch offset");
}

static void test_center_submits_zero_or_yields_to_line(void)
{
    reset_fake();
    Ps2Control_Update();
    require_int(set_command_count == 1UL, "centered online PS2 submits command when line is off");
    require_int(last_command.linear_x == 0.0f && last_command.angular_z == 0.0f, "centered online PS2 command is zero");

    fake_line_enabled = 1U;
    fake_tick_ms += 20U;
    Ps2Control_Update();
    require_int(clear_source_count == 1UL, "centered PS2 yields to enabled line control");

    fake_sample.right_x = 0U;
    fake_tick_ms += 20U;
    Ps2Control_Update();
    require_int(set_command_count == 2UL && fabsf(last_command.angular_z) > 0.1f,
                "manual stick immediately retakes control from line");
}

static void test_heading_button_mapping(void)
{
    ps2_control_state_t state;

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_active != 0U && state.heading_target_deg == 90.0f, "L1 starts +90 relative heading");

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_R1_MASK;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_target_deg == -90.0f, "R1 starts -90 relative heading");

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L2_MASK;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_target_deg == 360.0f, "L2 starts +360 relative heading");

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_R2_MASK;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_target_deg == -360.0f, "R2 starts -360 relative heading");
}

static void test_imu_gate_and_runtime_cancel(void)
{
    ps2_control_state_t state;

    /* 未校准 IMU 拒绝启动 heading（ImuUsable 严格检查） */
    reset_fake();
    fake_imu.gyro_calibrated = 0U;
    fake_sample.btn2         = PS2_MACRO_L1_MASK;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_active == 0U, "uncalibrated IMU rejects heading macro");

    /* 进行中 transient error (SPI_ERROR) 或 IMU 掉线不应取消 heading：
     heading 进行中不再检查 IMU 状态，只靠 gyro 速率积分累积航向。 */
    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2Control_Update();
    fake_sample.btn2       = 0U;
    fake_imu.quality_flags = IMU_BMI270_QUALITY_SPI_ERROR;
    fake_imu.online        = 0U;
    fake_tick_ms += 20U;
    fake_imu.last_update_ms = fake_tick_ms;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_active != 0U, "IMU error/offline does NOT cancel active heading");
}

static void test_held_macro_retries_after_transient_imu_gate(void)
{
    ps2_control_state_t state;

    reset_fake();
    fake_imu.last_update_ms = fake_hal_tick_ms - 51U;
    fake_sample.btn2        = PS2_MACRO_L1_MASK;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_active == 0U, "stale IMU initially rejects held macro");

    fake_tick_ms += 20U;
    fake_hal_tick_ms += 20U;
    fake_imu.last_update_ms = fake_hal_tick_ms;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_active != 0U && state.heading_target_deg == 90.0f,
                "held macro starts when transient IMU gate clears");
}

static void test_released_macro_does_not_start_after_imu_gate_clears(void)
{
    ps2_control_state_t state;

    reset_fake();
    fake_imu.last_update_ms = fake_hal_tick_ms - 51U;
    fake_sample.btn2        = PS2_MACRO_L1_MASK;
    Ps2Control_Update();

    fake_sample.btn2 = 0U;
    fake_tick_ms += 20U;
    fake_hal_tick_ms += 20U;
    fake_imu.last_update_ms = fake_hal_tick_ms;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_active == 0U, "released macro cannot start after transient IMU gate clears");
}

static void test_offline_and_safety_stop_cancel_heading(void)
{
    ps2_control_state_t state;

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2Control_Update();
    fake_read_ok = 0U;
    Ps2Control_Update();
    Ps2Control_Update();
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.online == 0U && state.heading_active == 0U, "three failed reads cancel heading and mark offline");
    require_int(clear_source_count != 0UL, "offline PS2 clears its source");

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2Control_Update();
    fake_sample.btn2 = 0U;
    fake_fault_stop  = 1U;
    fake_tick_ms += 20U;
    fake_imu.last_update_ms = fake_tick_ms;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_active == 0U && state.heading_end_reason == RELATIVE_YAW_END_SAFETY_STOP,
                "fault stop cancels heading macro");
}

static void test_revoke_generation_cancels_heading_after_short_safety_event(void)
{
    ps2_control_state_t state;

    reset_fake();
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2Control_Update();
    fake_sample.btn2 = 0U;
    fake_motion_revoke_generation++;
    fake_tick_ms += 20U;
    fake_imu.last_update_ms = fake_tick_ms;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_active == 0U && state.heading_end_reason == RELATIVE_YAW_END_SAFETY_STOP,
                "motion revoke generation cancels an old heading macro");
    require_int(last_command.linear_x == 0.0f && last_command.angular_z == 0.0f,
                "revoked heading macro submits zero speed");
}

static void test_maintenance_rejects_new_heading_macro(void)
{
    ps2_control_state_t state;

    reset_fake();
    fake_maintenance = 1U;
    fake_sample.btn2 = PS2_MACRO_L1_MASK;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_active == 0U, "maintenance rejects a new heading macro");
    require_int(last_command.linear_x == 0.0f && last_command.angular_z == 0.0f,
                "maintenance-time macro input cannot create motion");
}

static void test_heading_uses_generation_from_input_sample(void)
{
    ps2_control_state_t state;

    reset_fake();
    fake_sample.btn2                  = PS2_MACRO_L1_MASK;
    fake_maintenance_crosses_on_query = 1U;
    Ps2Control_Update();
    Ps2Control_GetState(&state);
    require_int(state.heading_active == 0U, "maintenance crossing after input sample invalidates macro edge");
}

int main(void)
{
    test_center_submits_zero_or_yields_to_line();
    test_heading_button_mapping();
    test_heading_imu_freshness_uses_hal_tick_epoch();
    test_imu_gate_and_runtime_cancel();
    test_held_macro_retries_after_transient_imu_gate();
    test_released_macro_does_not_start_after_imu_gate_clears();
    test_offline_and_safety_stop_cancel_heading();
    test_revoke_generation_cancels_heading_after_short_safety_event();
    test_maintenance_rejects_new_heading_macro();
    test_heading_uses_generation_from_input_sample();
    (void)printf("PASS: PS2 control host tests\n");
    return 0;
}
