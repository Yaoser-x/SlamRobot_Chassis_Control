#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "control_config.h"
#include "bsp_config.h"
#include "motor_hardware_layout.h"
#include "differential_drive_kinematics.h"
#include "command_management_service.h"
#include "system_monitoring_service.h"
#include "control_service.h"
#include "wheel_speed_estimator.h"
#include "bmi270_driver.h"
#include "motor_ph_en_mapper.h"
#include "param_service.h"
#include "wheel_speed_pid_controller.h"
#include "safety_management_service.h"
#include "state_estimation_status.h"
#include "robot_link_protocol.h"

static uint32_t fake_primask;
static uint32_t fake_tick;
static uint8_t  fake_safety_initialized;
static uint8_t  fake_emergency_stop;
static uint8_t  fake_fault_stop;
static uint8_t  fake_maintenance_lock;
static uint32_t fake_safety_generation;

uint32_t __get_PRIMASK(void)
{
    return fake_primask;
}

void __disable_irq(void)
{
    fake_primask = 1U;
}

void __set_PRIMASK(uint32_t primask)
{
    fake_primask = primask;
}

uint32_t osKernelGetTickCount(void)
{
    return fake_tick;
}

int32_t osDelayUntil(uint32_t ticks)
{
    fake_tick = ticks;
    return 0;
}

static uint8_t fake_motion_allowed(void)
{
    return (fake_emergency_stop == 0U && fake_fault_stop == 0U && fake_maintenance_lock == 0U) ? 1U : 0U;
}

uint8_t SafetyManagement_Init(const safety_management_config_t *config)
{
    if (config == NULL)
    {
        return 0U;
    }
    fake_safety_initialized = 1U;
    fake_emergency_stop     = 0U;
    fake_fault_stop         = 0U;
    fake_maintenance_lock   = 0U;
    fake_safety_generation++;
    CommandManagement_SetMotionGate(1U, fake_safety_generation);
    return 1U;
}

uint8_t SafetyManagement_IsInitialized(void)
{
    return fake_safety_initialized;
}

void SafetyManagement_SetEmergencyStop(uint8_t enabled)
{
    fake_emergency_stop = (enabled != 0U) ? 1U : 0U;
    fake_safety_generation++;
    CommandManagement_SetMotionGate(fake_motion_allowed(), fake_safety_generation);
}

void SafetyManagement_SetFaultStop(uint8_t enabled)
{
    fake_fault_stop = (enabled != 0U) ? 1U : 0U;
    fake_safety_generation++;
    CommandManagement_SetMotionGate(fake_motion_allowed(), fake_safety_generation);
}

uint8_t SafetyManagement_BeginMaintenance(void)
{
    if (fake_maintenance_lock != 0U)
    {
        return 0U;
    }
    fake_maintenance_lock = 1U;
    fake_safety_generation++;
    CommandManagement_SetMotionGate(0U, fake_safety_generation);
    return 1U;
}

void SafetyManagement_EndMaintenance(void)
{
    fake_maintenance_lock = 0U;
    fake_safety_generation++;
    CommandManagement_SetMotionGate(fake_motion_allowed(), fake_safety_generation);
}

uint8_t SafetyManagement_IsEmergencyStop(void)
{
    return fake_emergency_stop;
}

uint8_t SafetyManagement_IsFaultStop(void)
{
    return fake_fault_stop;
}

uint8_t SafetyManagement_IsMaintenanceLocked(void)
{
    return fake_maintenance_lock;
}

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)printf("FAIL: %s\n", message);
        _Exit(1);
    }
}

static void require_close(float actual, float expected, float epsilon, const char *message)
{
    float error = actual - expected;
    if (error < 0.0f)
    {
        error = -error;
    }
    if (error > epsilon)
    {
        (void)printf("FAIL: %s actual=%f expected=%f\n", message, actual, expected);
        _Exit(1);
    }
}

static void write_float_le(uint8_t *out, float value)
{
    uint32_t raw = 0U;
    (void)memcpy(&raw, &value, sizeof(raw));
    out[0] = (uint8_t)(raw & 0xFFU);
    out[1] = (uint8_t)((raw >> 8) & 0xFFU);
    out[2] = (uint8_t)((raw >> 16) & 0xFFU);
    out[3] = (uint8_t)((raw >> 24) & 0xFFU);
}

static uint16_t read_u16_le(const uint8_t *in)
{
    return (uint16_t)(((uint16_t)in[0]) | ((uint16_t)in[1] << 8));
}

static int16_t read_i16_le(const uint8_t *in)
{
    return (int16_t)read_u16_le(in);
}

static uint32_t read_u32_le(const uint8_t *in)
{
    return ((uint32_t)in[0]) | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

static int32_t read_i32_le(const uint8_t *in)
{
    return (int32_t)read_u32_le(in);
}

static float read_float_le(const uint8_t *in)
{
    uint32_t raw = read_u32_le(in);
    float    value;

    (void)memcpy(&value, &raw, sizeof(value));
    return value;
}

static void test_protocol_frame_and_velocity(void)
{
    uint8_t                  frame[ROBOT_LINK_PROTOCOL_MAX_FRAME]              = {0};
    uint8_t                  payload[ROBOT_LINK_PROTOCOL_VELOCITY_PAYLOAD_LEN] = {0};
    upper_velocity_payload_t velocity                                          = {0};
    uint16_t                 frame_len;
    uint8_t                  one_byte_payload = 1U;

    write_float_le(&payload[0], 0.25f);
    write_float_le(&payload[4], -1.5f);
    payload[8] = 1U;
    payload[9] = 2U;

    frame_len = UpperProtocol_BuildFrame(UPPER_CMD_SET_VELOCITY,
                                         payload,
                                         ROBOT_LINK_PROTOCOL_VELOCITY_PAYLOAD_LEN,
                                         frame,
                                         (uint16_t)sizeof(frame));
    require_int(frame_len == 15U, "velocity frame length");
    require_int(frame[0] == ROBOT_LINK_PROTOCOL_HEAD_0 && frame[1] == ROBOT_LINK_PROTOCOL_HEAD_1, "frame header");
    require_int(frame[2] == ROBOT_LINK_PROTOCOL_CMD_LEN(ROBOT_LINK_PROTOCOL_VELOCITY_PAYLOAD_LEN),
                "frame command length");
    require_int(frame[frame_len - 1U] == UpperProtocol_Checksum8(&frame[2], 12U), "frame checksum");
    require_int(UpperProtocol_ParseVelocityPayload(payload, (uint8_t)sizeof(payload), &velocity) != 0U,
                "velocity payload parse");
    require_close(velocity.linear_x, 0.25f, 0.0001f, "velocity linear");
    require_close(velocity.angular_z, -1.5f, 0.0001f, "velocity angular");
    require_int(velocity.enable == 1U && velocity.mode == 2U, "velocity flags");

    frame_len = UpperProtocol_BuildFrame(UPPER_CMD_ESTOP,
                                         &one_byte_payload,
                                         ROBOT_LINK_PROTOCOL_ESTOP_PAYLOAD_LEN,
                                         frame,
                                         (uint16_t)sizeof(frame));
    require_int(frame_len == 6U, "estop frame length");
    require_int(frame[3] == UPPER_CMD_ESTOP && frame[4] == 1U, "estop frame payload");

    frame_len = UpperProtocol_BuildFrame(UPPER_CMD_LINE_CTRL,
                                         &one_byte_payload,
                                         ROBOT_LINK_PROTOCOL_LINE_CTRL_PAYLOAD_LEN,
                                         frame,
                                         (uint16_t)sizeof(frame));
    require_int(frame_len == 6U, "line frame length");
    require_int(frame[3] == UPPER_CMD_LINE_CTRL && frame[4] == 1U, "line frame payload");
}

static void test_status_v2_payload_layout_and_saturation(void)
{
    upper_status_payload_t status                                          = {0};
    uint8_t                payload[ROBOT_LINK_PROTOCOL_STATUS_PAYLOAD_LEN] = {0};
    uint8_t                payload_len;

    require_int(ROBOT_LINK_PROTOCOL_STATUS_PAYLOAD_LEN == 65U, "status v2 payload length");
    require_int(ROBOT_LINK_PROTOCOL_MAX_PAYLOAD >= ROBOT_LINK_PROTOCOL_STATUS_PAYLOAD_LEN,
                "status v2 fits max payload");

    status.battery_voltage = 12.345f;
    status.status_flags = UPPER_STATUS_FLAG_ESTOP | UPPER_STATUS_FLAG_LINE_ENABLED | UPPER_STATUS_FLAG_SPEED_VALID_ALL;
    status.control_source           = CONTROL_SOURCE_ESP12F;
    status.motor_enabled_mask       = 0x0FU;
    status.error_flags              = 0x01020304UL;
    status.latched_error_flags      = 0xA0B0C0D0UL;
    status.motor_speed_mps[0]       = 0.123f;
    status.motor_speed_mps[1]       = -0.456f;
    status.motor_speed_mps[2]       = 40.0f;
    status.motor_speed_mps[3]       = -40.0f;
    status.encoder_count[0]         = 123;
    status.encoder_count[1]         = -456;
    status.encoder_count[2]         = 2147483647;
    status.encoder_count[3]         = (int32_t)0x80000000UL;
    status.motor_current_a[0]       = 0.789f;
    status.motor_current_a[1]       = -1.0f;
    status.motor_current_a[2]       = 80.0f;
    status.motor_current_a[3]       = 1.234f;
    status.motor_target_mps[0]      = 0.111f;
    status.motor_target_mps[1]      = -0.222f;
    status.motor_target_mps[2]      = 40.0f;
    status.motor_target_mps[3]      = -40.0f;
    status.motor_output_permille[0] = 900;
    status.motor_output_permille[1] = -900;
    status.motor_output_permille[2] = 0;
    status.motor_output_permille[3] = 321;
    status.motor_speed_valid_mask   = 0x0BU;
    status.encoder_anomaly_mask     = 0x05U;
    status.comm_health_flags        = UPPER_COMM_HEALTH_CRC_ERR | UPPER_COMM_HEALTH_TX_DROP;

    payload_len = UpperProtocol_BuildStatusPayload(&status, payload, (uint8_t)sizeof(payload));
    require_int(payload_len == ROBOT_LINK_PROTOCOL_STATUS_PAYLOAD_LEN, "status v2 build length");
    require_int(payload[0] == ROBOT_LINK_PROTOCOL_VERSION, "status v2 version");
    require_int(payload[1] == status.status_flags, "status flags");
    require_int(payload[2] == CONTROL_SOURCE_ESP12F, "control source");
    require_int(payload[3] == 0x0FU, "enabled mask");
    require_int(read_u32_le(&payload[4]) == 0x01020304UL, "error flags");
    require_int(read_u32_le(&payload[8]) == 0xA0B0C0D0UL, "latched flags");
    require_int(read_u16_le(&payload[12]) == 12345U, "battery mv");
    require_int(read_i16_le(&payload[14]) == 123, "m1 speed mmps");
    require_int(read_i16_le(&payload[16]) == -456, "m2 speed mmps");
    require_int(read_i16_le(&payload[18]) == 32767, "m3 speed saturates high");
    require_int(read_i16_le(&payload[20]) == -32768, "m4 speed saturates low");
    require_int(read_i32_le(&payload[22]) == 123, "m1 encoder");
    require_int(read_i32_le(&payload[26]) == -456, "m2 encoder");
    require_int(read_i32_le(&payload[30]) == 2147483647, "m3 encoder");
    require_int(read_i32_le(&payload[34]) == (int32_t)0x80000000UL, "m4 encoder");
    require_int(read_u16_le(&payload[38]) == 789U, "m1 current ma");
    require_int(read_u16_le(&payload[40]) == 0U, "negative current saturates low");
    require_int(read_u16_le(&payload[42]) == 65535U, "current saturates high");
    require_int(read_u16_le(&payload[44]) == 1234U, "m4 current ma");
    require_int(read_i16_le(&payload[46]) == 111, "m1 target mmps");
    require_int(read_i16_le(&payload[48]) == -222, "m2 target mmps");
    require_int(read_i16_le(&payload[50]) == 32767, "m3 target saturates high");
    require_int(read_i16_le(&payload[52]) == -32768, "m4 target saturates low");
    require_int(read_i16_le(&payload[54]) == 900, "m1 pwm");
    require_int(read_i16_le(&payload[56]) == -900, "m2 pwm");
    require_int(read_i16_le(&payload[58]) == 0, "m3 pwm");
    require_int(read_i16_le(&payload[60]) == 321, "m4 pwm");
    require_int(payload[62] == 0x0BU, "speed valid mask");
    require_int(payload[63] == 0x05U, "encoder anomaly mask");
    require_int(payload[64] == (UPPER_COMM_HEALTH_CRC_ERR | UPPER_COMM_HEALTH_TX_DROP), "comm health flags");
}

static void test_imu_status_payload_extended_layout(void)
{
    upper_imu_status_payload_t imu                                                 = {0};
    uint8_t                    payload[ROBOT_LINK_PROTOCOL_IMU_STATUS_PAYLOAD_LEN] = {0};
    uint8_t                    payload_len;

    require_int(ROBOT_LINK_PROTOCOL_IMU_STATUS_PAYLOAD_LEN == 99U, "imu payload extended length");
    require_int(ROBOT_LINK_PROTOCOL_MAX_PAYLOAD >= ROBOT_LINK_PROTOCOL_IMU_STATUS_PAYLOAD_LEN,
                "imu payload fits max payload");

    imu.accel_g[0]            = 1.0f;
    imu.gyro_corrected_dps[2] = 3.0f;
    imu.euler_deg[2]          = 90.0f;
    imu.quaternion[0]         = 1.0f;
    imu.quaternion[3]         = 0.5f;
    imu.timestamp_ms          = 1234UL;
    imu.sensor_time           = 0x00302010UL;
    imu.sample_count          = 42UL;
    imu.quality_flags         = STATE_ESTIMATION_IMU_QUALITY_FIFO_OVERFLOW | STATE_ESTIMATION_IMU_QUALITY_ACCEL_ANOMALY;
    imu.quality_counters[0]   = 1UL;
    imu.quality_counters[6]   = 7UL;
    imu.status_flags          = UPPER_IMU_FLAG_ONLINE | UPPER_IMU_FLAG_SENSOR_TIME;
    imu.temperature_c         = 25;

    payload_len = UpperProtocol_BuildImuStatusPayload(&imu, payload, (uint8_t)sizeof(payload));
    require_int(payload_len == ROBOT_LINK_PROTOCOL_IMU_STATUS_PAYLOAD_LEN, "imu extended build length");
    require_int(payload[0] == ROBOT_LINK_PROTOCOL_VERSION, "imu version");
    require_close(read_float_le(&payload[1]), 1.0f, 0.0001f, "imu accel body x");
    require_close(read_float_le(&payload[33]), 90.0f, 0.0001f, "imu yaw");
    require_close(read_float_le(&payload[37]), 1.0f, 0.0001f, "imu quaternion w");
    require_close(read_float_le(&payload[49]), 0.5f, 0.0001f, "imu quaternion z");
    require_int(read_u32_le(&payload[53]) == 1234UL, "imu host timestamp");
    require_int(read_u32_le(&payload[57]) == 0x00302010UL, "imu sensor time");
    require_int(read_u32_le(&payload[61]) == 42UL, "imu sample count");
    require_int(read_u32_le(&payload[65])
                    == (STATE_ESTIMATION_IMU_QUALITY_FIFO_OVERFLOW | STATE_ESTIMATION_IMU_QUALITY_ACCEL_ANOMALY),
                "imu quality flags");
    require_int(read_u32_le(&payload[69]) == 1UL, "imu first quality counter");
    require_int(read_u32_le(&payload[93]) == 7UL, "imu last quality counter");
    require_int(payload[97] == (UPPER_IMU_FLAG_ONLINE | UPPER_IMU_FLAG_SENSOR_TIME), "imu status flags");
    require_int((int8_t)payload[98] == 25, "imu temperature");
    imu.temperature_c = -41;
    require_int(UpperProtocol_BuildImuStatusPayload(&imu, payload, sizeof(payload))
                    == ROBOT_LINK_PROTOCOL_IMU_STATUS_PAYLOAD_LEN,
                "negative temperature vector builds");
    require_int((int8_t)payload[98] == -41, "imu negative temperature is direct celsius");
    imu.temperature_c = 87;
    require_int(UpperProtocol_BuildImuStatusPayload(&imu, payload, sizeof(payload))
                    == ROBOT_LINK_PROTOCOL_IMU_STATUS_PAYLOAD_LEN,
                "positive temperature vector builds");
    require_int((int8_t)payload[98] == 87, "imu positive temperature is direct celsius");
}

static void test_diagnostic_payload_layout(void)
{
    upper_diagnostic_payload_t diagnostic                                          = {0};
    uint8_t                    payload[ROBOT_LINK_PROTOCOL_DIAGNOSTIC_PAYLOAD_LEN] = {0xA5U};
    uint8_t                    payload_len;

    require_int(ROBOT_LINK_PROTOCOL_DIAGNOSTIC_PAYLOAD_LEN == 28U, "diagnostic payload fixed length");
    diagnostic.post_done                = 1U;
    diagnostic.imu_status_flags         = UPPER_IMU_FLAG_ONLINE | UPPER_IMU_FLAG_CALIBRATED;
    diagnostic.post_error_flags         = 0x01020304UL;
    diagnostic.adc_invalid_reason_flags = 0x11223344UL;
    diagnostic.task_timeout_mask        = (uint16_t)(1U << SYSTEM_MONITORING_TASK_IMU);
    diagnostic.imu_quality_flags        = 0x55667788UL;
    diagnostic.reset_reason_flags       = 0xA1B2C3D4UL;
    diagnostic.uptime_ms                = 0x10203040UL;

    payload_len = UpperProtocol_BuildDiagnosticPayload(&diagnostic, payload, (uint8_t)sizeof(payload));
    require_int(payload_len == ROBOT_LINK_PROTOCOL_DIAGNOSTIC_PAYLOAD_LEN, "diagnostic build length");
    require_int(payload[0] == ROBOT_LINK_PROTOCOL_VERSION, "diagnostic protocol version");
    require_int(payload[1] == ROBOT_LINK_PROTOCOL_DIAGNOSTIC_SCHEMA_VERSION, "diagnostic schema version");
    require_int(payload[2] == 1U, "diagnostic post done");
    require_int(payload[3] == diagnostic.imu_status_flags, "diagnostic imu status");
    require_int(read_u32_le(&payload[4]) == diagnostic.post_error_flags, "diagnostic post flags");
    require_int(read_u32_le(&payload[8]) == diagnostic.adc_invalid_reason_flags, "diagnostic adc invalid reasons");
    require_int(read_u16_le(&payload[12]) == diagnostic.task_timeout_mask, "diagnostic task timeout mask");
    require_int(read_u16_le(&payload[14]) == 0U, "diagnostic reserved field is zero");
    require_int(read_u32_le(&payload[16]) == diagnostic.imu_quality_flags, "diagnostic imu quality");
    require_int(read_u32_le(&payload[20]) == diagnostic.reset_reason_flags, "diagnostic reset reasons");
    require_int(read_u32_le(&payload[24]) == diagnostic.uptime_ms, "diagnostic uptime");
}

static void test_reset_reason_captures_boot_flags(void)
{
    SystemMonitoring_CaptureResetReason(0xA5A55A5AUL);
    require_int(SystemMonitoring_GetResetReason() == 0xA5A55A5AUL, "reset reason preserves startup snapshot");
}

static void test_control_priority_timeout_and_reject_stop(void)
{
    chassis_cmd_t cmd      = {0};
    chassis_cmd_t snapshot = {0};

    ControlService_Init();
    fake_tick = 100U;

    cmd = (chassis_cmd_t){.linear_x     = 0.1f,
                          .angular_z    = 0.0f,
                          .enable       = 1U,
                          .source       = CONTROL_SOURCE_DEBUG,
                          .timestamp_ms = 100U};
    require_int(ControlService_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "debug command accepted");
    cmd.source = CONTROL_SOURCE_ESP12F;
    require_int(ControlService_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "esp command accepted");
    cmd.source = CONTROL_SOURCE_PS2;
    require_int(ControlService_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "ps2 command accepted");
    cmd.source   = CONTROL_SOURCE_UPPER;
    cmd.linear_x = 0.2f;
    require_int(ControlService_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "upper command accepted");

    require_int(ControlService_GetCommand(&snapshot, 120U) != 0U, "command available");
    require_int(snapshot.source == CONTROL_SOURCE_UPPER, "upper has highest priority");
    require_close(snapshot.linear_x, 0.2f, 0.0001f, "upper payload wins");

    ControlService_ClearSource(CONTROL_SOURCE_UPPER);
    require_int(ControlService_GetCommand(&snapshot, 120U) != 0U, "fallback command available");
    require_int(snapshot.source == CONTROL_SOURCE_PS2, "ps2 fallback priority");

    /* Clear remaining sources, then verify timeout for PS2 specifically */
    ControlService_ClearSource(CONTROL_SOURCE_DEBUG);
    ControlService_ClearSource(CONTROL_SOURCE_ESP12F);
    require_int(ControlService_GetCommand(&snapshot, 601U) == 0U, "command timeout");

    cmd = (chassis_cmd_t){.linear_x     = 0.1f,
                          .angular_z    = 0.0f,
                          .enable       = 1U,
                          .source       = CONTROL_SOURCE_PS2,
                          .timestamp_ms = 700U};
    ControlService_ClearSource(CONTROL_SOURCE_DEBUG);
    require_int(ControlService_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "ps2 reset accepted");
    cmd.linear_x = NAN;
    require_int(ControlService_SetCommand(&cmd) == CONTROL_COMMAND_REJECTED_AND_STOPPED, "nan reject and stop");
    require_int(ControlService_GetCommand(&snapshot, 701U) == 0U, "reject clears source");
}

static void test_control_service_uses_runtime_limits(void)
{
    chassis_cmd_t cmd = {
        .linear_x     = 1.0f,
        .angular_z    = 2.0f,
        .enable       = 1U,
        .source       = CONTROL_SOURCE_DEBUG,
        .timestamp_ms = 100U,
    };
    chassis_cmd_t snapshot;
    param_model_t params;

    ParamService_Defaults(&params);
    params.max_linear_mps  = 0.2f;
    params.max_angular_rps = 0.5f;
    require_int(ParamService_Set(&params) != 0U, "runtime limits accepted");
    ControlService_Init();

    require_int(ControlService_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "runtime-limited command accepted");
    require_int(ControlService_GetCommand(&snapshot, 100U) != 0U, "runtime-limited command available");
    require_close(snapshot.linear_x, 0.2f, 0.0001f, "runtime linear limit applied");
    require_close(snapshot.angular_z, 0.5f, 0.0001f, "runtime angular limit applied");
    ParamService_SetDefaults();
}

static void test_control_service_maintenance_rejects_commands(void)
{
    chassis_cmd_t cmd = {
        .linear_x     = 0.1f,
        .angular_z    = 0.0f,
        .enable       = 1U,
        .source       = CONTROL_SOURCE_DEBUG,
        .timestamp_ms = 200U,
    };
    chassis_cmd_t snapshot;
    uint32_t      revoke_generation;

    ControlService_Init();
    revoke_generation = ControlService_GetMotionRevokeGeneration();
    require_int(ControlService_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "command exists before maintenance");
    require_int(ControlService_BeginMaintenance() != 0U, "maintenance lock acquired");
    require_int(ControlService_GetMotionRevokeGeneration() == revoke_generation + 1U,
                "maintenance revokes persistent motion producers");
    require_int(ControlService_IsMaintenanceLocked() != 0U, "maintenance lock visible");
    require_int(ControlService_BeginMaintenance() == 0U, "nested maintenance lock rejected");
    require_int(ControlService_IsMaintenanceLocked() != 0U, "rejected nested begin preserves original lock");
    require_int(ControlService_SetCommand(&cmd) == CONTROL_COMMAND_REJECTED, "maintenance rejects commands");
    require_int(ControlService_GetCommand(&snapshot, 200U) == 0U, "maintenance exposes no active command");
    ControlService_EndMaintenance();
    require_int(ControlService_IsMaintenanceLocked() == 0U, "maintenance lock released");
    require_int(ControlService_GetCommand(&snapshot, 200U) == 0U,
                "released maintenance does not restore stale command");
}

static void test_stale_motion_generation_cannot_submit_after_safety_window(void)
{
    chassis_cmd_t cmd = {
        .linear_x     = 0.1f,
        .angular_z    = 0.0f,
        .enable       = 1U,
        .source       = CONTROL_SOURCE_DEBUG,
        .timestamp_ms = 210U,
    };
    uint32_t stale_generation;

    ControlService_Init();
    stale_generation = ControlService_GetMotionRevokeGeneration();
    require_int(ControlService_BeginMaintenance() != 0U, "maintenance starts safety window");
    ControlService_EndMaintenance();
    require_int(ControlService_SetCommandForGeneration(&cmd, stale_generation) == CONTROL_COMMAND_REJECTED,
                "stale producer token cannot submit after maintenance");
    require_int(ControlService_SetCommandForGeneration(&cmd, ControlService_GetMotionRevokeGeneration())
                    == CONTROL_COMMAND_ACCEPTED,
                "current producer token accepts a new command");
}

static void test_control_stop_recovery_requires_new_command(void)
{
    chassis_cmd_t cmd      = {.linear_x     = 0.2f,
                              .angular_z    = 0.0f,
                              .enable       = 1U,
                              .source       = CONTROL_SOURCE_UPPER,
                              .timestamp_ms = 100U};
    chassis_cmd_t snapshot = {0};
    uint32_t      revoke_generation;

    ControlService_Init();
    revoke_generation = ControlService_GetMotionRevokeGeneration();
    require_int(ControlService_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "upper before estop");
    ControlService_SetEmergencyStop(1U);
    require_int(ControlService_GetMotionRevokeGeneration() == revoke_generation + 1U,
                "estop revokes persistent motion producers");
    require_int(ControlService_GetCommand(&snapshot, 110U) == 0U, "estop blocks command");
    ControlService_SetEmergencyStop(0U);
    require_int(ControlService_GetCommand(&snapshot, 111U) == 0U, "estop recovery does not revive command");

    cmd.timestamp_ms  = 120U;
    revoke_generation = ControlService_GetMotionRevokeGeneration();
    require_int(ControlService_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "upper before fault");
    ControlService_SetFaultStop(1U);
    require_int(ControlService_GetMotionRevokeGeneration() == revoke_generation + 1U,
                "fault stop revokes persistent motion producers");
    ControlService_SetFaultStop(0U);
    require_int(ControlService_GetCommand(&snapshot, 121U) == 0U, "fault recovery does not revive command");
}

static void test_side_target_distribution(void)
{
    float left  = 0.0f;
    float right = 0.0f;

    DifferentialDriveKinematics_ResolveDifferentialTargets(0.2f, 2.0f, CHASSIS_TRACK_WIDTH_M, &left, &right);
    require_close(left, 0.2f - CHASSIS_TRACK_WIDTH_M, 0.0001f, "left target");
    require_close(right, 0.2f + CHASSIS_TRACK_WIDTH_M, 0.0001f, "right target");
}

static void test_default_motor_layout(void)
{
    require_int(MotorHardwareLayout_MotorEnabled(MOTOR_ID_M1) == 0U, "m1 disabled");
    require_int(MotorHardwareLayout_MotorEnabled(MOTOR_ID_M2) != 0U, "m2 enabled");
    require_int(MotorHardwareLayout_MotorEnabled(MOTOR_ID_M3) != 0U, "m3 enabled");
    require_int(MotorHardwareLayout_MotorEnabled(MOTOR_ID_M4) == 0U, "m4 disabled");
    require_int(MotorHardwareLayout_MotorSide(MOTOR_ID_M1) == MOTOR_SIDE_LEFT, "m1 left");
    require_int(MotorHardwareLayout_MotorSide(MOTOR_ID_M2) == MOTOR_SIDE_LEFT, "m2 left");
    require_int(MotorHardwareLayout_MotorSide(MOTOR_ID_M3) == MOTOR_SIDE_RIGHT, "m3 right");
    require_int(MotorHardwareLayout_MotorSide(MOTOR_ID_M4) == MOTOR_SIDE_RIGHT, "m4 right");
    require_int(MotorHardwareLayout_HasBothSides() != 0U, "both sides enabled");
}

static void test_encoder_wrap_diff(void)
{
    require_int(WheelSpeedEstimator_DiffCount(10U, 65530U, 65535U) == 16, "16-bit forward wrap");
    require_int(WheelSpeedEstimator_DiffCount(65530U, 10U, 65535U) == -16, "16-bit reverse wrap");
    require_int(WheelSpeedEstimator_DiffCount(5U, 0xFFFFFFF0U, 0xFFFFFFFFU) == 21, "32-bit forward wrap");
}

static void test_encoder_speed_window(void)
{
    encoder_speed_window_t window;

    WheelSpeedEstimator_SpeedWindowReset(&window);
    WheelSpeedEstimator_SpeedWindowPush(&window, 10, 10U);
    WheelSpeedEstimator_SpeedWindowPush(&window, 11, 9U);
    WheelSpeedEstimator_SpeedWindowPush(&window, 12, 11U);
    WheelSpeedEstimator_SpeedWindowPush(&window, 13, 10U);
    WheelSpeedEstimator_SpeedWindowPush(&window, 14, 10U);
    require_int(window.delta_sum == 60, "speed window sums five deltas");
    require_int(window.dt_sum_ms == 50U, "speed window sums actual dt");
    require_int(window.sample_count == CHASSIS_ENCODER_SPEED_WINDOW_SAMPLES, "speed window reaches capacity");

    WheelSpeedEstimator_SpeedWindowPush(&window, -15, 12U);
    require_int(window.delta_sum == 35, "speed window removes oldest delta");
    require_int(window.dt_sum_ms == 52U, "speed window removes oldest dt");

    WheelSpeedEstimator_SpeedWindowReset(&window);
    require_int(window.delta_sum == 0, "speed window reset clears delta");
    require_int(window.dt_sum_ms == 0U, "speed window reset clears dt");
    require_int(window.sample_count == 0U, "speed window reset clears count");
}

static void test_encoder_interval_average_speed(void)
{
    require_close(WheelSpeedEstimator_CountDeltaSpeedMps(800, 500U, 2464.0f, 0.035f),
                  0.142799f,
                  0.00001f,
                  "500ms count interval average speed");
    require_close(WheelSpeedEstimator_CountDeltaSpeedMps(-800, 500U, 2464.0f, 0.035f),
                  -0.142799f,
                  0.00001f,
                  "reverse interval average speed");
    require_close(WheelSpeedEstimator_CountDeltaSpeedMps(800, 0U, 2464.0f, 0.035f),
                  0.0f,
                  0.00001f,
                  "zero interval returns zero");
}

static void test_motor_ph_en_mapper_phase_enable(void)
{
    motor_output_phase_enable_t output;

    output = MotorPhEnMapper_ResolvePhaseEnable(300);
    require_int(output.en_permille == 300, "positive drive keeps PWM magnitude on EN");
    require_int(output.phase_high != 0U, "positive drive sets PH high");

    output = MotorPhEnMapper_ResolvePhaseEnable(-300);
    require_int(output.en_permille == 300, "negative drive keeps PWM magnitude on EN");
    require_int(output.phase_high == 0U, "negative drive sets PH low");

    output = MotorPhEnMapper_ResolvePhaseEnable(0);
    require_int(output.en_permille == 0, "zero drive disables EN");
    require_int(output.phase_high == 0U, "zero drive leaves PH low");
}

static void test_encoder_delta_filter(void)
{
    encoder_speed_window_t window;

    WheelSpeedEstimator_SpeedWindowReset(&window);
    require_int(WheelSpeedEstimator_DeltaAccepted(45, &window, 10U, 2464.0f, 0.035f, 2.5f, 0.45f, 3U) != 0U,
                "filter accepts normal cold sample");
    require_int(WheelSpeedEstimator_DeltaAccepted(400, &window, 10U, 2464.0f, 0.035f, 2.5f, 0.45f, 3U) == 0U,
                "filter rejects impossible speed");
    require_int(WheelSpeedEstimator_DeltaAccepted(-400, &window, 10U, 2464.0f, 0.035f, 2.5f, 0.45f, 3U) == 0U,
                "filter rejects impossible reverse speed");

    WheelSpeedEstimator_SpeedWindowPush(&window, 45, 10U);
    WheelSpeedEstimator_SpeedWindowPush(&window, 44, 10U);
    WheelSpeedEstimator_SpeedWindowPush(&window, 46, 10U);
    require_int(WheelSpeedEstimator_DeltaAccepted(70, &window, 10U, 2464.0f, 0.035f, 2.5f, 0.45f, 3U) != 0U,
                "filter accepts plausible change");
    require_int(WheelSpeedEstimator_DeltaAccepted(113, &window, 10U, 2464.0f, 0.035f, 2.5f, 0.45f, 3U) == 0U,
                "filter rejects isolated spike");
}

static void test_encoder_reject_streak_rebuilds_window(void)
{
    encoder_speed_window_t window;
    uint8_t                reject_streak = 0U;
    uint16_t               rebuild_count = 0U;

    WheelSpeedEstimator_SpeedWindowReset(&window);
    WheelSpeedEstimator_SpeedWindowPush(&window, 45, 10U);
    WheelSpeedEstimator_SpeedWindowPush(&window, 44, 10U);
    WheelSpeedEstimator_SpeedWindowPush(&window, 46, 10U);

    require_int(WheelSpeedEstimator_RecordDeltaOrRebuild(&window,
                                                         113,
                                                         10U,
                                                         2464.0f,
                                                         0.035f,
                                                         2.5f,
                                                         0.45f,
                                                         3U,
                                                         3U,
                                                         &reject_streak,
                                                         &rebuild_count)
                    == 0U,
                "first spike rejected");
    require_int(window.sample_count == 3U, "first reject keeps old window");
    require_int(reject_streak == 1U, "first reject streak");

    (void)WheelSpeedEstimator_RecordDeltaOrRebuild(&window,
                                                   114,
                                                   10U,
                                                   2464.0f,
                                                   0.035f,
                                                   2.5f,
                                                   0.45f,
                                                   3U,
                                                   3U,
                                                   &reject_streak,
                                                   &rebuild_count);
    require_int(window.sample_count == 3U, "second reject still keeps old window");
    require_int(reject_streak == 2U, "second reject streak");

    require_int(WheelSpeedEstimator_RecordDeltaOrRebuild(&window,
                                                         115,
                                                         10U,
                                                         2464.0f,
                                                         0.035f,
                                                         2.5f,
                                                         0.45f,
                                                         3U,
                                                         3U,
                                                         &reject_streak,
                                                         &rebuild_count)
                    != 0U,
                "third reject rebuilds with current delta");
    require_int(window.sample_count == 1U, "rebuild starts new window");
    require_int(window.delta_sum == 115, "rebuild captures current delta");
    require_int(reject_streak == 0U, "rebuild clears reject streak");
    require_int(rebuild_count == 1U, "rebuild count increments");
}

static void test_task_timing_next_wake(void)
{
    const system_monitoring_config_t config = {
        .task_timeout_ms = {80U, 40U, 40U, 80U, 40U, 40U, 80U, 200U, 400U},
    };
    system_monitoring_task_health_t health;
    uint8_t                         missed = 0U;
    uint32_t                        next   = SystemMonitoring_NextWake(100U, 105U, 10U, &missed);

    require_int(next == 110U, "periodic next wake");
    require_int(missed == 0U, "periodic no miss");

    next = SystemMonitoring_NextWake(110U, 125U, 10U, &missed);
    require_int(next == 135U, "miss realigns to now plus period");
    require_int(missed == 1U, "miss detected");

    require_int(SystemMonitoring_Init(&config, 0UL) != 0U, "system monitor config accepted");
    SystemMonitoring_Heartbeat(SYSTEM_MONITORING_TASK_HOST, 100U);
    SystemMonitoring_UpdateTimeouts(141U);
    (void)SystemMonitoring_GetTaskHealth(&health);
    require_int(health.timeout_count[SYSTEM_MONITORING_TASK_HOST] == 1U, "host heartbeat timeout counted");
    require_int(health.timed_out[SYSTEM_MONITORING_TASK_HOST] != 0U, "host timeout state set");
    SystemMonitoring_UpdateTimeouts(160U);
    (void)SystemMonitoring_GetTaskHealth(&health);
    require_int(health.timeout_count[SYSTEM_MONITORING_TASK_HOST] == 1U, "timeout counted once until recovery");
    SystemMonitoring_Heartbeat(SYSTEM_MONITORING_TASK_HOST, 170U);
    (void)SystemMonitoring_GetTaskHealth(&health);
    require_int(health.timed_out[SYSTEM_MONITORING_TASK_HOST] == 0U, "heartbeat clears timeout state");

    SystemMonitoring_ResetTaskHealth();
    SystemMonitoring_Heartbeat(SYSTEM_MONITORING_TASK_IMU, 100U);
    SystemMonitoring_UpdateTimeouts(181U);
    require_int(SystemMonitoring_GetTimeoutMask() == (uint16_t)(1U << SYSTEM_MONITORING_TASK_IMU),
                "only imu timeout bit is reported");
    SystemMonitoring_Heartbeat(SYSTEM_MONITORING_TASK_IMU, 182U);
    require_int(SystemMonitoring_GetTimeoutMask() == 0U, "imu heartbeat clears only timeout mask bit");
}

static void test_imu_state_contract(void)
{
    state_estimation_imu_status_t state = {0};

    require_int(CHASSIS_IMU_PERIOD_MS == 10U, "imu task period matches 100Hz ODR");
    require_int((sizeof(state.gyro_raw) / sizeof(state.gyro_raw[0])) == 3U, "gyro raw axis count");
    require_int((sizeof(state.gyro_corrected_dps) / sizeof(state.gyro_corrected_dps[0])) == 3U,
                "gyro corrected axis count");
    require_int((sizeof(state.gyro_filtered_dps) / sizeof(state.gyro_filtered_dps[0])) == 3U,
                "gyro filtered axis count");
    require_int((sizeof(state.gyro_dps) / sizeof(state.gyro_dps[0])) == 3U, "gyro compatibility axis count");
    require_int(sizeof(state.roll_deg) == sizeof(float), "imu roll field");
    require_int(sizeof(state.pitch_deg) == sizeof(float), "imu pitch field");
    require_int(sizeof(state.yaw_deg) == sizeof(float), "imu yaw field");
    require_int((sizeof(state.quaternion) / sizeof(state.quaternion[0])) == 4U, "imu quaternion field");
    require_int((sizeof(state.body_accel_g) / sizeof(state.body_accel_g[0])) == 3U, "imu body accel field");
    require_int((sizeof(state.body_gyro_dps) / sizeof(state.body_gyro_dps[0])) == 3U, "imu body gyro field");
    require_int(sizeof(state.quality_flags) == sizeof(uint32_t), "imu quality flags field");
}

/* ────────── L1: PID 控制器纯逻辑测试 ────────── */

static void test_pid_init_and_reset(void)
{
    pid_state_t  pid    = {0};
    pid_params_t params = {.kp = 1.0f, .ki = 0.5f, .kd = 0.1f, .integral_limit = 10.0f, .output_limit = 100.0f};

    PidController_Init(&pid, &params);
    require_int(pid.initialized == 1U, "pid init sets initialized");
    require_close(pid.params.kp, 1.0f, 0.001f, "pid init stores kp");

    pid.integral   = 5.0f;
    pid.prev_error = 3.0f;
    PidController_Reset(&pid);
    require_close(pid.integral, 0.0f, 0.001f, "pid reset clears integral");
    require_close(pid.prev_error, 0.0f, 0.001f, "pid reset clears prev_error");
}

static void test_pid_step_proportional(void)
{
    pid_state_t  pid    = {0};
    pid_params_t params = {.kp = 2.0f, .ki = 0.0f, .kd = 0.0f, .integral_limit = 100.0f, .output_limit = 1000.0f};
    float        output;

    PidController_Init(&pid, &params);
    output = PidController_Step(&pid, 1.0f, 0.5f, 0.01f);
    require_close(output, 1.0f, 0.01f, "pid p-only: kp*error = 2*0.5 = 1.0");

    output = PidController_Step(&pid, 0.0f, 1.0f, 0.01f);
    require_close(output, -2.0f, 0.01f, "pid p-only negative error");
}

static void test_pid_step_integral(void)
{
    pid_state_t  pid    = {0};
    pid_params_t params = {.kp = 0.0f, .ki = 10.0f, .kd = 0.0f, .integral_limit = 5.0f, .output_limit = 1000.0f};
    float        output;

    PidController_Init(&pid, &params);
    /* 连续 5 步，误差 1.0，dt=0.1 → integral = 0.5 → output = 10*0.5 = 5.0 */
    for (int i = 0; i < 5; ++i)
    {
        output = PidController_Step(&pid, 1.0f, 0.0f, 0.1f);
    }
    require_close(output, 5.0f, 0.01f, "pid i-term accumulates");

    /* 继续步进 → integral 限幅 5.0 → output 限幅 50.0 */
    for (int i = 0; i < 100; ++i)
    {
        output = PidController_Step(&pid, 1.0f, 0.0f, 0.1f);
    }
    require_close(output, 50.0f, 0.1f, "pid i-term integral_limit caps accumulation");
}

static void test_pid_step_derivative(void)
{
    pid_state_t  pid    = {0};
    pid_params_t params = {.kp = 0.0f, .ki = 0.0f, .kd = 1.0f, .integral_limit = 100.0f, .output_limit = 1000.0f};
    float        output;

    PidController_Init(&pid, &params);
    /* 第一步: error=1.0, prev_error=0 → d = (1.0-0)/0.01 = 100 */
    output = PidController_Step(&pid, 1.0f, 0.0f, 0.01f);
    require_close(output, 100.0f, 0.1f, "pid d-term: kd*(error-prev)/dt");

    /* 第二步: error=1.0, prev_error=1.0 → d = 0 */
    output = PidController_Step(&pid, 1.0f, 0.0f, 0.01f);
    require_close(output, 0.0f, 0.01f, "pid d-term zero when error constant");
}

static void test_pid_output_limit(void)
{
    pid_state_t  pid    = {0};
    pid_params_t params = {.kp = 1000.0f, .ki = 0.0f, .kd = 0.0f, .integral_limit = 100.0f, .output_limit = 500.0f};
    float        output;

    PidController_Init(&pid, &params);
    output = PidController_Step(&pid, 1.0f, 0.0f, 0.01f);
    require_close(output, 500.0f, 0.1f, "pid output clamped to +limit");

    output = PidController_Step(&pid, -1.0f, 0.0f, 0.01f);
    require_close(output, -500.0f, 0.1f, "pid output clamped to -limit");
}

static void test_pid_zero_dt_returns_zero(void)
{
    pid_state_t  pid    = {0};
    pid_params_t params = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f, .integral_limit = 10.0f, .output_limit = 100.0f};
    float        output;

    PidController_Init(&pid, &params);
    output = PidController_Step(&pid, 1.0f, 0.0f, 0.0f);
    require_close(output, 0.0f, 0.001f, "pid zero dt returns 0");

    output = PidController_Step(&pid, 1.0f, 0.0f, -0.01f);
    require_close(output, 0.0f, 0.001f, "pid negative dt returns 0");
}

static void test_pid_direction_reversal_resets(void)
{
    pid_state_t  pid    = {0};
    pid_params_t params = {.kp = 1.0f, .ki = 1.0f, .kd = 0.0f, .integral_limit = 100.0f, .output_limit = 1000.0f};

    PidController_Init(&pid, &params);
    /* 积累正向误差 */
    (void)PidController_Step(&pid, 1.0f, 0.0f, 0.1f);
    (void)PidController_Step(&pid, 1.0f, 0.0f, 0.1f);
    require_int(pid.integral > 0.0f, "pid integral positive after positive error");

    /* 反向 → PID reset（由 ChassisService_StepMotorPid 调用） */
    PidController_Reset(&pid);
    require_close(pid.integral, 0.0f, 0.001f, "pid integral cleared after direction reversal");
}

static void test_pid_conditional_anti_windup(void)
{
    pid_state_t  pid    = {0};
    pid_params_t params = {.kp = 0.0f, .ki = 1.0f, .kd = 0.0f, .integral_limit = 100.0f, .output_limit = 100.0f};

    PidController_Init(&pid, &params);
    (void)PidController_Step(&pid, 1.0f, 0.0f, 1.0f);
    require_close(pid.integral, 1.0f, 0.001f, "pid baseline integral");
    (void)PidController_StepLimited(&pid, 1.0f, 0.0f, 1.0f, 1);
    require_close(pid.integral, 1.0f, 0.001f, "positive actuator limit freezes positive error");
    (void)PidController_StepLimited(&pid, -1.0f, 0.0f, 1.0f, 1);
    require_close(pid.integral, 0.0f, 0.001f, "opposite error unwinds limited integral");

    params.kp           = 2.0f;
    params.output_limit = 1.0f;
    PidController_Init(&pid, &params);
    (void)PidController_StepLimited(&pid, 1.0f, 0.0f, 1.0f, 0);
    require_close(pid.integral, 0.0f, 0.001f, "internal positive saturation freezes positive integral");
}

static void test_pid_external_output_bounds(void)
{
    pid_state_t  pid    = {0};
    pid_params_t params = {.kp = 0.0f, .ki = 1.0f, .kd = 0.0f, .integral_limit = 100.0f, .output_limit = 500.0f};
    float        output;

    PidController_Init(&pid, &params);
    (void)PidController_Step(&pid, 1.0f, 0.0f, 1.0f);
    output = PidController_StepBounded(&pid, 1.0f, 0.0f, 1.0f, 0, -10.0f, 1.0f);
    require_close(output, 1.0f, 0.001f, "external positive bound clamps correction");
    require_close(pid.integral, 1.0f, 0.001f, "external positive saturation freezes integral");

    output = PidController_StepBounded(&pid, -1.0f, 0.0f, 1.0f, 0, -10.0f, 1.0f);
    require_close(output, 0.0f, 0.001f, "opposite error leaves positive saturation");
    require_close(pid.integral, 0.0f, 0.001f, "opposite error unwinds externally limited integral");

    PidController_Reset(&pid);
    (void)PidController_Step(&pid, -1.0f, 0.0f, 1.0f);
    output = PidController_StepBounded(&pid, -1.0f, 0.0f, 1.0f, 0, -1.0f, 10.0f);
    require_close(output, -1.0f, 0.001f, "external negative bound is symmetric");
    require_close(pid.integral, -1.0f, 0.001f, "external negative saturation freezes integral");

    params.kp = 1000.0f;
    params.ki = 0.0f;
    PidController_Init(&pid, &params);
    output = PidController_StepBounded(&pid, 1.0f, 0.0f, 1.0f, 0, -900.0f, 900.0f);
    require_close(output, 500.0f, 0.001f, "internal PID limit intersects wider external bound");
}

static void test_control_dt_uses_measured_period_and_rejects_long_gap(void)
{
    uint32_t last_step_ms = 0U;
    uint8_t  initialized  = 0U;
    float    dt_s         = 0.0f;

    require_int(DifferentialDriveKinematics_ControlDt(100U, &last_step_ms, &initialized, &dt_s) == 1U,
                "first control step accepted");
    require_close(dt_s, 0.010f, 0.0001f, "first control step uses 10ms");
    require_int(DifferentialDriveKinematics_ControlDt(105U, &last_step_ms, &initialized, &dt_s) == 1U,
                "5ms control step accepted");
    require_close(dt_s, 0.005f, 0.0001f, "5ms measured dt");
    require_int(DifferentialDriveKinematics_ControlDt(115U, &last_step_ms, &initialized, &dt_s) == 1U,
                "10ms control step accepted");
    require_close(dt_s, 0.010f, 0.0001f, "10ms measured dt");
    require_int(DifferentialDriveKinematics_ControlDt(145U, &last_step_ms, &initialized, &dt_s) == 1U,
                "30ms control step accepted");
    require_close(dt_s, 0.030f, 0.0001f, "30ms measured dt");
    require_int(DifferentialDriveKinematics_ControlDt(245U, &last_step_ms, &initialized, &dt_s) == 1U,
                "100ms control step accepted");
    require_close(dt_s, 0.100f, 0.0001f, "100ms measured dt");
    require_int(DifferentialDriveKinematics_ControlDt(346U, &last_step_ms, &initialized, &dt_s) == 0U,
                "gap above 100ms is rejected");
}

/* ────────── L2: differential-drive kinematics tests ────────── */

static void test_chassis_math_differential(void)
{
    float left, right;

    /* 纯直线 */
    DifferentialDriveKinematics_ResolveDifferentialTargets(0.2f, 0.0f, CHASSIS_TRACK_WIDTH_M, &left, &right);
    require_close(left, 0.2f, 0.001f, "straight: left = linear_x");
    require_close(right, 0.2f, 0.001f, "straight: right = linear_x");

    /* 原地旋转 */
    DifferentialDriveKinematics_ResolveDifferentialTargets(0.0f, 2.0f, CHASSIS_TRACK_WIDTH_M, &left, &right);
    require_close(left, -right, 0.001f, "spin: left = -right");

    /* 混合：正 angular_z = CCW = 右轮更快 */
    DifferentialDriveKinematics_ResolveDifferentialTargets(0.3f, 1.0f, CHASSIS_TRACK_WIDTH_M, &left, &right);
    require_int(right > left, "turn: positive angular_z → right > left");
}

int main(void)
{
    test_protocol_frame_and_velocity();
    test_status_v2_payload_layout_and_saturation();
    test_imu_status_payload_extended_layout();
    test_diagnostic_payload_layout();
    test_reset_reason_captures_boot_flags();
    test_control_priority_timeout_and_reject_stop();
    test_control_service_uses_runtime_limits();
    test_control_service_maintenance_rejects_commands();
    test_stale_motion_generation_cannot_submit_after_safety_window();
    test_control_stop_recovery_requires_new_command();
    test_side_target_distribution();
    test_default_motor_layout();
    test_encoder_wrap_diff();
    test_encoder_speed_window();
    test_encoder_interval_average_speed();
    test_motor_ph_en_mapper_phase_enable();
    test_encoder_delta_filter();
    test_encoder_reject_streak_rebuilds_window();
    test_task_timing_next_wake();
    test_imu_state_contract();
    test_pid_init_and_reset();
    test_pid_step_proportional();
    test_pid_step_integral();
    test_pid_step_derivative();
    test_pid_output_limit();
    test_pid_zero_dt_returns_zero();
    test_pid_direction_reversal_resets();
    test_pid_conditional_anti_windup();
    test_pid_external_output_bounds();
    test_control_dt_uses_measured_period_and_rejects_long_gap();
    test_chassis_math_differential();
    (void)printf("PASS: f407_v2 host tests\n");
    return 0;
}
