#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "chassis_config.h"
#include "chassis_math.h"
#include "chassis_task_timing.h"
#include "control_manager.h"
#include "encoder_math.h"
#include "upper_protocol.h"

static uint32_t fake_primask;
static uint32_t fake_tick;

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

static void test_protocol_frame_and_velocity(void)
{
  uint8_t frame[UPPER_PROTOCOL_MAX_FRAME] = {0};
  uint8_t payload[UPPER_PROTOCOL_VELOCITY_PAYLOAD_LEN] = {0};
  upper_velocity_payload_t velocity = {0};
  uint16_t frame_len;

  write_float_le(&payload[0], 0.25f);
  write_float_le(&payload[4], -1.5f);
  payload[8] = 1U;
  payload[9] = 2U;

  frame_len = UpperProtocol_BuildFrame(UPPER_CMD_SET_VELOCITY,
                                       payload,
                                       UPPER_PROTOCOL_VELOCITY_PAYLOAD_LEN,
                                       frame,
                                       (uint16_t)sizeof(frame));
  require_int(frame_len == 15U, "velocity frame length");
  require_int(frame[0] == UPPER_PROTOCOL_HEAD_0 && frame[1] == UPPER_PROTOCOL_HEAD_1, "frame header");
  require_int(frame[2] == UPPER_PROTOCOL_CMD_LEN(UPPER_PROTOCOL_VELOCITY_PAYLOAD_LEN), "frame command length");
  require_int(frame[frame_len - 1U] == UpperProtocol_Checksum8(&frame[2], 12U), "frame checksum");
  require_int(UpperProtocol_ParseVelocityPayload(payload, (uint8_t)sizeof(payload), &velocity) != 0U,
              "velocity payload parse");
  require_close(velocity.linear_x, 0.25f, 0.0001f, "velocity linear");
  require_close(velocity.angular_z, -1.5f, 0.0001f, "velocity angular");
  require_int(velocity.enable == 1U && velocity.mode == 2U, "velocity flags");
}

static void test_control_priority_timeout_and_reject_stop(void)
{
  chassis_cmd_t cmd = {0};
  chassis_cmd_t snapshot = {0};

  ControlManager_Init();
  fake_tick = 100U;

  cmd = (chassis_cmd_t){ .linear_x = 0.1f, .angular_z = 0.0f, .enable = 1U, .source = CONTROL_SOURCE_DEBUG, .timestamp_ms = 100U };
  require_int(ControlManager_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "debug command accepted");
  cmd.source = CONTROL_SOURCE_ESP12F;
  require_int(ControlManager_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "esp command accepted");
  cmd.source = CONTROL_SOURCE_PS2;
  require_int(ControlManager_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "ps2 command accepted");
  cmd.source = CONTROL_SOURCE_UPPER;
  cmd.linear_x = 0.2f;
  require_int(ControlManager_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "upper command accepted");

  require_int(ControlManager_GetCommand(&snapshot, 120U) != 0U, "command available");
  require_int(snapshot.source == CONTROL_SOURCE_UPPER, "upper has highest priority");
  require_close(snapshot.linear_x, 0.2f, 0.0001f, "upper payload wins");

  ControlManager_ClearSource(CONTROL_SOURCE_UPPER);
  require_int(ControlManager_GetCommand(&snapshot, 120U) != 0U, "fallback command available");
  require_int(snapshot.source == CONTROL_SOURCE_PS2, "ps2 fallback priority");

  require_int(ControlManager_GetCommand(&snapshot, 601U) == 0U, "command timeout");

  cmd = (chassis_cmd_t){ .linear_x = 0.1f, .angular_z = 0.0f, .enable = 1U, .source = CONTROL_SOURCE_PS2, .timestamp_ms = 700U };
  require_int(ControlManager_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "ps2 reset accepted");
  cmd.linear_x = NAN;
  require_int(ControlManager_SetCommand(&cmd) == CONTROL_COMMAND_REJECTED_AND_STOPPED, "nan reject and stop");
  require_int(ControlManager_GetCommand(&snapshot, 701U) == 0U, "reject clears source");
}

static void test_control_stop_recovery_requires_new_command(void)
{
  chassis_cmd_t cmd = { .linear_x = 0.2f, .angular_z = 0.0f, .enable = 1U, .source = CONTROL_SOURCE_UPPER, .timestamp_ms = 100U };
  chassis_cmd_t snapshot = {0};

  ControlManager_Init();
  require_int(ControlManager_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "upper before estop");
  ControlManager_SetEmergencyStop(1U);
  require_int(ControlManager_GetCommand(&snapshot, 110U) == 0U, "estop blocks command");
  ControlManager_SetEmergencyStop(0U);
  require_int(ControlManager_GetCommand(&snapshot, 111U) == 0U, "estop recovery does not revive command");

  cmd.timestamp_ms = 120U;
  require_int(ControlManager_SetCommand(&cmd) == CONTROL_COMMAND_ACCEPTED, "upper before fault");
  ControlManager_SetFaultStop(1U);
  ControlManager_SetFaultStop(0U);
  require_int(ControlManager_GetCommand(&snapshot, 121U) == 0U, "fault recovery does not revive command");
}

static void test_side_target_distribution(void)
{
  float left = 0.0f;
  float right = 0.0f;

  ChassisMath_ResolveDifferentialTargets(0.2f, 2.0f, CHASSIS_WHEEL_BASE_M, &left, &right);
  require_close(left, 0.022f, 0.0001f, "left target");
  require_close(right, 0.378f, 0.0001f, "right target");
}

static void test_encoder_wrap_diff(void)
{
  require_int(EncoderMath_DiffCount(10U, 65530U, 65535U) == 16, "16-bit forward wrap");
  require_int(EncoderMath_DiffCount(65530U, 10U, 65535U) == -16, "16-bit reverse wrap");
  require_int(EncoderMath_DiffCount(5U, 0xFFFFFFF0U, 0xFFFFFFFFU) == 21, "32-bit forward wrap");
}

static void test_task_timing_next_wake(void)
{
  uint8_t missed = 0U;
  uint32_t next = ChassisTaskTiming_NextWake(100U, 105U, 10U, &missed);

  require_int(next == 110U, "periodic next wake");
  require_int(missed == 0U, "periodic no miss");

  next = ChassisTaskTiming_NextWake(110U, 125U, 10U, &missed);
  require_int(next == 135U, "miss realigns to now plus period");
  require_int(missed == 1U, "miss detected");
}

int main(void)
{
  test_protocol_frame_and_velocity();
  test_control_priority_timeout_and_reject_stop();
  test_control_stop_recovery_requires_new_command();
  test_side_target_distribution();
  test_encoder_wrap_diff();
  test_task_timing_next_wake();
  (void)printf("PASS: f407_v2 host tests\n");
  return 0;
}
