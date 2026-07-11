#include "line_control.h"

#include "control_manager.h"
#include "line_uart.h"

#include <stdio.h>
#include <stdlib.h>

static uint32_t fake_revoke_generation;
static uint32_t submitted_count;
static uint32_t clear_count;
static uint8_t fake_estop;
static uint8_t fake_fault_stop;
static uint8_t fake_maintenance;
static chassis_cmd_t last_command;
static line_sensor_data_t fake_sensor;

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
  return 100U;
}

uint32_t ControlManager_GetMotionRevokeGeneration(void)
{
  return fake_revoke_generation;
}

uint8_t ControlManager_IsEmergencyStop(void) { return fake_estop; }
uint8_t ControlManager_IsFaultStop(void) { return fake_fault_stop; }
uint8_t ControlManager_IsMaintenanceLocked(void) { return fake_maintenance; }

control_command_result_t ControlManager_SetCommand(const chassis_cmd_t *cmd)
{
  last_command = *cmd;
  submitted_count++;
  return CONTROL_COMMAND_ACCEPTED;
}

control_command_result_t ControlManager_SetCommandForGeneration(const chassis_cmd_t *cmd,
                                                                uint32_t expected_generation)
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
  fake_revoke_generation = 10U;
  submitted_count = 0U;
  clear_count = 0U;
  fake_estop = 0U;
  fake_fault_stop = 0U;
  fake_maintenance = 0U;
  last_command = (chassis_cmd_t){0};
  fake_sensor = (line_sensor_data_t){0};
  fake_sensor.valid = 1U;
  fake_sensor.timestamp_ms = 100U;
  fake_sensor.state[3] = 1U;
  fake_sensor.state[4] = 1U;
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
  fake_estop = 1U;
  LineControl_Enable(1U);
  require_int(LineControl_IsEnabled() == 0U, "estop rejects line rearm");

  fake_estop = 0U;
  fake_fault_stop = 1U;
  LineControl_Enable(1U);
  require_int(LineControl_IsEnabled() == 0U, "fault stop rejects line rearm");
}

static void test_safety_generation_revokes_old_line_enable(void)
{
  reset_fake();
  LineControl_Update();
  require_int(submitted_count == 1U && last_command.source == CONTROL_SOURCE_LINE,
              "enabled line submits before safety revocation");

  fake_revoke_generation++;
  LineControl_Update();
  require_int(LineControl_IsEnabled() == 0U, "old line enable is revoked");
  require_int(submitted_count == 1U, "revoked line cannot resubmit stale motion");
  require_int(clear_count != 0U, "revoked line source is cleared");

  LineControl_Enable(1U);
  LineControl_Update();
  require_int(submitted_count == 2U, "new explicit line enable can move again");
}

int main(void)
{
  test_safety_generation_revokes_old_line_enable();
  test_safety_state_rejects_line_rearm();
  (void)printf("PASS: line control safety generation tests\n");
  return 0;
}
