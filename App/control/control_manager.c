#include "control_manager.h"

#include "chassis_config.h"
#include "cmsis_os2.h"
#include "main.h"

static chassis_cmd_t source_cmds[CONTROL_SOURCE_LINE + 1U];
static uint8_t emergency_stop;
static uint8_t fault_stop;

static uint8_t ControlManager_IsFiniteFloat(float value)
{
  const float max_float = 3.402823466e+38f;
  return (value == value && value <= max_float && value >= -max_float) ? 1U : 0U;
}

static float ControlManager_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float ControlManager_ClampFloat(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}

static uint8_t ControlManager_KinematicsValid(void)
{
  return (CHASSIS_WHEEL_RADIUS_M > 0.0f && CHASSIS_WHEEL_BASE_M > 0.0f) ? 1U : 0U;
}

void ControlManager_Init(void)
{
  for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
  {
    source_cmds[i] = (chassis_cmd_t){0};
  }
  emergency_stop = 0U;
  fault_stop = 0U;
}

void ControlManager_ClearCommand(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
  {
    source_cmds[i] = (chassis_cmd_t){0};
  }
  __set_PRIMASK(primask);
}

void ControlManager_ClearSource(uint8_t source)
{
  uint32_t primask;

  if (source == CONTROL_SOURCE_NONE || source > CONTROL_SOURCE_LINE)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  source_cmds[source] = (chassis_cmd_t){0};
  __set_PRIMASK(primask);
}

control_command_result_t ControlManager_SetCommand(const chassis_cmd_t *cmd)
{
  if (cmd != 0)
  {
    chassis_cmd_t sanitized = *cmd;
    uint32_t primask;

    if (ControlManager_IsEmergencyStop() != 0U || ControlManager_IsFaultStop() != 0U)
    {
      return CONTROL_COMMAND_REJECTED;
    }
    if (sanitized.source == CONTROL_SOURCE_NONE || sanitized.source > CONTROL_SOURCE_LINE)
    {
      return CONTROL_COMMAND_REJECTED;
    }

    if (ControlManager_IsFiniteFloat(sanitized.linear_x) == 0U ||
        ControlManager_IsFiniteFloat(sanitized.angular_z) == 0U)
    {
      ControlManager_ClearSource(sanitized.source);
      return CONTROL_COMMAND_REJECTED_AND_STOPPED;
    }

    if (sanitized.enable == 0U)
    {
      ControlManager_ClearSource(sanitized.source);
      return CONTROL_COMMAND_REJECTED_AND_STOPPED;
    }

    sanitized.linear_x = ControlManager_ClampFloat(sanitized.linear_x, CHASSIS_MAX_LINEAR_MPS);
    if (ControlManager_KinematicsValid() == 0U &&
        ControlManager_AbsFloat(sanitized.angular_z) > CHASSIS_ANGULAR_EPSILON_RPS)
    {
      ControlManager_ClearSource(sanitized.source);
      return CONTROL_COMMAND_REJECTED_AND_STOPPED;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (emergency_stop != 0U || fault_stop != 0U)
    {
      __set_PRIMASK(primask);
      return CONTROL_COMMAND_REJECTED;
    }
    source_cmds[sanitized.source] = sanitized;
    __set_PRIMASK(primask);
    return CONTROL_COMMAND_ACCEPTED;
  }

  return CONTROL_COMMAND_REJECTED;
}

void ControlManager_SetEmergencyStop(uint8_t enabled)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  emergency_stop = (enabled != 0U) ? 1U : 0U;
  for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
  {
    source_cmds[i] = (chassis_cmd_t){0};
  }
  __set_PRIMASK(primask);
}

void ControlManager_SetFaultStop(uint8_t enabled)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  fault_stop = (enabled != 0U) ? 1U : 0U;
  for (uint8_t i = 0U; i <= CONTROL_SOURCE_LINE; ++i)
  {
    source_cmds[i] = (chassis_cmd_t){0};
  }
  __set_PRIMASK(primask);
}

uint8_t ControlManager_GetCommand(chassis_cmd_t *cmd, uint32_t now_ms)
{
  static const uint8_t source_priority[] = {
    CONTROL_SOURCE_UPPER,
    CONTROL_SOURCE_PS2,
    CONTROL_SOURCE_ESP12F,
    CONTROL_SOURCE_LINE,
    CONTROL_SOURCE_DEBUG,
  };
  uint32_t primask;

  if (cmd != 0)
  {
    *cmd = (chassis_cmd_t){0};
  }
  primask = __get_PRIMASK();
  __disable_irq();
  if (emergency_stop != 0U || fault_stop != 0U)
  {
    __set_PRIMASK(primask);
    return 0U;
  }
  for (uint8_t i = 0U; i < (uint8_t)(sizeof(source_priority) / sizeof(source_priority[0])); ++i)
  {
    chassis_cmd_t snapshot = source_cmds[source_priority[i]];
    uint32_t age_ms = now_ms - snapshot.timestamp_ms;

    if (snapshot.enable != 0U &&
        snapshot.source != CONTROL_SOURCE_NONE &&
        age_ms <= CHASSIS_CMD_TIMEOUT_MS)
    {
      if (cmd != 0)
      {
        *cmd = snapshot;
      }
      __set_PRIMASK(primask);
      return 1U;
    }
  }
  __set_PRIMASK(primask);
  return 0U;
}

uint8_t ControlManager_IsEmergencyStop(void)
{
  uint8_t value;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  value = emergency_stop;
  __set_PRIMASK(primask);
  return value;
}

uint8_t ControlManager_IsFaultStop(void)
{
  uint8_t value;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  value = fault_stop;
  __set_PRIMASK(primask);
  return value;
}

uint8_t ControlManager_GetActiveSource(void)
{
  chassis_cmd_t snapshot;

  if (ControlManager_GetCommand(&snapshot, osKernelGetTickCount()) == 0U)
  {
    return CONTROL_SOURCE_NONE;
  }

  return snapshot.source;
}
