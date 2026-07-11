#include "line_control.h"

#include "chassis_config.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "line_uart.h"

static volatile uint8_t g_line_enabled;
static uint32_t g_line_enable_generation;

static uint8_t LineControl_SafetyActive(void)
{
  return (ControlManager_IsEmergencyStop() != 0U ||
          ControlManager_IsFaultStop() != 0U ||
          ControlManager_IsMaintenanceLocked() != 0U) ? 1U : 0U;
}

static void LineControl_SubmitCommand(float linear_x, float angular_z)
{
  chassis_cmd_t cmd = {
    .linear_x = linear_x,
    .angular_z = angular_z,
    .enable = 1U,
    .source = CONTROL_SOURCE_LINE,
    .timestamp_ms = osKernelGetTickCount(),
  };
  (void)ControlManager_SetCommandForGeneration(&cmd, g_line_enable_generation);
}

static float LineControl_ClampFloat(float value, float limit)
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

void LineControl_Init(void)
{
  g_line_enabled = LINE_DEFAULT_ENABLED;
  g_line_enable_generation = ControlManager_GetMotionRevokeGeneration();
}

void LineControl_Update(void)
{
  line_sensor_data_t sensor;
  uint32_t now_ms;
  uint8_t  i;
  float    sum_positions;
  uint8_t  detected;
  float    position;
  float    error;
  float    angular_z;

  if (g_line_enabled == 0U)
  {
    return;
  }
  if (LineControl_SafetyActive() != 0U ||
      g_line_enable_generation != ControlManager_GetMotionRevokeGeneration())
  {
    LineControl_Enable(0U);
    return;
  }

  (void)LineUart_GetSensorData(&sensor);
  now_ms = osKernelGetTickCount();

  /* 传感器超时检测：基于时间戳判断数据新鲜度 */
  if (sensor.valid == 0U ||
      (sensor.timestamp_ms > 0U &&
       (now_ms - sensor.timestamp_ms) > LINE_SENSOR_TIMEOUT_MS))
  {
    if (sensor.timestamp_ms > 0U)
    {
      ControlManager_ClearSource(CONTROL_SOURCE_LINE);
    }
    return;
  }

  /* 加权平均计算黑线位置 */
  sum_positions = 0.0f;
  detected = 0U;
  for (i = 0U; i < LINE_SENSOR_CHANNELS; ++i)
  {
    if (sensor.state[i] != 0U)
    {
      sum_positions += (float)i;
      detected++;
    }
  }

  if (detected < LINE_DETECT_THRESHOLD_COUNT)
  {
    /* 丢线：清源，允许回退到更低优先级控制源 */
    ControlManager_ClearSource(CONTROL_SOURCE_LINE);
    return;
  }

  position = sum_positions / (float)detected;
  error = position - 3.5f; /* 中心位置：CH4(3) 和 CH5(4) 中间 */
  angular_z = LINE_KP * error;
  angular_z = LineControl_ClampFloat(angular_z, LINE_ANGULAR_MAX_RPS);

  LineControl_SubmitCommand(LINE_SPEED_MPS, angular_z);
}

void LineControl_Enable(uint8_t enable)
{
  if (enable != 0U)
  {
    uint32_t generation = ControlManager_GetMotionRevokeGeneration();

    if (LineControl_SafetyActive() != 0U)
    {
      g_line_enabled = 0U;
      ControlManager_ClearSource(CONTROL_SOURCE_LINE);
      return;
    }
    g_line_enable_generation = generation;
    if (LineControl_SafetyActive() != 0U ||
        generation != ControlManager_GetMotionRevokeGeneration())
    {
      g_line_enabled = 0U;
      ControlManager_ClearSource(CONTROL_SOURCE_LINE);
      return;
    }
    g_line_enabled = 1U;
  }
  else
  {
    g_line_enabled = 0U;
    ControlManager_ClearSource(CONTROL_SOURCE_LINE);
  }
}

uint8_t LineControl_IsEnabled(void)
{
  return g_line_enabled;
}

void LineControl_GetState(line_control_state_t *state)
{
  if (state != 0)
  {
    *state = (line_control_state_t){0};
    state->globally_enabled = g_line_enabled;
  }
}
