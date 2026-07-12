#include "line_control.h"

#include "chassis_config.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "line_uart.h"
#include "param_store.h"

#define LINE_LOST_NONE       0U
#define LINE_LOST_STALE      1U
#define LINE_LOST_NO_CHANNEL 2U

static volatile uint8_t     g_line_enabled;
static uint32_t             g_line_enable_generation;
static line_calibration_t   g_line_calibration;
static uint32_t             g_line_calibration_last_timestamp;
static line_control_state_t g_line_state;
static uint8_t              g_detect_streak;
static uint8_t              g_lost_streak;
static float                g_previous_error;
static uint32_t             g_previous_timestamp_ms;
static uint32_t             g_line_last_processed_timestamp;

static uint8_t LineControl_SafetyActive(void)
{
    return (ControlManager_IsEmergencyStop() != 0U || ControlManager_IsFaultStop() != 0U
            || ControlManager_IsMaintenanceLocked() != 0U)
               ? 1U
               : 0U;
}

static void LineControl_SubmitCommand(float linear_x, float angular_z)
{
    chassis_cmd_t cmd = {
        .linear_x     = linear_x,
        .angular_z    = angular_z,
        .enable       = 1U,
        .source       = CONTROL_SOURCE_LINE,
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
    g_line_enabled           = LINE_DEFAULT_ENABLED;
    g_line_enable_generation = ControlManager_GetMotionRevokeGeneration();
    LineCalibration_Init(&g_line_calibration);
    g_line_calibration_last_timestamp = 0U;
    g_line_state                      = (line_control_state_t){0};
    g_detect_streak                   = 0U;
    g_lost_streak                     = 0U;
    g_previous_error                  = 0.0f;
    g_previous_timestamp_ms           = 0U;
    g_line_last_processed_timestamp   = 0U;
}

void LineControl_Update(void)
{
    line_sensor_data_t sensor;
    uint32_t           now_ms;
    uint8_t            i;
    float              sum_positions;
    uint8_t            detected;
    float              position;
    float              error;
    float              angular_z;
    float              derivative = 0.0f;
    float              speed;
    param_store_t      params;

    (void)ParamStore_GetSnapshot(&params);

    (void)LineUart_GetSensorData(&sensor);
    if (g_line_calibration.collecting != 0U && sensor.valid != 0U
        && sensor.timestamp_ms != g_line_calibration_last_timestamp)
    {
        g_line_state.tracking_active = 0U;
        g_line_state.lost_reason     = LINE_LOST_STALE;
        LineCalibration_Feed(&g_line_calibration, sensor.analog);
        g_line_calibration_last_timestamp = sensor.timestamp_ms;
    }

    if (g_line_enabled == 0U)
    {
        return;
    }
    if (LineControl_SafetyActive() != 0U || g_line_enable_generation != ControlManager_GetMotionRevokeGeneration())
    {
        LineControl_Enable(0U);
        return;
    }

    now_ms = osKernelGetTickCount();

    /* 传感器超时检测：基于时间戳判断数据新鲜度 */
    if (sensor.valid == 0U || (sensor.timestamp_ms > 0U && (now_ms - sensor.timestamp_ms) > LINE_SENSOR_TIMEOUT_MS))
    {
        g_line_state.tracking_active = 0U;
        g_line_state.lost_reason     = LINE_LOST_STALE;
        if (sensor.timestamp_ms > 0U)
        {
            ControlManager_ClearSource(CONTROL_SOURCE_LINE);
        }
        return;
    }
    if (sensor.timestamp_ms == g_line_last_processed_timestamp)
    {
        return;
    }
    g_line_last_processed_timestamp = sensor.timestamp_ms;

    /* 加权平均计算黑线位置 */
    sum_positions = 0.0f;
    detected      = 0U;
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
        g_detect_streak = 0U;
        if (g_lost_streak < 255U)
        {
            g_lost_streak++;
        }
        g_line_state.tracking_active = 0U;
        g_line_state.lost_reason     = LINE_LOST_NO_CHANNEL;
        if (g_lost_streak >= params.line_lost_debounce_frames)
        {
            ControlManager_ClearSource(CONTROL_SOURCE_LINE);
        }
        return;
    }

    g_lost_streak = 0U;
    if (g_detect_streak < 255U)
    {
        g_detect_streak++;
    }
    if (g_detect_streak < params.line_detect_debounce_frames)
    {
        return;
    }

    position = sum_positions / (float)detected;
    error    = 3.5f - position; /* 中心：CH3/CH4中间。线偏左→error>0→左转靠近线 */
    if (g_previous_timestamp_ms != 0U && sensor.timestamp_ms != g_previous_timestamp_ms)
    {
        derivative =
            (error - g_previous_error) * 1000.0f / (float)(uint32_t)(sensor.timestamp_ms - g_previous_timestamp_ms);
    }
    angular_z                     = params.line_kp * error + params.line_kd * derivative;
    g_line_state.output_saturated = (angular_z > LINE_ANGULAR_MAX_RPS || angular_z < -LINE_ANGULAR_MAX_RPS) ? 1U : 0U;
    angular_z                     = LineControl_ClampFloat(angular_z, LINE_ANGULAR_MAX_RPS);
    speed = params.line_speed_mps / (1.0f + params.line_slowdown_gain * ((error < 0.0f) ? -error : error));

    g_line_state.line_position    = position;
    g_line_state.error            = error;
    g_line_state.error_derivative = derivative;
    g_line_state.detected_count   = detected;
    g_line_state.linear_x         = speed;
    g_line_state.angular_z        = angular_z;
    g_line_state.tracking_active  = 1U;
    g_line_state.globally_enabled = g_line_enabled;
    g_line_state.active_low       = params.line_active_low;
    g_line_state.lost_reason      = LINE_LOST_NONE;
    for (i = 0U; i < LINE_SENSOR_CHANNELS; ++i)
    {
        g_line_state.sensor_state[i]  = sensor.state[i];
        g_line_state.sensor_raw[i]    = sensor.analog[i];
        g_line_state.threshold_raw[i] = params.line_threshold_raw[i];
    }
    g_previous_error        = error;
    g_previous_timestamp_ms = sensor.timestamp_ms;
    LineControl_SubmitCommand(speed, angular_z);
}

uint8_t LineControl_CalibrationBegin(line_calibration_surface_t surface, uint16_t samples)
{
    g_line_calibration_last_timestamp = 0U;
    return LineCalibration_Begin(&g_line_calibration, surface, samples);
}

uint8_t LineControl_CalibrationBuild(uint16_t thresholds[LINE_CALIBRATION_CHANNELS], uint8_t *active_low)
{
    return LineCalibration_Apply(&g_line_calibration, thresholds, active_low);
}

uint8_t LineControl_CalibrationApplyAndSave(void)
{
    uint16_t      thresholds[LINE_CALIBRATION_CHANNELS];
    uint8_t       active_low;
    param_store_t params;

    if (LineCalibration_Apply(&g_line_calibration, thresholds, &active_low) == 0U)
    {
        return 0U;
    }
    ParamStore_Get(&params);
    for (uint8_t i = 0U; i < LINE_CALIBRATION_CHANNELS; ++i)
    {
        params.line_threshold_raw[i] = thresholds[i];
    }
    params.line_active_low = active_low;
    return ParamStore_Set(&params);
}

void LineControl_CalibrationGet(line_calibration_t *calibration)
{
    if (calibration != 0)
    {
        *calibration = g_line_calibration;
    }
}

void LineControl_CalibrationCancel(void)
{
    LineCalibration_Init(&g_line_calibration);
    g_line_calibration_last_timestamp = 0U;
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
        g_line_enable_generation        = generation;
        g_detect_streak                 = 0U;
        g_lost_streak                   = 0U;
        g_previous_timestamp_ms         = 0U;
        g_line_last_processed_timestamp = 0U;
        if (LineControl_SafetyActive() != 0U || generation != ControlManager_GetMotionRevokeGeneration())
        {
            g_line_enabled = 0U;
            ControlManager_ClearSource(CONTROL_SOURCE_LINE);
            return;
        }
        g_line_enabled = 1U;
    }
    else
    {
        g_line_enabled                  = 0U;
        g_detect_streak                 = 0U;
        g_lost_streak                   = 0U;
        g_line_state.tracking_active    = 0U;
        g_line_last_processed_timestamp = 0U;
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
        *state                  = g_line_state;
        state->globally_enabled = g_line_enabled;
    }
}
