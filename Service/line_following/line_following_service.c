#include "line_following_service.h"
#include "line_following_maintenance.h"
#include "line_following_internal.h"
#include "line_parameter_sync.h"
#include "line_sensor_calibration.h"
#include "platform_critical.h"
#include "platform_time.h"

#include "command_management_service.h"

#include "line_sensor_driver.h"

#include "parameter_management_service.h"

#include "safety_management_service.h"

#define LINE_LOST_NONE       0U
#define LINE_LOST_STALE      1U
#define LINE_LOST_NO_CHANNEL 2U

static line_following_config_t              g_line_config;
static uint8_t                              g_line_enabled;
static uint32_t                             g_line_enable_generation;
static line_calibration_model_t             g_line_sensor_calibration;
static uint32_t                             g_line_sensor_calibration_last_timestamp;
static uint32_t                             g_line_sensor_calibration_generation;
static line_following_calibration_request_t g_line_sensor_calibration_request;
static uint8_t                              g_line_sensor_calibration_request_state;
static line_following_status_t              g_line_state;
static uint8_t                              g_detect_streak;
static uint8_t                              g_lost_streak;
static float                                g_previous_error;
static uint32_t                             g_previous_timestamp_ms;
static uint32_t                             g_line_last_processed_timestamp;
static uint32_t                             g_line_control_generation;
static uint32_t                             g_line_applied_control_generation;
static line_parameter_sync_t                g_line_parameter_sync;

static uint8_t LineFollowing_Publish(line_following_status_t *state, uint32_t expected_control_generation)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    if (g_line_control_generation != expected_control_generation)
    {
        PlatformCritical_Exit(critical);
        return 0U;
    }
    state->globally_enabled = g_line_enabled;
    state->generation       = g_line_state.generation + 1UL;
    g_line_state            = *state;
    PlatformCritical_Exit(critical);
    return 1U;
}

static uint8_t LineFollowing_SafetyActive(void)
{
    return (SafetyManagement_IsMotionAllowed() == 0U) ? 1U : 0U;
}

static void LineFollowing_SubmitCommand(float linear_x, float angular_z, uint32_t input_generation)
{
    command_intent_t intent = {
        .kind                       = COMMAND_INTENT_ACTIVE,
        .source                     = COMMAND_SOURCE_LINE,
        .linear_x                   = linear_x,
        .angular_z                  = angular_z,
        .sample_time_ms             = PlatformTime_TaskNowMs(),
        .producer_generation        = g_line_state.generation,
        .expected_revoke_generation = input_generation,
    };
    (void)CommandManagement_ApplyIntent(&intent);
}

static void LineFollowing_ApplySlotIntent(command_intent_kind_t kind, uint32_t input_generation)
{
    command_intent_t intent = {
        .kind                       = kind,
        .source                     = COMMAND_SOURCE_LINE,
        .sample_time_ms             = PlatformTime_TaskNowMs(),
        .producer_generation        = g_line_state.generation,
        .expected_revoke_generation = input_generation,
    };

    (void)CommandManagement_ApplyIntent(&intent);
}

static float LineFollowing_ClampFloat(float value, float limit)
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

uint8_t LineFollowing_Init(const line_following_config_t *config)
{
    if (config == 0 || config->angular_max_rps <= 0.0f || config->sensor_timeout_ms == 0UL
        || config->detect_threshold_count == 0U || config->detect_threshold_count > 8U)
    {
        return 0U;
    }
    g_line_config            = *config;
    g_line_enabled           = config->default_enabled;
    g_line_enable_generation = CommandManagement_GetMotionRevokeGeneration();
    LineSensorCalibration_Init(&g_line_sensor_calibration);
    g_line_sensor_calibration_last_timestamp = 0U;
    g_line_sensor_calibration_generation     = 1UL;
    g_line_sensor_calibration_request        = (line_following_calibration_request_t){0};
    g_line_sensor_calibration_request_state  = 0U;
    g_line_state                             = (line_following_status_t){0};
    g_detect_streak                          = 0U;
    g_lost_streak                            = 0U;
    g_previous_error                         = 0.0f;
    g_previous_timestamp_ms                  = 0U;
    g_line_last_processed_timestamp          = 0U;
    g_line_control_generation                = 1UL;
    g_line_applied_control_generation        = 1UL;
    g_line_state.generation                  = 1UL;
    LineParameterSync_Init(&g_line_parameter_sync);
    (void)LineParameterSync_Update(&g_line_parameter_sync);
    return 1U;
}

void LineFollowing_Update(void)
{
    (void)LineParameterSync_Update(&g_line_parameter_sync);
    LineSensorDriver_Update();
    LineSensorDriver_RequestAnalog();
    {
        line_following_calibration_request_t request;

        if (LineFollowingInternal_TakeCalibrationRequest(&request) != 0U)
        {
            (void)LineFollowingInternal_ResolveCalibrationRequest();
        }
    }
    line_sensor_data_t        sensor;
    uint32_t                  now_ms;
    uint8_t                   i;
    float                     sum_positions;
    uint8_t                   detected;
    float                     position;
    float                     error;
    float                     angular_z;
    float                     derivative = 0.0f;
    float                     speed;
    param_model_t             params;
    line_following_status_t   next_state;
    uint8_t                   state_changed = 0U;
    uint8_t                   enabled;
    uint32_t                  input_generation;
    uint32_t                  control_generation;
    platform_critical_state_t critical;
    line_calibration_model_t  calibration_next;
    uint32_t                  calibration_generation;
    uint32_t                  calibration_last_timestamp;

    (void)ParameterManagement_GetSnapshot(&params);
    critical           = PlatformCritical_Enter();
    next_state         = g_line_state;
    enabled            = g_line_enabled;
    input_generation   = g_line_enable_generation;
    control_generation = g_line_control_generation;
    PlatformCritical_Exit(critical);
    critical                   = PlatformCritical_Enter();
    calibration_next           = g_line_sensor_calibration;
    calibration_generation     = g_line_sensor_calibration_generation;
    calibration_last_timestamp = g_line_sensor_calibration_last_timestamp;
    PlatformCritical_Exit(critical);
    if (g_line_applied_control_generation != control_generation)
    {
        g_detect_streak                   = 0U;
        g_lost_streak                     = 0U;
        g_previous_timestamp_ms           = 0U;
        g_line_last_processed_timestamp   = 0U;
        g_line_applied_control_generation = control_generation;
    }

    (void)LineSensorDriver_GetSensorData(&sensor);
    next_state.sensor_valid        = sensor.valid;
    next_state.sensor_timestamp_ms = sensor.timestamp_ms;
    if (calibration_next.collecting != 0U && sensor.valid != 0U && sensor.timestamp_ms != calibration_last_timestamp)
    {
        next_state.tracking_active = 0U;
        next_state.lost_reason     = LINE_LOST_STALE;
        state_changed              = 1U;
        LineSensorCalibration_Feed(&calibration_next, sensor.analog);
        critical = PlatformCritical_Enter();
        if (g_line_sensor_calibration_generation == calibration_generation)
        {
            g_line_sensor_calibration                = calibration_next;
            g_line_sensor_calibration_last_timestamp = sensor.timestamp_ms;
            g_line_sensor_calibration_generation++;
        }
        PlatformCritical_Exit(critical);
    }

    if (enabled == 0U)
    {
        if (state_changed != 0U)
        {
            (void)LineFollowing_Publish(&next_state, control_generation);
        }
        return;
    }
    if (LineFollowing_SafetyActive() != 0U || input_generation != CommandManagement_GetMotionRevokeGeneration())
    {
        LineFollowing_Enable(0U);
        return;
    }

    now_ms = PlatformTime_TaskNowMs();

    /* 传感器超时检测：基于时间戳判断数据新鲜度 */
    if (sensor.valid == 0U
        || (sensor.timestamp_ms > 0U && (now_ms - sensor.timestamp_ms) > g_line_config.sensor_timeout_ms))
    {
        next_state.tracking_active = 0U;
        next_state.lost_reason     = LINE_LOST_STALE;
        if (sensor.timestamp_ms > 0U)
        {
            LineFollowing_ApplySlotIntent(COMMAND_INTENT_RELEASE, input_generation);
        }
        (void)LineFollowing_Publish(&next_state, control_generation);
        return;
    }
    if (sensor.timestamp_ms == g_line_last_processed_timestamp)
    {
        if (state_changed != 0U)
        {
            (void)LineFollowing_Publish(&next_state, control_generation);
        }
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

    if (detected < g_line_config.detect_threshold_count)
    {
        g_detect_streak = 0U;
        if (g_lost_streak < 255U)
        {
            g_lost_streak++;
        }
        next_state.tracking_active = 0U;
        next_state.lost_reason     = LINE_LOST_NO_CHANNEL;
        if (g_lost_streak >= params.line_lost_debounce_frames)
        {
            LineFollowing_ApplySlotIntent(COMMAND_INTENT_RELEASE, input_generation);
        }
        (void)LineFollowing_Publish(&next_state, control_generation);
        return;
    }

    g_lost_streak = 0U;
    if (g_detect_streak < 255U)
    {
        g_detect_streak++;
    }
    if (g_detect_streak < params.line_detect_debounce_frames)
    {
        if (state_changed != 0U)
        {
            (void)LineFollowing_Publish(&next_state, control_generation);
        }
        return;
    }

    position = sum_positions / (float)detected;
    error    = 3.5f - position; /* 中心：CH3/CH4中间。线偏左→error>0→左转靠近线 */
    if (g_previous_timestamp_ms != 0U && sensor.timestamp_ms != g_previous_timestamp_ms)
    {
        derivative =
            (error - g_previous_error) * 1000.0f / (float)(uint32_t)(sensor.timestamp_ms - g_previous_timestamp_ms);
    }
    angular_z = params.line_kp * error + params.line_kd * derivative;
    next_state.output_saturated =
        (angular_z > g_line_config.angular_max_rps || angular_z < -g_line_config.angular_max_rps) ? 1U : 0U;
    angular_z = LineFollowing_ClampFloat(angular_z, g_line_config.angular_max_rps);
    speed     = params.line_speed_mps / (1.0f + params.line_slowdown_gain * ((error < 0.0f) ? -error : error));

    next_state.line_position    = position;
    next_state.error            = error;
    next_state.error_derivative = derivative;
    next_state.detected_count   = detected;
    next_state.linear_x         = speed;
    next_state.angular_z        = angular_z;
    next_state.tracking_active  = 1U;
    next_state.active_low       = params.line_active_low;
    next_state.lost_reason      = LINE_LOST_NONE;
    for (i = 0U; i < LINE_SENSOR_CHANNELS; ++i)
    {
        next_state.sensor_state[i]  = sensor.state[i];
        next_state.sensor_raw[i]    = sensor.analog[i];
        next_state.threshold_raw[i] = params.line_threshold_raw[i];
    }
    g_previous_error        = error;
    g_previous_timestamp_ms = sensor.timestamp_ms;
    if (LineFollowing_Publish(&next_state, control_generation) != 0U)
    {
        LineFollowing_SubmitCommand(speed, angular_z, input_generation);
    }
}

uint8_t LineFollowingInternal_CalibrationStart(line_sensor_calibration_surface_t surface, uint16_t samples)
{
    uint8_t                   started;
    line_calibration_model_t  next;
    platform_critical_state_t critical;

    critical = PlatformCritical_Enter();
    next     = g_line_sensor_calibration;
    PlatformCritical_Exit(critical);
    started = LineSensorCalibration_Begin(&next, (line_calibration_surface_t)surface, samples);
    if (started != 0U)
    {
        critical                                 = PlatformCritical_Enter();
        g_line_sensor_calibration                = next;
        g_line_sensor_calibration_last_timestamp = 0U;
        g_line_sensor_calibration_generation++;
        PlatformCritical_Exit(critical);
    }
    return started;
}

uint8_t LineFollowing_RequestCalibration(line_sensor_calibration_surface_t surface, uint16_t samples)
{
    platform_critical_state_t critical;
    uint8_t                   accepted = 0U;

    if ((uint8_t)surface > (uint8_t)LINE_CALIBRATION_SURFACE_LINE || samples < 4U || samples > 2000U)
    {
        return 0U;
    }
    critical = PlatformCritical_Enter();
    if (g_line_sensor_calibration.collecting == 0U && g_line_sensor_calibration_request_state == 0U)
    {
        g_line_sensor_calibration_request.surface = surface;
        g_line_sensor_calibration_request.samples = samples;
        g_line_sensor_calibration_request_state   = 1U;
        accepted                                  = 1U;
    }
    PlatformCritical_Exit(critical);
    return accepted;
}

uint8_t LineFollowingInternal_TakeCalibrationRequest(line_following_calibration_request_t *request)
{
    platform_critical_state_t critical;
    uint8_t                   available = 0U;

    if (request == 0)
    {
        return 0U;
    }
    critical = PlatformCritical_Enter();
    if (g_line_sensor_calibration_request_state == 1U)
    {
        *request                                = g_line_sensor_calibration_request;
        g_line_sensor_calibration_request_state = 2U;
        available                               = 1U;
    }
    PlatformCritical_Exit(critical);
    return available;
}

uint8_t LineFollowingInternal_ResolveCalibrationRequest(void)
{
    line_following_calibration_request_t request;
    platform_critical_state_t            critical;
    uint8_t                              pending;

    critical = PlatformCritical_Enter();
    pending  = (g_line_sensor_calibration_request_state == 2U) ? 1U : 0U;
    request  = g_line_sensor_calibration_request;
    PlatformCritical_Exit(critical);
    if (pending != 0U)
    {
        pending = LineFollowingInternal_CalibrationStart(request.surface, request.samples);
    }
    critical = PlatformCritical_Enter();
    if (g_line_sensor_calibration_request_state == 2U)
    {
        g_line_sensor_calibration_request_state = 0U;
    }
    PlatformCritical_Exit(critical);
    return pending;
}

uint8_t LineFollowingInternal_CalibrationBuild(uint16_t thresholds[LINE_CALIBRATION_CHANNELS], uint8_t *active_low)
{
    line_calibration_model_t  snapshot;
    platform_critical_state_t critical = PlatformCritical_Enter();

    snapshot = g_line_sensor_calibration;
    PlatformCritical_Exit(critical);
    return LineSensorCalibration_Apply(&snapshot, thresholds, active_low);
}

line_calibration_apply_result_t LineFollowing_ApplyCalibration(void)
{
    uint16_t                  thresholds[LINE_CALIBRATION_CHANNELS];
    uint8_t                   active_low;
    param_model_t             params;
    line_calibration_model_t  snapshot;
    platform_critical_state_t critical = PlatformCritical_Enter();

    snapshot = g_line_sensor_calibration;
    PlatformCritical_Exit(critical);

    if (snapshot.ready_mask != 0x03U)
    {
        return LINE_CALIBRATION_APPLY_INCOMPLETE;
    }
    if (LineSensorCalibration_Apply(&snapshot, thresholds, &active_low) == 0U)
    {
        critical                            = PlatformCritical_Enter();
        g_line_sensor_calibration.fail_mask = snapshot.fail_mask;
        g_line_sensor_calibration_generation++;
        PlatformCritical_Exit(critical);
        return LINE_CALIBRATION_APPLY_LOW_SEPARATION;
    }
    (void)ParameterManagement_GetSnapshot(&params);
    for (uint8_t i = 0U; i < LINE_CALIBRATION_CHANNELS; ++i)
    {
        params.line_threshold_raw[i] = thresholds[i];
    }
    params.line_active_low = active_low;
    if (ParameterManagement_Set(&params) == 0U)
    {
        return LINE_CALIBRATION_APPLY_PARAMETER_REJECTED;
    }
    (void)LineParameterSync_Update(&g_line_parameter_sync);
    critical                            = PlatformCritical_Enter();
    g_line_sensor_calibration.fail_mask = 0U;
    g_line_sensor_calibration_generation++;
    PlatformCritical_Exit(critical);
    return LINE_CALIBRATION_APPLY_OK;
}

void LineFollowing_CalibrationGet(line_sensor_calibration_t *calibration)
{
    if (calibration != 0)
    {
        platform_critical_state_t critical = PlatformCritical_Enter();

        for (uint8_t surface = 0U; surface < 2U; ++surface)
        {
            calibration->count[surface] = g_line_sensor_calibration.count[surface];
            for (uint8_t channel = 0U; channel < LINE_CALIBRATION_CHANNELS; ++channel)
            {
                calibration->sum[surface][channel] = g_line_sensor_calibration.sum[surface][channel];
                calibration->min[surface][channel] = g_line_sensor_calibration.min[surface][channel];
                calibration->max[surface][channel] = g_line_sensor_calibration.max[surface][channel];
            }
        }
        calibration->target_samples = g_line_sensor_calibration.target_samples;
        calibration->collecting     = g_line_sensor_calibration.collecting;
        calibration->surface        = g_line_sensor_calibration.surface;
        calibration->ready_mask     = g_line_sensor_calibration.ready_mask;
        calibration->fail_mask      = g_line_sensor_calibration.fail_mask;
        PlatformCritical_Exit(critical);
    }
}

void LineFollowing_CalibrationAbort(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    g_line_sensor_calibration.collecting    = 0U;
    g_line_sensor_calibration_request_state = 0U;
    g_line_sensor_calibration_generation++;
    PlatformCritical_Exit(critical);
}

void LineFollowing_CalibrationCancel(void)
{
    line_calibration_model_t  next;
    platform_critical_state_t critical;

    LineSensorCalibration_Init(&next);
    critical                                 = PlatformCritical_Enter();
    g_line_sensor_calibration                = next;
    g_line_sensor_calibration_last_timestamp = 0U;
    g_line_sensor_calibration_generation++;
    g_line_sensor_calibration_request_state = 0U;
    PlatformCritical_Exit(critical);
}

line_following_result_t LineFollowing_Enable(uint8_t enable)
{
    line_following_status_t   next_state;
    platform_critical_state_t critical;
    uint8_t                   clear_source     = 0U;
    uint32_t                  input_generation = 0UL;
    line_following_result_t   result           = LINE_FOLLOWING_RESULT_APPLIED;

    if (enable != 0U)
    {
        uint32_t generation = CommandManagement_GetMotionRevokeGeneration();

        if (LineFollowing_SafetyActive() != 0U)
        {
            enable       = 0U;
            clear_source = 1U;
            result       = LINE_FOLLOWING_RESULT_REJECTED;
        }
        else if (generation != CommandManagement_GetMotionRevokeGeneration())
        {
            enable       = 0U;
            clear_source = 1U;
            result       = LINE_FOLLOWING_RESULT_REJECTED;
        }
        else
        {
            input_generation = generation;
        }
    }
    else
    {
        clear_source = 1U;
    }

    critical       = PlatformCritical_Enter();
    g_line_enabled = (enable != 0U) ? 1U : 0U;
    if (g_line_enabled != 0U)
    {
        g_line_enable_generation = input_generation;
    }
    g_line_control_generation++;
    next_state = g_line_state;
    if (g_line_enabled == 0U)
    {
        next_state.tracking_active = 0U;
    }
    next_state.globally_enabled   = g_line_enabled;
    next_state.last_enable_result = result;
    next_state.generation++;
    g_line_state = next_state;
    PlatformCritical_Exit(critical);

    if (clear_source != 0U)
    {
        if (enable == 0U && result == LINE_FOLLOWING_RESULT_APPLIED)
        {
            LineFollowing_ApplySlotIntent(COMMAND_INTENT_REARM, input_generation);
        }
        else
        {
            LineFollowing_ApplySlotIntent(COMMAND_INTENT_RELEASE, input_generation);
        }
    }
    return result;
}

uint8_t LineFollowing_IsEnabled(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();
    uint8_t                   enabled  = g_line_enabled;

    PlatformCritical_Exit(critical);
    return enabled;
}

uint32_t LineFollowing_GetStatus(line_following_status_t *state)
{
    uint32_t critical;

    if (state != 0)
    {
        critical                = PlatformCritical_Enter();
        *state                  = g_line_state;
        state->globally_enabled = g_line_enabled;
        PlatformCritical_Exit(critical);
        return state->generation;
    }
    return 0UL;
}
