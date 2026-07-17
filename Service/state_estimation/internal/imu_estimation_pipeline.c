#include "imu_estimation_pipeline.h"

#include "attitude_estimator.h"
#include "imu_signal_filter.h"
#include "imu_quality_monitor.h"
#include "imu_timestamp_tracker.h"

#define IMU_ACCEL_FILTER_ALPHA      0.20f
#define IMU_GYRO_FILTER_ALPHA       0.20f
#define IMU_ATTITUDE_MAX_DT_S       0.100f
#define IMU_GYRO_CAL_MAX_ABS_DPS    20.0f
#define IMU_GYRO_CAL_STILL_SPAN_DPS 5.0f
#define IMU_AUTO_CAL_MAX_ATTEMPTS   5U
#define IMU_AUTO_CAL_RETRY_MS       2000U
#define IMU_GYRO_SATURATION_DPS     490.0f
#define IMU_ACCEL_REJECT_MIN_G_SQ   0.16f
#define IMU_ACCEL_REJECT_MAX_G_SQ   3.24f

static imu_calibration_model_t           calibration;
static imu_bmi270_mahony_t               attitude;
static imu_bmi270_mahony_params_t        attitude_params;
static imu_bmi270_gyro_cal_accumulator_t calibration_accumulator;
static uint8_t                           calibration_active;
static uint8_t                           calibration_automatic;
static uint32_t                          last_calibration_sample_count;
static uint32_t                          auto_cal_next_ms;

void ImuEstimationPipeline_Init(void)
{
    ImuBmi270Calibration_Default(&calibration);
    attitude_params = ImuBmi270Mahony_DefaultParams();
    ImuEstimationPipeline_ResetRuntime();
}

void ImuEstimationPipeline_ResetRuntime(void)
{
    ImuBmi270Mahony_Init(&attitude);
    ImuBmi270GyroCalAccumulator_Init(&calibration_accumulator);
    calibration_active            = 0U;
    calibration_automatic         = 0U;
    last_calibration_sample_count = 0UL;
    auto_cal_next_ms              = 1000UL;
}

void ImuEstimationPipeline_Process(const bmi270_sample_t         *sample,
                                   const bmi270_driver_state_t   *device,
                                   state_estimation_imu_status_t *status)
{
    float    sensor_accel_g[3];
    float    corrected_gyro_dps[3];
    float    body_accel_g[3];
    float    body_gyro_dps[3];
    float    ros_accel_g[3];
    float    ros_gyro_dps[3];
    float    euler_deg[3];
    float    dt_s           = 0.010f;
    uint8_t  dt_valid       = 1U;
    uint32_t quality_events = 0UL;

    if (sample == 0 || device == 0 || status == 0)
    {
        return;
    }
    if (sample->sensor_time_valid != 0U && status->sensor_time_valid != 0U)
    {
        dt_valid = ImuBmi270Time_DeltaSeconds(sample->sensor_time, status->sensor_time, IMU_ATTITUDE_MAX_DT_S, &dt_s);
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        sensor_accel_g[axis] = (sample->accel_g[axis] - calibration.accel_bias_g[axis]) * calibration.accel_scale[axis];
        corrected_gyro_dps[axis] = sample->gyro_dps[axis]
                                   - ImuBmi270Calibration_GyroBiasAtTemperature(&calibration,
                                                                                axis,
                                                                                device->temperature_c,
                                                                                device->temperature_valid);
        status->accel_raw[axis]          = sample->accel_raw[axis];
        status->gyro_raw[axis]           = sample->gyro_raw[axis];
        status->gyro_bias_dps[axis]      = calibration.gyro_bias_dps[axis];
        status->gyro_corrected_dps[axis] = corrected_gyro_dps[axis];
        if (sample->gyro_dps[axis] >= IMU_GYRO_SATURATION_DPS || sample->gyro_dps[axis] <= -IMU_GYRO_SATURATION_DPS)
        {
            quality_events |= STATE_ESTIMATION_IMU_QUALITY_GYRO_SATURATION;
        }
    }
    ImuBmi270Coordinate_Apply(calibration.sensor_to_body, sensor_accel_g, body_accel_g);
    ImuBmi270Coordinate_Apply(calibration.sensor_to_body, corrected_gyro_dps, body_gyro_dps);
    ImuBmi270Coordinate_BodyToRos(body_accel_g, ros_accel_g);
    ImuBmi270Coordinate_BodyToRos(body_gyro_dps, ros_gyro_dps);
    {
        float accel_norm_sq =
            body_accel_g[0] * body_accel_g[0] + body_accel_g[1] * body_accel_g[1] + body_accel_g[2] * body_accel_g[2];
        if (accel_norm_sq < IMU_ACCEL_REJECT_MIN_G_SQ || accel_norm_sq > IMU_ACCEL_REJECT_MAX_G_SQ)
        {
            quality_events |= STATE_ESTIMATION_IMU_QUALITY_ACCEL_ANOMALY;
        }
    }
    if (dt_valid != 0U)
    {
        ImuBmi270Mahony_Update(&attitude, body_gyro_dps, body_accel_g, dt_s, &attitude_params);
    }
    else
    {
        quality_events |= STATE_ESTIMATION_IMU_QUALITY_TIMESTAMP_ERROR | STATE_ESTIMATION_IMU_QUALITY_ATTITUDE_INVALID;
    }
    ImuBmi270Quaternion_ToEulerDeg(&attitude.q, euler_deg);
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        if (status->filter_initialized == 0U)
        {
            status->accel_g[axis]           = sensor_accel_g[axis];
            status->body_accel_g[axis]      = body_accel_g[axis];
            status->ros_accel_g[axis]       = ros_accel_g[axis];
            status->gyro_filtered_dps[axis] = body_gyro_dps[axis];
            status->ros_gyro_dps[axis]      = ros_gyro_dps[axis];
        }
        else
        {
            status->accel_g[axis] =
                ImuSignalFilter_LowPass(status->accel_g[axis], sensor_accel_g[axis], IMU_ACCEL_FILTER_ALPHA);
            status->body_accel_g[axis] =
                ImuSignalFilter_LowPass(status->body_accel_g[axis], body_accel_g[axis], IMU_ACCEL_FILTER_ALPHA);
            status->ros_accel_g[axis] =
                ImuSignalFilter_LowPass(status->ros_accel_g[axis], ros_accel_g[axis], IMU_ACCEL_FILTER_ALPHA);
            status->gyro_filtered_dps[axis] =
                ImuSignalFilter_LowPass(status->gyro_filtered_dps[axis], body_gyro_dps[axis], IMU_GYRO_FILTER_ALPHA);
            status->ros_gyro_dps[axis] =
                ImuSignalFilter_LowPass(status->ros_gyro_dps[axis], ros_gyro_dps[axis], IMU_GYRO_FILTER_ALPHA);
        }
        status->gyro_dps[axis]      = status->gyro_filtered_dps[axis];
        status->body_gyro_dps[axis] = status->gyro_filtered_dps[axis];
    }
    status->filter_initialized      = 1U;
    status->quaternion[0]           = attitude.q.w;
    status->quaternion[1]           = attitude.q.x;
    status->quaternion[2]           = attitude.q.y;
    status->quaternion[3]           = attitude.q.z;
    status->roll_deg                = euler_deg[0];
    status->pitch_deg               = euler_deg[1];
    status->yaw_deg                 = ImuSignalFilter_WrapAngleDeg(euler_deg[2]);
    status->accel_correction_weight = attitude.accel_weight;
    status->sensor_time             = sample->sensor_time;
    status->sensor_time_valid       = sample->sensor_time_valid;
    status->last_update_ms          = sample->timestamp_ms;
    if ((attitude.status_flags & IMU_BMI270_FUSION_ACCEL_DEGRADED) != 0UL)
    {
        quality_events |= STATE_ESTIMATION_IMU_QUALITY_ACCEL_ANOMALY;
    }
    if ((attitude.status_flags & IMU_BMI270_FUSION_INVALID_DT) != 0UL)
    {
        quality_events |= STATE_ESTIMATION_IMU_QUALITY_ATTITUDE_INVALID;
    }
    ImuQualityMonitor_RecordSample(quality_events, status);
}

uint8_t ImuEstimationPipeline_ApplyCalibration(const imu_calibration_t *value)
{
    imu_calibration_model_t model;

    if (value == 0)
    {
        return 0U;
    }
    model.version              = value->version;
    model.temperature_offset_c = value->temperature_offset_c;
    model.crc                  = value->crc;
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        model.accel_bias_g[axis]                     = value->accel_bias_g[axis];
        model.accel_scale[axis]                      = value->accel_scale[axis];
        model.gyro_bias_dps[axis]                    = value->gyro_bias_dps[axis];
        model.temperature_gyro_slope_dps_per_c[axis] = value->temperature_gyro_slope_dps_per_c[axis];
        for (uint8_t column = 0U; column < 3U; ++column)
        {
            model.sensor_to_body[axis][column] = value->sensor_to_body[axis][column];
        }
    }
    if (ImuBmi270Calibration_Validate(&model) == 0U)
    {
        return 0U;
    }
    calibration = model;
    return 1U;
}

void ImuEstimationPipeline_ClearCalibration(void)
{
    ImuBmi270Calibration_Default(&calibration);
    calibration_active = 0U;
    ImuBmi270GyroCalAccumulator_Init(&calibration_accumulator);
    ImuBmi270Mahony_Init(&attitude);
}

void ImuEstimationPipeline_GetCalibration(imu_calibration_t *value)
{
    if (value != 0)
    {
        value->version              = calibration.version;
        value->temperature_offset_c = calibration.temperature_offset_c;
        value->crc                  = calibration.crc;
        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            value->accel_bias_g[axis]                     = calibration.accel_bias_g[axis];
            value->accel_scale[axis]                      = calibration.accel_scale[axis];
            value->gyro_bias_dps[axis]                    = calibration.gyro_bias_dps[axis];
            value->temperature_gyro_slope_dps_per_c[axis] = calibration.temperature_gyro_slope_dps_per_c[axis];
            for (uint8_t column = 0U; column < 3U; ++column)
            {
                value->sensor_to_body[axis][column] = calibration.sensor_to_body[axis][column];
            }
        }
    }
}

uint8_t ImuEstimationPipeline_BeginCalibration(uint16_t samples, uint16_t interval_ms, uint8_t automatic)
{
    if (calibration_active != 0U
        || ImuBmi270GyroCalAccumulator_Begin(&calibration_accumulator, samples, interval_ms) == 0U)
    {
        return 0U;
    }
    calibration_active    = 1U;
    calibration_automatic = automatic;
    return 1U;
}

void ImuEstimationPipeline_ServiceCalibration(uint32_t                       now_ms,
                                              uint8_t                        stationary,
                                              state_estimation_imu_status_t *status)
{
    imu_bmi270_gyro_cal_acc_state_t result;
    float                           bias_dps[3];
    float                           accel_mean_g[3];

    if (status == 0)
    {
        return;
    }
    if (status->enabled == 0U || (status->gyro_auto_cal_enabled == 0U && calibration_active == 0U))
    {
        status->gyro_auto_cal_state = STATE_ESTIMATION_IMU_AUTO_CAL_DISABLED;
        return;
    }
    if (status->gyro_calibrated != 0U && calibration_active == 0U)
    {
        status->gyro_auto_cal_state = STATE_ESTIMATION_IMU_AUTO_CAL_SUCCESS;
        return;
    }
    if (calibration_active == 0U)
    {
        if (status->gyro_auto_cal_attempts >= IMU_AUTO_CAL_MAX_ATTEMPTS)
        {
            status->gyro_auto_cal_state = STATE_ESTIMATION_IMU_AUTO_CAL_FAILED;
            return;
        }
        if (status->online == 0U || status->sample_count == last_calibration_sample_count)
        {
            status->gyro_auto_cal_state = STATE_ESTIMATION_IMU_AUTO_CAL_WAIT_ONLINE;
            return;
        }
        last_calibration_sample_count = status->sample_count;
        if (stationary == 0U)
        {
            status->gyro_auto_cal_state = STATE_ESTIMATION_IMU_AUTO_CAL_WAIT_STATIONARY;
            return;
        }
        if ((int32_t)(now_ms - auto_cal_next_ms) < 0)
        {
            status->gyro_auto_cal_state = STATE_ESTIMATION_IMU_AUTO_CAL_RETRY_WAIT;
            return;
        }
        if (ImuEstimationPipeline_BeginCalibration(500U, 10U, 1U) != 0U)
        {
            status->gyro_auto_cal_state = STATE_ESTIMATION_IMU_AUTO_CAL_COLLECTING;
        }
        return;
    }
    if (status->sample_count == last_calibration_sample_count)
    {
        return;
    }
    last_calibration_sample_count = status->sample_count;
    if (stationary == 0U)
    {
        calibration_active           = 0U;
        status->gyro_cal_fail_reason = 5U;
        status->gyro_auto_cal_state  = STATE_ESTIMATION_IMU_AUTO_CAL_RETRY_WAIT;
        auto_cal_next_ms             = now_ms + IMU_AUTO_CAL_RETRY_MS;
        ImuBmi270GyroCalAccumulator_Init(&calibration_accumulator);
        return;
    }
    result                        = ImuBmi270GyroCalAccumulator_Feed(&calibration_accumulator,
                                              now_ms,
                                              status->accel_g,
                                              status->gyro_corrected_dps,
                                              IMU_GYRO_CAL_MAX_ABS_DPS,
                                              IMU_GYRO_CAL_STILL_SPAN_DPS);
    status->gyro_cal_sample_count = calibration_accumulator.sample_count;
    status->gyro_auto_cal_state   = STATE_ESTIMATION_IMU_AUTO_CAL_COLLECTING;
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        status->gyro_cal_min_dps[axis]  = calibration_accumulator.min_dps[axis];
        status->gyro_cal_max_dps[axis]  = calibration_accumulator.max_dps[axis];
        status->gyro_cal_span_dps[axis] = calibration_accumulator.max_dps[axis] - calibration_accumulator.min_dps[axis];
        status->gyro_cal_mean_dps[axis] =
            (calibration_accumulator.sample_count != 0U)
                ? calibration_accumulator.sum_dps[axis] / (float)calibration_accumulator.sample_count
                : 0.0f;
    }
    if (result == IMU_BMI270_GYRO_CAL_ACC_READY
        && ImuBmi270GyroCalAccumulator_GetResult(&calibration_accumulator, bias_dps, accel_mean_g) != 0U)
    {
        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            calibration.gyro_bias_dps[axis] += bias_dps[axis];
            status->gyro_bias_dps[axis] = calibration.gyro_bias_dps[axis];
        }
        calibration.crc                   = ImuBmi270Calibration_Crc(&calibration);
        status->gyro_calibrated           = 1U;
        status->gyro_auto_cal_state       = STATE_ESTIMATION_IMU_AUTO_CAL_SUCCESS;
        status->gyro_auto_cal_last_result = 1U;
        calibration_active                = 0U;
        ImuBmi270GyroCalAccumulator_Init(&calibration_accumulator);
    }
    else if (result == IMU_BMI270_GYRO_CAL_ACC_FAIL_ABS || result == IMU_BMI270_GYRO_CAL_ACC_FAIL_SPAN)
    {
        status->gyro_cal_fail_reason = (result == IMU_BMI270_GYRO_CAL_ACC_FAIL_ABS) ? 3U : 4U;
        status->gyro_cal_fail_axis   = calibration_accumulator.fail_axis;
        if (calibration_automatic != 0U)
        {
            status->gyro_auto_cal_attempts++;
        }
        status->gyro_auto_cal_state = (status->gyro_auto_cal_attempts >= IMU_AUTO_CAL_MAX_ATTEMPTS)
                                          ? STATE_ESTIMATION_IMU_AUTO_CAL_FAILED
                                          : STATE_ESTIMATION_IMU_AUTO_CAL_RETRY_WAIT;
        auto_cal_next_ms            = now_ms + IMU_AUTO_CAL_RETRY_MS;
        calibration_active          = 0U;
        ImuBmi270GyroCalAccumulator_Init(&calibration_accumulator);
    }
}
