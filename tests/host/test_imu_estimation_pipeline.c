#include "imu_estimation_pipeline.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static void test_boot_recalibration_uses_history_as_seed(void)
{
    imu_calibration_t             calibration;
    bmi270_driver_state_t         device = {0};
    bmi270_sample_t               sample = {0};
    state_estimation_imu_status_t status = {
        .enabled                   = 1U,
        .online                    = 1U,
        .filter_initialized        = 1U,
        .gyro_calibrated           = 1U,
        .gyro_auto_cal_enabled     = 0U,
        .gyro_auto_cal_state       = STATE_ESTIMATION_IMU_AUTO_CAL_SUCCESS,
        .gyro_auto_cal_attempts    = 4U,
        .gyro_auto_cal_last_result = 1U,
        .gyro_cal_fail_reason      = 3U,
        .gyro_cal_fail_axis        = 2U,
        .gyro_cal_sample_count     = 123U,
        .quaternion                = {0.5f, 0.5f, 0.5f, 0.5f},
        .roll_deg                  = 12.0f,
        .pitch_deg                 = -8.0f,
        .yaw_deg                   = 37.0f,
    };

    ImuEstimationPipeline_Init();
    ImuEstimationPipeline_GetCalibration(&calibration);
    calibration.gyro_bias_dps[0]                    = 9.0f;
    calibration.temperature_gyro_slope_dps_per_c[0] = 0.01f;
    calibration.crc                                 = ParameterImuCalibration_Crc(&calibration);
    require_int(ImuEstimationPipeline_ApplyCalibration(&calibration) != 0U,
                "valid historical calibration is accepted as the boot seed");

    ImuEstimationPipeline_ArmAutomaticCalibration(1000U, &status);
    ImuEstimationPipeline_GetCalibration(&calibration);
    require_int(fabsf(calibration.gyro_bias_dps[0] - 9.0f) < 0.0001f
                    && fabsf(calibration.temperature_gyro_slope_dps_per_c[0] - 0.01f) < 0.0001f,
                "arming boot calibration preserves historical bias and temperature compensation");
    require_int(status.gyro_auto_cal_enabled != 0U && status.gyro_calibrated == 0U
                    && status.gyro_auto_cal_state == STATE_ESTIMATION_IMU_AUTO_CAL_WAIT_ONLINE
                    && status.gyro_auto_cal_attempts == 0U && status.gyro_auto_cal_last_result == 0U
                    && status.gyro_cal_sample_count == 0U,
                "arming boot calibration clears prior terminal and sampling state");
    require_int(status.filter_initialized == 0U && status.quaternion[0] == 1.0f && status.quaternion[1] == 0.0f
                    && status.roll_deg == 0.0f && status.pitch_deg == 0.0f && status.yaw_deg == 0.0f,
                "arming boot calibration resets attitude and filter runtime");

    status.sample_count = 1U;
    ImuEstimationPipeline_ServiceCalibration(1000U, 1U, &status);
    require_int(status.gyro_auto_cal_state == STATE_ESTIMATION_IMU_AUTO_CAL_RETRY_WAIT,
                "boot calibration honors the one-second start delay");
    status.sample_count = 2U;
    ImuEstimationPipeline_ServiceCalibration(2000U, 1U, &status);
    require_int(status.gyro_auto_cal_state == STATE_ESTIMATION_IMU_AUTO_CAL_COLLECTING,
                "boot calibration starts after the delay with a fresh sample");

    sample.accel_g[2]  = 1.0f;
    sample.gyro_dps[0] = 10.0f;
    for (uint32_t feed = 0U; feed < 500U; ++feed)
    {
        status.sample_count = 3U + feed;
        sample.timestamp_ms = 2010U + feed * 10U;
        ImuEstimationPipeline_Process(&sample, &device, &status);
        ImuEstimationPipeline_ServiceCalibration(sample.timestamp_ms, 1U, &status);
    }
    ImuEstimationPipeline_GetCalibration(&calibration);
    require_int(status.gyro_calibrated != 0U && status.gyro_auto_cal_state == STATE_ESTIMATION_IMU_AUTO_CAL_SUCCESS,
                "boot calibration reaches the terminal success state");
    require_int(fabsf(calibration.gyro_bias_dps[0] - 10.0f) < 0.0001f,
                "boot calibration adds the stationary residual to the historical absolute bias");
    require_int(status.filter_initialized == 0U && status.quaternion[0] == 1.0f && status.yaw_deg == 0.0f,
                "successful boot calibration resets attitude to zero");

    status.sample_count++;
    sample.timestamp_ms += 10U;
    ImuEstimationPipeline_Process(&sample, &device, &status);
    require_int(fabsf(status.gyro_corrected_dps[0]) < 0.0001f,
                "the first post-calibration frame uses the new absolute bias");
}

static void test_failed_boot_recalibration_preserves_history(void)
{
    imu_calibration_t             calibration;
    bmi270_driver_state_t         device = {0};
    bmi270_sample_t               sample = {0};
    state_estimation_imu_status_t status = {
        .enabled               = 1U,
        .online                = 1U,
        .gyro_auto_cal_enabled = 1U,
    };

    ImuEstimationPipeline_Init();
    ImuEstimationPipeline_GetCalibration(&calibration);
    calibration.gyro_bias_dps[0] = 9.0f;
    calibration.crc              = ParameterImuCalibration_Crc(&calibration);
    require_int(ImuEstimationPipeline_ApplyCalibration(&calibration) != 0U,
                "failure test applies a valid historical bias");
    ImuEstimationPipeline_ArmAutomaticCalibration(0U, &status);
    sample.accel_g[2]  = 1.0f;
    sample.gyro_dps[0] = 40.0f;
    for (uint8_t attempt = 0U; attempt < 5U; ++attempt)
    {
        require_int(ImuEstimationPipeline_BeginCalibration(1U, 1U, 1U) != 0U, "each bounded automatic attempt starts");
        status.sample_count++;
        sample.timestamp_ms++;
        ImuEstimationPipeline_Process(&sample, &device, &status);
        ImuEstimationPipeline_ServiceCalibration(sample.timestamp_ms, 1U, &status);
    }
    ImuEstimationPipeline_ServiceCalibration(sample.timestamp_ms + 1U, 1U, &status);
    ImuEstimationPipeline_GetCalibration(&calibration);
    require_int(status.gyro_calibrated == 0U && status.gyro_auto_cal_attempts == 5U
                    && status.gyro_auto_cal_state == STATE_ESTIMATION_IMU_AUTO_CAL_FAILED,
                "five failed attempts remain explicitly failed and uncalibrated");
    require_int(fabsf(calibration.gyro_bias_dps[0] - 9.0f) < 0.0001f,
                "failed boot recalibration does not overwrite the historical bias model");
}

int main(void)
{
    imu_calibration_t             calibration;
    bmi270_driver_state_t         device = {0};
    bmi270_sample_t               sample = {0};
    state_estimation_imu_status_t status = {0};

    test_boot_recalibration_uses_history_as_seed();
    test_failed_boot_recalibration_preserves_history();

    ImuEstimationPipeline_Init();
    ImuEstimationPipeline_GetCalibration(&calibration);
    calibration.gyro_bias_dps[0] = 9.0f;
    calibration.crc              = ParameterImuCalibration_Crc(&calibration);
    require_int(ImuEstimationPipeline_ApplyCalibration(&calibration) != 0U, "valid persisted calibration is applied");

    status.enabled               = 1U;
    status.online                = 1U;
    status.gyro_auto_cal_enabled = 0U;
    status.gyro_calibrated       = 1U;
    sample.accel_g[2]            = 1.0f;
    sample.gyro_dps[0]           = 10.0f;
    require_int(ImuEstimationPipeline_BeginCalibration(2U, 1U, 0U) != 0U,
                "manual recalibration starts from calibrated state with auto calibration disabled");

    status.sample_count = 1U;
    sample.timestamp_ms = 1U;
    ImuEstimationPipeline_Process(&sample, &device, &status);
    require_int(fabsf(status.gyro_corrected_dps[0] - 1.0f) < 0.0001f,
                "persisted bias is applied before manual recalibration");
    ImuEstimationPipeline_ServiceCalibration(1U, 1U, &status);
    require_int(status.gyro_cal_sample_count == 1U, "manual recalibration collects first sample");
    status.sample_count = 2U;
    sample.timestamp_ms = 2U;
    ImuEstimationPipeline_Process(&sample, &device, &status);
    ImuEstimationPipeline_ServiceCalibration(2U, 1U, &status);
    require_int(status.gyro_auto_cal_state == STATE_ESTIMATION_IMU_AUTO_CAL_SUCCESS, "manual recalibration completes");
    require_int(status.filter_initialized == 0U && status.quaternion[0] == 1.0f && status.roll_deg == 0.0f
                    && status.pitch_deg == 0.0f && status.yaw_deg == 0.0f,
                "successful manual calibration resets attitude and filter runtime");
    ImuEstimationPipeline_GetCalibration(&calibration);
    require_int(fabsf(calibration.gyro_bias_dps[0] - 10.0f) < 0.0001f,
                "manual recalibration adds residual to persisted bias");
    status.sample_count = 3U;
    sample.timestamp_ms = 3U;
    ImuEstimationPipeline_Process(&sample, &device, &status);
    require_int(fabsf(status.gyro_corrected_dps[0]) < 0.0001f, "updated absolute bias removes the stationary raw rate");
    require_int(ImuEstimationPipeline_BeginCalibration(1U, 1U, 0U) != 0U,
                "manual recalibration can start again after completion");

    ImuEstimationPipeline_ClearCalibration();
    status                       = (state_estimation_imu_status_t){0};
    status.enabled               = 1U;
    status.online                = 1U;
    status.gyro_auto_cal_enabled = 1U;
    sample.gyro_dps[0]           = 2.0f;
    require_int(ImuEstimationPipeline_BeginCalibration(2U, 1U, 1U) != 0U,
                "automatic calibration starts from default bias");
    for (uint32_t count = 1U; count <= 2U; ++count)
    {
        status.sample_count = count;
        sample.timestamp_ms = count;
        ImuEstimationPipeline_Process(&sample, &device, &status);
        ImuEstimationPipeline_ServiceCalibration(count, 1U, &status);
    }
    ImuEstimationPipeline_GetCalibration(&calibration);
    require_int(fabsf(calibration.gyro_bias_dps[0] - 2.0f) < 0.0001f,
                "automatic calibration from default bias retains absolute-bias semantics");
    require_int(status.filter_initialized == 0U && status.quaternion[0] == 1.0f && status.yaw_deg == 0.0f,
                "successful automatic calibration resets attitude runtime");

    ImuEstimationPipeline_ResetRuntime(5000U);
    status = (state_estimation_imu_status_t){
        .enabled               = 1U,
        .online                = 1U,
        .gyro_auto_cal_enabled = 1U,
        .sample_count          = 10U,
    };
    ImuEstimationPipeline_ServiceCalibration(5000U, 1U, &status);
    require_int(status.gyro_auto_cal_state == STATE_ESTIMATION_IMU_AUTO_CAL_RETRY_WAIT,
                "runtime reset delays auto calibration relative to current time");
    status.sample_count++;
    ImuEstimationPipeline_ServiceCalibration(5999U, 1U, &status);
    require_int(status.gyro_auto_cal_state == STATE_ESTIMATION_IMU_AUTO_CAL_RETRY_WAIT,
                "auto calibration remains delayed before the relative deadline");
    status.sample_count++;
    ImuEstimationPipeline_ServiceCalibration(6000U, 1U, &status);
    require_int(status.gyro_auto_cal_state == STATE_ESTIMATION_IMU_AUTO_CAL_COLLECTING,
                "auto calibration starts at the relative deadline");

    (void)puts("imu estimation pipeline tests passed");
    return 0;
}
