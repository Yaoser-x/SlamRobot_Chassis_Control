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

int main(void)
{
    imu_calibration_t             calibration;
    bmi270_driver_state_t         device = {0};
    bmi270_sample_t               sample = {0};
    state_estimation_imu_status_t status = {0};

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
