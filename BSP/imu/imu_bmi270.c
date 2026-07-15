#include "imu_bmi270.h"

#include "bmi270_bus.h"
#include "bmi270_device.h"
#include "bmi270_registers.h"
#include "imu_calibration.h"
#include "imu_filter.h"
#include "imu_bmi270_fifo.h"
#include "imu_bmi270_math.h"
#include "imu_timestamp.h"
#include "main.h"

#include <math.h>

#define BMI270_SPI_SELECT_DELAY_MS          1U
#define BMI270_INIT_RETRY_MS                1000U
#define BMI270_FIFO_READ_MAX_BYTES          128U
#define BMI270_FIFO_MAX_SAMPLES             8U
#define BMI270_ACCEL_LSB_PER_G              16384.0f
#define BMI270_GYRO_LSB_PER_DPS             65.6f
#define BMI270_ACCEL_FILTER_ALPHA           0.20f
#define BMI270_GYRO_FILTER_ALPHA            0.20f
#define BMI270_ATTITUDE_MAX_DT_S            0.100f
#define BMI270_DIRECT_FALLBACK_DT_S         0.010f
#define BMI270_GYRO_SATURATION_DPS          490.0f
#define BMI270_ACCEL_REJECT_MIN_G           0.40f
#define BMI270_ACCEL_REJECT_MAX_G           1.80f
#define BMI270_GYRO_CAL_DEFAULT_SAMPLES     500U
#define BMI270_GYRO_CAL_MIN_SAMPLES         50U
#define BMI270_GYRO_CAL_MAX_SAMPLES         2000U
#define BMI270_GYRO_CAL_DEFAULT_DELAY_MS    10U
#define BMI270_GYRO_CAL_STILL_SPAN_DPS      5.0f
#define BMI270_GYRO_CAL_MAX_ABS_DPS         20.0f
#define BMI270_GYRO_AUTO_CAL_ENABLED        1U
#define BMI270_GYRO_AUTO_CAL_SAMPLES        500U
#define BMI270_GYRO_AUTO_CAL_DELAY_MS       10U
#define BMI270_GYRO_AUTO_CAL_START_DELAY_MS 1000U
#define BMI270_GYRO_AUTO_CAL_RETRY_MS       2000U
#define BMI270_GYRO_AUTO_CAL_MAX_ATTEMPTS   5U

static imu_bmi270_state_t                imu_state;
static uint32_t                          imu_next_init_retry_ms;
static uint32_t                          imu_gyro_auto_cal_next_ms;
static imu_bmi270_profile_id_t           imu_selected_profile = IMU_BMI270_PROFILE_PERFORMANCE;
static imu_bmi270_calibration_t          imu_calibration;
static imu_bmi270_mahony_t               imu_fusion;
static imu_bmi270_mahony_params_t        imu_fusion_params;
static volatile uint8_t                  imu_gyro_calibration_active;
static imu_bmi270_gyro_cal_accumulator_t imu_gyro_cal_accumulator;
static uint8_t                           imu_gyro_calibration_is_auto;
static uint32_t                          imu_gyro_cal_last_imu_sample_count;

static void ImuBmi270_ServiceAutoCal(uint32_t now_ms, uint8_t stationary);

static int16_t ImuBmi270_ReadI16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static float ImuBmi270_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void ImuBmi270_UpdateGyroCalDiag(uint8_t     reason,
                                        uint8_t     axis,
                                        const float sum_dps[3],
                                        const float min_dps[3],
                                        const float max_dps[3],
                                        uint16_t    sample_count)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    imu_state.gyro_cal_fail_reason  = reason;
    imu_state.gyro_cal_fail_axis    = axis;
    imu_state.gyro_cal_sample_count = sample_count;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if (sum_dps != 0 && min_dps != 0 && max_dps != 0 && sample_count != 0U)
        {
            imu_state.gyro_cal_mean_dps[i] = sum_dps[i] / (float)sample_count;
            imu_state.gyro_cal_min_dps[i]  = min_dps[i];
            imu_state.gyro_cal_max_dps[i]  = max_dps[i];
            imu_state.gyro_cal_span_dps[i] = max_dps[i] - min_dps[i];
        }
        else
        {
            imu_state.gyro_cal_mean_dps[i] = 0.0f;
            imu_state.gyro_cal_min_dps[i]  = 0.0f;
            imu_state.gyro_cal_max_dps[i]  = 0.0f;
            imu_state.gyro_cal_span_dps[i] = 0.0f;
        }
    }
    __set_PRIMASK(primask);
}

static void ImuBmi270_ScheduleAutoCal(uint32_t now_ms, uint32_t delay_ms)
{
    imu_gyro_auto_cal_next_ms = now_ms + delay_ms;
    if (imu_state.gyro_auto_cal_enabled != 0U && imu_state.gyro_calibrated == 0U)
    {
        imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_WAIT;
    }
}

static void ImuBmi270_SetError(uint8_t error)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    imu_state.last_error = error;
    if (error != IMU_BMI270_ERROR_NONE)
    {
        imu_state.error_count++;
        if (error == IMU_BMI270_ERROR_SPI)
        {
            imu_state.quality_flags |= IMU_BMI270_QUALITY_SPI_ERROR;
            imu_state.quality_latched_flags |= IMU_BMI270_QUALITY_SPI_ERROR;
            imu_state.spi_error_count++;
        }
        else if (error == IMU_BMI270_ERROR_CONFIG || error == IMU_BMI270_ERROR_CHIP_ID)
        {
            imu_state.quality_flags |= IMU_BMI270_QUALITY_INIT_FAILED;
            imu_state.quality_latched_flags |= IMU_BMI270_QUALITY_INIT_FAILED;
            imu_state.init_failure_count++;
        }
        else if (error == IMU_BMI270_ERROR_FIFO)
        {
            imu_state.quality_flags |= IMU_BMI270_QUALITY_FIFO_OVERFLOW;
            imu_state.quality_latched_flags |= IMU_BMI270_QUALITY_FIFO_OVERFLOW;
            imu_state.fifo_overflow_count++;
        }
        else if (error == IMU_BMI270_ERROR_TIMESTAMP)
        {
            imu_state.quality_flags |= IMU_BMI270_QUALITY_TIMESTAMP_ERROR;
            imu_state.quality_latched_flags |= IMU_BMI270_QUALITY_TIMESTAMP_ERROR;
            imu_state.timestamp_error_count++;
        }
        else if (error == IMU_BMI270_ERROR_PROFILE_VERIFY)
        {
            imu_state.quality_flags |= IMU_BMI270_QUALITY_PROFILE_MISMATCH;
            imu_state.quality_latched_flags |= IMU_BMI270_QUALITY_PROFILE_MISMATCH;
        }
    }
    __set_PRIMASK(primask);
}

static void ImuBmi270_SetQuality(uint32_t flags)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    imu_state.quality_flags |= flags;
    imu_state.quality_latched_flags |= flags;
    if ((flags & IMU_BMI270_QUALITY_GYRO_SATURATION) != 0UL)
    {
        imu_state.gyro_saturation_count++;
    }
    if ((flags & IMU_BMI270_QUALITY_ACCEL_ANOMALY) != 0UL)
    {
        imu_state.accel_anomaly_count++;
    }
    if ((flags & IMU_BMI270_QUALITY_ATTITUDE_INVALID) != 0UL)
    {
        imu_state.attitude_invalid_count++;
    }
    if ((flags & IMU_BMI270_QUALITY_POLL_FALLBACK) != 0UL)
    {
        imu_state.poll_fallback_count++;
    }
    __set_PRIMASK(primask);
}

static void ImuBmi270_ClearTransientQuality(void)
{
    imu_state.quality_flags &=
        ~(IMU_BMI270_QUALITY_SPI_ERROR | IMU_BMI270_QUALITY_INIT_FAILED | IMU_BMI270_QUALITY_FIFO_OVERFLOW
          | IMU_BMI270_QUALITY_TIMESTAMP_ERROR | IMU_BMI270_QUALITY_GYRO_SATURATION | IMU_BMI270_QUALITY_ACCEL_ANOMALY
          | IMU_BMI270_QUALITY_ATTITUDE_INVALID | IMU_BMI270_QUALITY_POLL_FALLBACK | IMU_BMI270_QUALITY_PROFILE_MISMATCH
          | IMU_BMI270_QUALITY_TEMPERATURE_INVALID);
}

uint8_t ImuBmi270_ReadReg(uint8_t reg, uint8_t *value)
{
    if (Bmi270Bus_ReadReg(reg, value) == 0U)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_SPI);
        return 0U;
    }
    return 1U;
}

uint8_t ImuBmi270_Diagnose(imu_bmi270_diag_t *diag)
{
    return Bmi270Bus_RunRecoveryProbe(BMI270_REG_CHIP_ID, diag);
}

static uint8_t ImuBmi270_ReadBytes(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (Bmi270Bus_ReadBytes(reg, data, len) == 0U)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_SPI);
        return 0U;
    }
    return 1U;
}

static uint8_t ImuBmi270_ReadRawFrame(int16_t accel_raw[3], int16_t gyro_raw[3])
{
    uint8_t data[12];

    if (accel_raw == 0 || gyro_raw == 0)
    {
        return 0U;
    }
    if (ImuBmi270_ReadBytes(BMI270_REG_DATA_8, data, sizeof(data)) == 0U)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_READ);
        return 0U;
    }

    accel_raw[0] = ImuBmi270_ReadI16(&data[0]);
    accel_raw[1] = ImuBmi270_ReadI16(&data[2]);
    accel_raw[2] = ImuBmi270_ReadI16(&data[4]);
    gyro_raw[0]  = ImuBmi270_ReadI16(&data[6]);
    gyro_raw[1]  = ImuBmi270_ReadI16(&data[8]);
    gyro_raw[2]  = ImuBmi270_ReadI16(&data[10]);
    if (ImuBmi270_RawFrameHasSignal(accel_raw, gyro_raw) == 0U)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_INVALID_FRAME);
        return 0U;
    }
    return 1U;
}

static void ImuBmi270_UpdateTemperature(void)
{
    uint8_t  data[2];
    float    temperature_c;
    uint32_t primask;

    if (ImuBmi270_ReadBytes(BMI270_REG_TEMP_0, data, sizeof(data)) == 0U
        || ImuBmi270_TemperatureRawToC(ImuBmi270_ReadI16(data), &temperature_c) == 0U)
    {
        imu_state.temperature_valid = 0U;
        ImuBmi270_SetQuality(IMU_BMI270_QUALITY_TEMPERATURE_INVALID);
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    imu_state.temperature_c     = temperature_c;
    imu_state.temperature_valid = 1U;
    imu_state.quality_flags &= ~IMU_BMI270_QUALITY_TEMPERATURE_INVALID;
    __set_PRIMASK(primask);
}

static uint8_t ImuBmi270_ReadSensorTime(uint32_t *sensor_time)
{
    uint8_t data[3];

    if (sensor_time == 0)
    {
        return 0U;
    }
    if (ImuBmi270_ReadBytes(BMI270_REG_SENSORTIME_0, data, sizeof(data)) == 0U)
    {
        return 0U;
    }

    *sensor_time = ((uint32_t)data[0]) | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
    return 1U;
}

static uint8_t ImuBmi270_ReadFifoLength(uint16_t *fifo_len)
{
    uint8_t data[2];

    if (fifo_len == 0)
    {
        return 0U;
    }
    if (ImuBmi270_ReadBytes(BMI270_REG_FIFO_LENGTH_0, data, sizeof(data)) == 0U)
    {
        return 0U;
    }

    *fifo_len = (uint16_t)((((uint16_t)data[1] & 0x3FU) << 8) | data[0]);
    return 1U;
}

uint8_t ImuBmi270_WriteReg(uint8_t reg, uint8_t value)
{
    if (Bmi270Bus_WriteReg(reg, value) == 0U)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_SPI);
        return 0U;
    }
    return 1U;
}

static uint8_t ImuBmi270_WriteBytes(uint8_t reg, const uint8_t *data, uint8_t len)
{
    if (Bmi270Bus_WriteBytes(reg, data, len) == 0U)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_SPI);
        return 0U;
    }
    return 1U;
}

static const bmi270_device_io_t imu_device_io = {
    .read_reg    = ImuBmi270_ReadReg,
    .write_reg   = ImuBmi270_WriteReg,
    .write_bytes = ImuBmi270_WriteBytes,
};

static uint8_t ImuBmi270_HandleDeviceStatus(bmi270_device_status_t status)
{
    if (status == BMI270_DEVICE_CONFIG_ERROR)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_CONFIG);
    }
    else if (status == BMI270_DEVICE_PROFILE_MISMATCH)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_PROFILE_VERIFY);
    }
    return (status == BMI270_DEVICE_OK) ? 1U : 0U;
}

static uint8_t ImuBmi270_LoadConfigFile(void)
{
    return ImuBmi270_HandleDeviceStatus(Bmi270Device_LoadConfig(&imu_device_io));
}

static uint8_t ImuBmi270_WaitInitOk(void)
{
    return ImuBmi270_HandleDeviceStatus(Bmi270Device_WaitInitOk(&imu_device_io));
}

static uint8_t ImuBmi270_ApplyProfile(const imu_bmi270_profile_t *profile)
{
    return ImuBmi270_HandleDeviceStatus(Bmi270Device_ApplyProfile(&imu_device_io, profile));
}

void ImuBmi270_Init(void)
{
    imu_state                       = (imu_bmi270_state_t){0};
    imu_state.enabled               = 1U;
    imu_state.profile               = (uint8_t)imu_selected_profile;
    imu_state.init_state            = IMU_BMI270_INIT_STATE_RESET;
    imu_state.quaternion[0]         = 1.0f;
    imu_state.gyro_auto_cal_enabled = BMI270_GYRO_AUTO_CAL_ENABLED;
    imu_state.gyro_auto_cal_state =
        (BMI270_GYRO_AUTO_CAL_ENABLED != 0U) ? IMU_BMI270_GYRO_AUTO_CAL_WAIT : IMU_BMI270_GYRO_AUTO_CAL_DISABLED;
    imu_next_init_retry_ms             = 0U;
    imu_gyro_auto_cal_next_ms          = 0U;
    imu_gyro_calibration_active        = 0U;
    imu_gyro_calibration_is_auto       = 0U;
    imu_gyro_cal_last_imu_sample_count = 0UL;
    ImuBmi270GyroCalAccumulator_Init(&imu_gyro_cal_accumulator);
    ImuBmi270Calibration_Default(&imu_calibration);
    imu_fusion_params = ImuBmi270Mahony_DefaultParams();
    ImuBmi270Mahony_Init(&imu_fusion);
    Bmi270Bus_Deselect();
}

uint8_t ImuBmi270_SetEnabled(uint8_t enabled)
{
    imu_state.enabled    = (enabled != 0U) ? 1U : 0U;
    imu_state.init_state = (enabled != 0U) ? imu_state.init_state : IMU_BMI270_INIT_STATE_DISABLED;
    if (enabled == 0U)
    {
        imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_DISABLED;
    }
    else if (imu_state.gyro_auto_cal_enabled != 0U && imu_state.gyro_calibrated == 0U)
    {
        ImuBmi270_ScheduleAutoCal(HAL_GetTick(), BMI270_GYRO_AUTO_CAL_START_DELAY_MS);
    }
    return 1U;
}

uint8_t ImuBmi270_SetProfile(imu_bmi270_profile_id_t profile)
{
    if (ImuBmi270Profile_Get(profile) == 0)
    {
        return 0U;
    }
    imu_selected_profile             = profile;
    imu_state.profile                = (uint8_t)profile;
    imu_state.online                 = 0U;
    imu_state.filter_initialized     = 0U;
    imu_state.gyro_calibrated        = 0U;
    imu_state.gyro_auto_cal_attempts = 0U;
    ImuBmi270_ScheduleAutoCal(HAL_GetTick(), BMI270_GYRO_AUTO_CAL_START_DELAY_MS);
    return 1U;
}

uint8_t ImuBmi270_ProbeNow(void)
{
    uint8_t chip_id = 0U;

    (void)ImuBmi270_ReadReg(BMI270_REG_CHIP_ID, &chip_id);
    HAL_Delay(BMI270_SPI_SELECT_DELAY_MS);
    if (ImuBmi270_ReadReg(BMI270_REG_CHIP_ID, &chip_id) == 0U)
    {
        return 0U;
    }

    imu_state.chip_id = chip_id;
    if (chip_id != BMI270_CHIP_ID)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_CHIP_ID);
        return 0U;
    }

    ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
    return 1U;
}

uint8_t ImuBmi270_ConfigNow(void)
{
    const imu_bmi270_profile_t *profile = ImuBmi270Profile_Get(imu_selected_profile);

    imu_state.online            = 0U;
    imu_state.sensor_time_valid = 0U;
    imu_state.init_state        = IMU_BMI270_INIT_STATE_PROBE;
    if (ImuBmi270_WriteReg(BMI270_REG_CMD, BMI270_CMD_SOFT_RESET) == 0U)
    {
        return 0U;
    }
    HAL_Delay(5U);
    if (ImuBmi270_ProbeNow() == 0U)
    {
        return 0U;
    }
    imu_state.online = 0U;

    if (ImuBmi270_WriteReg(BMI270_REG_PWR_CONF, profile->pwr_conf) == 0U)
    {
        return 0U;
    }
    HAL_Delay(BMI270_SPI_SELECT_DELAY_MS);

    imu_state.init_state = IMU_BMI270_INIT_STATE_LOAD_CONFIG;
    if (ImuBmi270_LoadConfigFile() == 0U)
    {
        return 0U;
    }
    if (ImuBmi270_WaitInitOk() == 0U)
    {
        return 0U;
    }
    imu_state.init_state = IMU_BMI270_INIT_STATE_VERIFY_PROFILE;
    if (ImuBmi270_ApplyProfile(profile) == 0U)
    {
        return 0U;
    }
    imu_state.enabled            = 1U;
    imu_state.online             = 1U;
    imu_state.filter_initialized = 0U;
    imu_state.profile            = (uint8_t)imu_selected_profile;
    imu_state.init_state         = IMU_BMI270_INIT_STATE_SAMPLING;
    ImuBmi270Mahony_Init(&imu_fusion);
    if (imu_state.gyro_calibrated == 0U)
    {
        imu_state.gyro_auto_cal_attempts = 0U;
        ImuBmi270_ScheduleAutoCal(HAL_GetTick(), BMI270_GYRO_AUTO_CAL_START_DELAY_MS);
    }
    ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
    return 1U;
}

static void ImuBmi270_ProcessMeasurement(const int16_t accel_raw[3],
                                         const int16_t gyro_raw[3],
                                         uint32_t      sensor_time,
                                         uint8_t       sensor_time_valid,
                                         float         fallback_dt_s)
{
    uint32_t primask;
    uint32_t update_ms = HAL_GetTick();
    float    sensor_accel_g[3];
    float    body_accel_g[3];
    float    ros_accel_g[3];
    float    gyro_raw_dps[3];
    float    gyro_corrected_dps[3];
    float    body_gyro_dps[3];
    float    ros_gyro_dps[3];
    float    euler_deg[3];
    float    dt_s     = fallback_dt_s;
    uint8_t  dt_valid = 0U;
    float    accel_norm;

    primask = __get_PRIMASK();
    __disable_irq();
    ImuBmi270_ClearTransientQuality();
    __set_PRIMASK(primask);

    for (uint8_t i = 0U; i < 3U; ++i)
    {
        sensor_accel_g[i] = (((float)accel_raw[i] / BMI270_ACCEL_LSB_PER_G) - imu_calibration.accel_bias_g[i])
                            * imu_calibration.accel_scale[i];
        gyro_raw_dps[i]       = (float)gyro_raw[i] / BMI270_GYRO_LSB_PER_DPS;
        gyro_corrected_dps[i] = gyro_raw_dps[i]
                                - ImuBmi270Calibration_GyroBiasAtTemperature(&imu_calibration,
                                                                             i,
                                                                             imu_state.temperature_c,
                                                                             imu_state.temperature_valid);
        if (ImuBmi270_AbsFloat(gyro_raw_dps[i]) >= BMI270_GYRO_SATURATION_DPS)
        {
            ImuBmi270_SetQuality(IMU_BMI270_QUALITY_GYRO_SATURATION);
        }
    }

    ImuBmi270Coordinate_Apply(imu_calibration.sensor_to_body, sensor_accel_g, body_accel_g);
    ImuBmi270Coordinate_Apply(imu_calibration.sensor_to_body, gyro_corrected_dps, body_gyro_dps);
    ImuBmi270Coordinate_BodyToRos(body_accel_g, ros_accel_g);
    ImuBmi270Coordinate_BodyToRos(body_gyro_dps, ros_gyro_dps);

    accel_norm = sqrtf((body_accel_g[0] * body_accel_g[0]) + (body_accel_g[1] * body_accel_g[1])
                       + (body_accel_g[2] * body_accel_g[2]));
    if (accel_norm < BMI270_ACCEL_REJECT_MIN_G || accel_norm > BMI270_ACCEL_REJECT_MAX_G)
    {
        ImuBmi270_SetQuality(IMU_BMI270_QUALITY_ACCEL_ANOMALY);
    }

    if (sensor_time_valid != 0U && imu_state.sensor_time_valid != 0U)
    {
        if (ImuBmi270Time_DeltaSeconds(sensor_time, imu_state.sensor_time, BMI270_ATTITUDE_MAX_DT_S, &dt_s) != 0U)
        {
            dt_valid = 1U;
        }
        else
        {
            ImuBmi270_SetError(IMU_BMI270_ERROR_TIMESTAMP);
            ImuBmi270_SetQuality(IMU_BMI270_QUALITY_ATTITUDE_INVALID);
        }
    }
    else if (fallback_dt_s > 0.0f)
    {
        dt_valid = 1U;
        ImuBmi270_SetQuality(IMU_BMI270_QUALITY_POLL_FALLBACK);
    }

    if (dt_valid != 0U)
    {
        ImuBmi270Mahony_Update(&imu_fusion, body_gyro_dps, body_accel_g, dt_s, &imu_fusion_params);
        if ((imu_fusion.status_flags & IMU_BMI270_FUSION_ACCEL_DEGRADED) != 0UL)
        {
            ImuBmi270_SetQuality(IMU_BMI270_QUALITY_ACCEL_ANOMALY);
        }
        if ((imu_fusion.status_flags & IMU_BMI270_FUSION_INVALID_DT) != 0UL)
        {
            ImuBmi270_SetQuality(IMU_BMI270_QUALITY_ATTITUDE_INVALID);
        }
    }
    ImuBmi270Quaternion_ToEulerDeg(&imu_fusion.q, euler_deg);

    primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        imu_state.accel_raw[i]          = accel_raw[i];
        imu_state.gyro_raw[i]           = gyro_raw[i];
        imu_state.gyro_corrected_dps[i] = gyro_corrected_dps[i];
    }
    if (imu_state.filter_initialized == 0U)
    {
        for (uint8_t i = 0U; i < 3U; ++i)
        {
            imu_state.accel_g[i]           = sensor_accel_g[i];
            imu_state.body_accel_g[i]      = body_accel_g[i];
            imu_state.ros_accel_g[i]       = ros_accel_g[i];
            imu_state.gyro_filtered_dps[i] = body_gyro_dps[i];
            imu_state.gyro_dps[i]          = body_gyro_dps[i];
            imu_state.body_gyro_dps[i]     = body_gyro_dps[i];
            imu_state.ros_gyro_dps[i]      = ros_gyro_dps[i];
        }
        imu_state.filter_initialized = 1U;
    }
    else
    {
        for (uint8_t i = 0U; i < 3U; ++i)
        {
            imu_state.accel_g[i] =
                ImuFilter_LowPass(imu_state.accel_g[i], sensor_accel_g[i], BMI270_ACCEL_FILTER_ALPHA);
            imu_state.body_accel_g[i] =
                ImuFilter_LowPass(imu_state.body_accel_g[i], body_accel_g[i], BMI270_ACCEL_FILTER_ALPHA);
            imu_state.ros_accel_g[i] =
                ImuFilter_LowPass(imu_state.ros_accel_g[i], ros_accel_g[i], BMI270_ACCEL_FILTER_ALPHA);
            imu_state.gyro_filtered_dps[i] =
                ImuFilter_LowPass(imu_state.gyro_filtered_dps[i], body_gyro_dps[i], BMI270_GYRO_FILTER_ALPHA);
            imu_state.gyro_dps[i]      = imu_state.gyro_filtered_dps[i];
            imu_state.body_gyro_dps[i] = imu_state.gyro_filtered_dps[i];
            imu_state.ros_gyro_dps[i] =
                ImuFilter_LowPass(imu_state.ros_gyro_dps[i], ros_gyro_dps[i], BMI270_GYRO_FILTER_ALPHA);
        }
    }
    imu_state.quaternion[0]           = imu_fusion.q.w;
    imu_state.quaternion[1]           = imu_fusion.q.x;
    imu_state.quaternion[2]           = imu_fusion.q.y;
    imu_state.quaternion[3]           = imu_fusion.q.z;
    imu_state.roll_deg                = euler_deg[0];
    imu_state.pitch_deg               = euler_deg[1];
    imu_state.yaw_deg                 = ImuFilter_WrapAngleDeg(euler_deg[2]);
    imu_state.accel_correction_weight = imu_fusion.accel_weight;
    imu_state.sensor_time             = sensor_time & IMU_BMI270_SENSOR_TIME_MASK;
    imu_state.sensor_time_valid       = sensor_time_valid;
    imu_state.sample_count++;
    imu_state.last_update_ms = update_ms;
    imu_state.online         = 1U;
    imu_state.init_state     = IMU_BMI270_INIT_STATE_SAMPLING;
    __set_PRIMASK(primask);
}

static uint8_t ImuBmi270_UpdateDirect(float fallback_dt_s)
{
    int16_t  accel_raw[3];
    int16_t  gyro_raw[3];
    uint32_t sensor_time       = 0UL;
    uint8_t  sensor_time_valid = 0U;

    if (ImuBmi270_ReadRawFrame(accel_raw, gyro_raw) == 0U)
    {
        return 0U;
    }
    sensor_time_valid = ImuBmi270_ReadSensorTime(&sensor_time);
    ImuBmi270_ProcessMeasurement(accel_raw, gyro_raw, sensor_time, sensor_time_valid, fallback_dt_s);
    return 1U;
}

static uint8_t ImuBmi270_UpdateFifo(void)
{
    uint16_t                       fifo_len = 0U;
    uint8_t                        fifo[BMI270_FIFO_READ_MAX_BYTES];
    imu_bmi270_fifo_sample_t       samples[BMI270_FIFO_MAX_SAMPLES];
    imu_bmi270_fifo_parse_result_t parse;
    uint32_t                       last_time;
    uint8_t                        any_processed = 0U;

    if (ImuBmi270_ReadFifoLength(&fifo_len) == 0U)
    {
        return 0U;
    }
    if (fifo_len == 0U)
    {
        return 0U;
    }
    if (fifo_len > BMI270_FIFO_READ_MAX_BYTES)
    {
        fifo_len = BMI270_FIFO_READ_MAX_BYTES;
        ImuBmi270_SetError(IMU_BMI270_ERROR_FIFO);
    }
    if (ImuBmi270_ReadBytes(BMI270_REG_FIFO_DATA, fifo, (uint8_t)fifo_len) == 0U)
    {
        return 0U;
    }
    if (ImuBmi270Fifo_Parse(fifo, fifo_len, samples, BMI270_FIFO_MAX_SAMPLES, &parse) == 0U)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_FIFO);
        return 0U;
    }
    if ((parse.flags & IMU_BMI270_FIFO_PARSE_SKIP_FRAME) != 0UL)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_FIFO);
    }

    last_time = parse.sensor_time & IMU_BMI270_SENSOR_TIME_MASK;
    for (uint32_t i = 0UL; i < parse.sample_count && i < BMI270_FIFO_MAX_SAMPLES; ++i)
    {
        uint32_t sample_time       = last_time;
        uint8_t  sample_time_valid = parse.sensor_time_valid;

        if (parse.sensor_time_valid != 0U && parse.sample_count > 0UL)
        {
            uint32_t remaining = (parse.sample_count - 1UL) - i;
            sample_time = (last_time - (remaining * IMU_BMI270_SENSOR_TIME_100HZ_TICKS)) & IMU_BMI270_SENSOR_TIME_MASK;
        }
        if (samples[i].accel_valid != 0U && samples[i].gyro_valid != 0U)
        {
            if (ImuBmi270_RawFrameHasSignal(samples[i].accel_raw, samples[i].gyro_raw) != 0U)
            {
                ImuBmi270_ProcessMeasurement(samples[i].accel_raw,
                                             samples[i].gyro_raw,
                                             sample_time,
                                             sample_time_valid,
                                             0.0f);
                any_processed = 1U;
            }
            else
            {
                ImuBmi270_SetError(IMU_BMI270_ERROR_INVALID_FRAME);
            }
        }
    }

    return any_processed;
}

uint8_t ImuBmi270_Update(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (imu_state.enabled == 0U)
    {
        return 1U;
    }
    if (imu_state.online == 0U)
    {
        if ((int32_t)(now_ms - imu_next_init_retry_ms) < 0)
        {
            imu_state.init_state = IMU_BMI270_INIT_STATE_RETRY_WAIT;
            return 0U;
        }
        if (ImuBmi270_ConfigNow() == 0U)
        {
            imu_next_init_retry_ms = now_ms + BMI270_INIT_RETRY_MS;
            imu_state.init_state   = IMU_BMI270_INIT_STATE_RETRY_WAIT;
            return 0U;
        }
    }

    if (ImuBmi270_UpdateFifo() != 0U)
    {
        ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
        ImuBmi270_UpdateTemperature();
        return 1U;
    }

    if (ImuBmi270_UpdateDirect(BMI270_DIRECT_FALLBACK_DT_S) == 0U)
    {
        return 0U;
    }

    ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
    ImuBmi270_UpdateTemperature();
    return 1U;
}

void ImuBmi270_OnDataReadyFromIsr(void)
{
    imu_state.drdy_count++;
}

static uint8_t ImuBmi270_BeginGyroCalibrationRequest(uint16_t samples, uint16_t delay_ms, uint8_t is_auto)
{
    if (imu_gyro_calibration_active != 0U)
    {
        return 0U;
    }
    if (samples == 0U)
    {
        samples = BMI270_GYRO_CAL_DEFAULT_SAMPLES;
    }
    if (delay_ms == 0U)
    {
        delay_ms = BMI270_GYRO_CAL_DEFAULT_DELAY_MS;
    }
    if (samples < BMI270_GYRO_CAL_MIN_SAMPLES)
    {
        samples = BMI270_GYRO_CAL_MIN_SAMPLES;
    }
    if (samples > BMI270_GYRO_CAL_MAX_SAMPLES)
    {
        samples = BMI270_GYRO_CAL_MAX_SAMPLES;
    }
    if (ImuBmi270GyroCalAccumulator_Begin(&imu_gyro_cal_accumulator, samples, delay_ms) == 0U)
    {
        return 0U;
    }
    imu_gyro_calibration_active         = 1U;
    imu_gyro_calibration_is_auto        = (is_auto != 0U) ? 1U : 0U;
    imu_gyro_cal_last_imu_sample_count  = imu_state.sample_count;
    imu_state.gyro_auto_cal_last_result = 0U;
    imu_state.gyro_auto_cal_state       = IMU_BMI270_GYRO_AUTO_CAL_WAIT;
    ImuBmi270_UpdateGyroCalDiag(IMU_BMI270_GYRO_CAL_FAIL_NONE, 0xFFU, 0, 0, 0, 0U);
    return 1U;
}

static void
ImuBmi270_FinishGyroCalibrationFailure(uint32_t now_ms, uint8_t reason, uint8_t axis, uint8_t consume_attempt)
{
    ImuBmi270_UpdateGyroCalDiag(reason,
                                axis,
                                imu_gyro_cal_accumulator.sum_dps,
                                imu_gyro_cal_accumulator.min_dps,
                                imu_gyro_cal_accumulator.max_dps,
                                imu_gyro_cal_accumulator.sample_count);
    imu_gyro_calibration_active         = 0U;
    imu_state.gyro_auto_cal_last_result = 0U;
    if (imu_gyro_calibration_is_auto != 0U)
    {
        if (consume_attempt != 0U && imu_state.gyro_auto_cal_attempts < 0xFFU)
        {
            imu_state.gyro_auto_cal_attempts++;
        }
        if (imu_state.gyro_auto_cal_attempts >= BMI270_GYRO_AUTO_CAL_MAX_ATTEMPTS)
        {
            imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_FAILED;
        }
        else
        {
            ImuBmi270_ScheduleAutoCal(now_ms, BMI270_GYRO_AUTO_CAL_RETRY_MS);
            imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_RETRY_WAIT;
        }
    }
    else
    {
        imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_FAILED;
    }
    ImuBmi270GyroCalAccumulator_Init(&imu_gyro_cal_accumulator);
}

static void ImuBmi270_ServiceAutoCal(uint32_t now_ms, uint8_t stationary)
{
    if (imu_state.gyro_calibrated != 0U)
    {
        if (imu_state.gyro_auto_cal_enabled != 0U)
        {
            imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_DONE;
        }
        return;
    }
    if (imu_gyro_calibration_active != 0U)
    {
        return;
    }
    if (ImuBmi270_AutoCalDue(imu_state.gyro_auto_cal_enabled,
                             imu_state.online,
                             imu_state.gyro_calibrated,
                             imu_state.gyro_auto_cal_attempts,
                             BMI270_GYRO_AUTO_CAL_MAX_ATTEMPTS,
                             now_ms,
                             imu_gyro_auto_cal_next_ms)
        == 0U)
    {
        return;
    }
    if (stationary == 0U)
    {
        imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_WAIT;
        return;
    }
    if (ImuBmi270_BeginGyroCalibrationRequest(BMI270_GYRO_AUTO_CAL_SAMPLES, BMI270_GYRO_AUTO_CAL_DELAY_MS, 1U) != 0U)
    {
        imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_RUNNING;
    }
}

uint8_t ImuBmi270_CalibrateGyro(uint16_t samples, uint16_t delay_ms)
{
    return ImuBmi270_BeginGyroCalibrationRequest(samples, delay_ms, 0U);
}

void ImuBmi270_ServiceCalibration(uint32_t now_ms, uint8_t stationary)
{
    float                           accel_g[3];
    float                           gyro_dps[3];
    float                           bias_dps[3];
    float                           accel_mean_g[3];
    imu_bmi270_gyro_cal_acc_state_t result;

    ImuBmi270_ServiceAutoCal(now_ms, stationary);
    if (imu_gyro_calibration_active == 0U)
    {
        return;
    }
    if (stationary == 0U)
    {
        ImuBmi270_FinishGyroCalibrationFailure(now_ms, IMU_BMI270_GYRO_CAL_FAIL_MOTION, 0xFFU, 0U);
        return;
    }
    if (imu_state.sample_count == imu_gyro_cal_last_imu_sample_count)
    {
        return;
    }
    imu_gyro_cal_last_imu_sample_count = imu_state.sample_count;
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        accel_g[axis] =
            (((float)imu_state.accel_raw[axis] / BMI270_ACCEL_LSB_PER_G) - imu_calibration.accel_bias_g[axis])
            * imu_calibration.accel_scale[axis];
        gyro_dps[axis] = (float)imu_state.gyro_raw[axis] / BMI270_GYRO_LSB_PER_DPS;
    }
    result = ImuBmi270GyroCalAccumulator_Feed(&imu_gyro_cal_accumulator,
                                              now_ms,
                                              accel_g,
                                              gyro_dps,
                                              BMI270_GYRO_CAL_MAX_ABS_DPS,
                                              BMI270_GYRO_CAL_STILL_SPAN_DPS);
    ImuBmi270_UpdateGyroCalDiag(IMU_BMI270_GYRO_CAL_FAIL_NONE,
                                0xFFU,
                                imu_gyro_cal_accumulator.sum_dps,
                                imu_gyro_cal_accumulator.min_dps,
                                imu_gyro_cal_accumulator.max_dps,
                                imu_gyro_cal_accumulator.sample_count);
    if (result == IMU_BMI270_GYRO_CAL_ACC_COLLECTING)
    {
        imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_RUNNING;
        return;
    }
    if (result == IMU_BMI270_GYRO_CAL_ACC_FAIL_ABS)
    {
        ImuBmi270_FinishGyroCalibrationFailure(now_ms,
                                               IMU_BMI270_GYRO_CAL_FAIL_ABS,
                                               imu_gyro_cal_accumulator.fail_axis,
                                               1U);
        return;
    }
    if (result == IMU_BMI270_GYRO_CAL_ACC_FAIL_SPAN)
    {
        ImuBmi270_FinishGyroCalibrationFailure(now_ms,
                                               IMU_BMI270_GYRO_CAL_FAIL_SPAN,
                                               imu_gyro_cal_accumulator.fail_axis,
                                               1U);
        return;
    }
    if (result == IMU_BMI270_GYRO_CAL_ACC_READY
        && ImuBmi270GyroCalAccumulator_GetResult(&imu_gyro_cal_accumulator, bias_dps, accel_mean_g) != 0U)
    {
        imu_bmi270_quaternion_t calibrated_q;
        float                   body_accel_g[3];
        uint8_t                 calibrated_q_valid;
        uint32_t                primask;

        ImuBmi270Coordinate_Apply(imu_calibration.sensor_to_body, accel_mean_g, body_accel_g);
        calibrated_q_valid = ImuBmi270Quaternion_FromAccel(body_accel_g, &calibrated_q);
        primask            = __get_PRIMASK();
        __disable_irq();
        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            imu_state.gyro_bias_dps[axis]       = bias_dps[axis];
            imu_calibration.gyro_bias_dps[axis] = bias_dps[axis];
            imu_state.gyro_corrected_dps[axis]  = 0.0f;
            imu_state.gyro_filtered_dps[axis]   = 0.0f;
            imu_state.gyro_dps[axis]            = 0.0f;
        }
        imu_state.gyro_calibrated           = 1U;
        imu_state.filter_initialized        = 0U;
        imu_state.gyro_auto_cal_last_result = 1U;
        imu_state.gyro_auto_cal_state       = IMU_BMI270_GYRO_AUTO_CAL_DONE;
        if (calibrated_q_valid != 0U)
        {
            imu_fusion.q            = calibrated_q;
            imu_fusion.integral[0]  = 0.0f;
            imu_fusion.integral[1]  = 0.0f;
            imu_fusion.integral[2]  = 0.0f;
            imu_fusion.accel_weight = 1.0f;
            imu_fusion.status_flags = 0UL;
            imu_fusion.initialized  = 1U;
        }
        else
        {
            ImuBmi270Mahony_Init(&imu_fusion);
        }
        __set_PRIMASK(primask);
        imu_gyro_calibration_active = 0U;
        ImuBmi270GyroCalAccumulator_Init(&imu_gyro_cal_accumulator);
        ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
    }
}

void ImuBmi270_ClearCalibration(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        imu_state.gyro_bias_dps[i]       = 0.0f;
        imu_calibration.gyro_bias_dps[i] = 0.0f;
        imu_state.gyro_corrected_dps[i]  = 0.0f;
        imu_state.gyro_filtered_dps[i]   = 0.0f;
        imu_state.gyro_dps[i]            = 0.0f;
    }
    imu_state.gyro_calibrated           = 0U;
    imu_state.filter_initialized        = 0U;
    imu_state.gyro_auto_cal_attempts    = 0U;
    imu_state.gyro_auto_cal_last_result = 0U;
    imu_gyro_calibration_active         = 0U;
    imu_gyro_calibration_is_auto        = 0U;
    ImuBmi270GyroCalAccumulator_Init(&imu_gyro_cal_accumulator);
    __set_PRIMASK(primask);
    ImuBmi270_ScheduleAutoCal(HAL_GetTick(), BMI270_GYRO_AUTO_CAL_START_DELAY_MS);
}

void ImuBmi270_ApplyGyroBias(const float bias_dps[3])
{
    uint32_t primask;

    if (bias_dps == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        imu_state.gyro_bias_dps[i]       = bias_dps[i];
        imu_calibration.gyro_bias_dps[i] = bias_dps[i];
        imu_state.gyro_corrected_dps[i]  = 0.0f;
        imu_state.gyro_filtered_dps[i]   = 0.0f;
        imu_state.gyro_dps[i]            = 0.0f;
    }
    imu_state.gyro_calibrated           = 0U;
    imu_state.filter_initialized        = 0U;
    imu_state.gyro_auto_cal_last_result = 0U;
    imu_gyro_calibration_active         = 0U;
    imu_gyro_calibration_is_auto        = 0U;
    ImuBmi270GyroCalAccumulator_Init(&imu_gyro_cal_accumulator);
    if (imu_state.gyro_auto_cal_enabled != 0U)
    {
        imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_WAIT;
    }
    __set_PRIMASK(primask);
    ImuBmi270_ScheduleAutoCal(HAL_GetTick(), BMI270_GYRO_AUTO_CAL_START_DELAY_MS);
}

uint8_t ImuBmi270_ApplyCalibration(const imu_bmi270_calibration_t *calibration)
{
    uint32_t primask;

    if (ImuBmi270Calibration_Validate(calibration) == 0U)
    {
        return 0U;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    imu_calibration = *calibration;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        imu_state.gyro_bias_dps[i] = imu_calibration.gyro_bias_dps[i];
    }
    imu_state.filter_initialized = 0U;
    __set_PRIMASK(primask);
    return 1U;
}

void ImuBmi270_GetCalibration(imu_bmi270_calibration_t *calibration)
{
    uint32_t primask;

    if (calibration == 0)
    {
        return;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *calibration = imu_calibration;
    __set_PRIMASK(primask);
}

void ImuBmi270_GetState(imu_bmi270_state_t *state)
{
    uint32_t primask;

    if (state == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *state = imu_state;
    __set_PRIMASK(primask);
}
