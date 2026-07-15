#ifndef IMU_BMI270_H
#define IMU_BMI270_H

#include <stdint.h>

#include "bmi270_bus.h"
#include "imu_calibration.h"
#include "imu_bmi270_profile.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        IMU_BMI270_ERROR_NONE           = 0,
        IMU_BMI270_ERROR_CHIP_ID        = 1,
        IMU_BMI270_ERROR_SPI            = 2,
        IMU_BMI270_ERROR_CONFIG         = 3,
        IMU_BMI270_ERROR_READ           = 4,
        IMU_BMI270_ERROR_PROFILE_VERIFY = 5,
        IMU_BMI270_ERROR_FIFO           = 6,
        IMU_BMI270_ERROR_TIMESTAMP      = 7,
        IMU_BMI270_ERROR_INVALID_FRAME  = 8
    } imu_bmi270_error_t;

#define IMU_BMI270_QUALITY_SPI_ERROR           (1UL << 0)
#define IMU_BMI270_QUALITY_INIT_FAILED         (1UL << 1)
#define IMU_BMI270_QUALITY_FIFO_OVERFLOW       (1UL << 2)
#define IMU_BMI270_QUALITY_TIMESTAMP_ERROR     (1UL << 3)
#define IMU_BMI270_QUALITY_GYRO_SATURATION     (1UL << 4)
#define IMU_BMI270_QUALITY_ACCEL_ANOMALY       (1UL << 5)
#define IMU_BMI270_QUALITY_ATTITUDE_INVALID    (1UL << 6)
#define IMU_BMI270_QUALITY_POLL_FALLBACK       (1UL << 7)
#define IMU_BMI270_QUALITY_PROFILE_MISMATCH    (1UL << 8)
#define IMU_BMI270_QUALITY_TEMPERATURE_INVALID (1UL << 9)

    typedef enum
    {
        IMU_BMI270_INIT_STATE_RESET          = 0,
        IMU_BMI270_INIT_STATE_PROBE          = 1,
        IMU_BMI270_INIT_STATE_LOAD_CONFIG    = 2,
        IMU_BMI270_INIT_STATE_VERIFY_PROFILE = 3,
        IMU_BMI270_INIT_STATE_SAMPLING       = 4,
        IMU_BMI270_INIT_STATE_RETRY_WAIT     = 5,
        IMU_BMI270_INIT_STATE_DISABLED       = 6
    } imu_bmi270_init_state_t;

    typedef enum
    {
        IMU_BMI270_GYRO_CAL_FAIL_NONE   = 0,
        IMU_BMI270_GYRO_CAL_FAIL_CONFIG = 1,
        IMU_BMI270_GYRO_CAL_FAIL_READ   = 2,
        IMU_BMI270_GYRO_CAL_FAIL_ABS    = 3,
        IMU_BMI270_GYRO_CAL_FAIL_SPAN   = 4,
        IMU_BMI270_GYRO_CAL_FAIL_MOTION = 5
    } imu_bmi270_gyro_cal_fail_t;

    typedef enum
    {
        IMU_BMI270_GYRO_AUTO_CAL_DISABLED   = 0,
        IMU_BMI270_GYRO_AUTO_CAL_WAIT       = 1,
        IMU_BMI270_GYRO_AUTO_CAL_RUNNING    = 2,
        IMU_BMI270_GYRO_AUTO_CAL_DONE       = 3,
        IMU_BMI270_GYRO_AUTO_CAL_RETRY_WAIT = 4,
        IMU_BMI270_GYRO_AUTO_CAL_FAILED     = 5
    } imu_bmi270_gyro_auto_cal_state_t;

    typedef struct
    {
        uint8_t  enabled;
        uint8_t  online;
        uint8_t  chip_id;
        uint8_t  last_error;
        uint8_t  init_state;
        uint8_t  profile;
        uint32_t error_count;
        uint32_t last_update_ms;
        uint32_t sensor_time;
        uint8_t  sensor_time_valid;
        uint32_t sample_count;
        uint32_t drdy_count;
        uint32_t poll_fallback_count;
        int16_t  accel_raw[3];
        int16_t  gyro_raw[3];
        float    accel_g[3];
        float    body_accel_g[3];
        float    ros_accel_g[3];
        float    gyro_dps[3];
        float    gyro_bias_dps[3];
        float    gyro_corrected_dps[3];
        float    gyro_filtered_dps[3];
        float    body_gyro_dps[3];
        float    ros_gyro_dps[3];
        float    quaternion[4];
        float    roll_deg;
        float    pitch_deg;
        float    yaw_deg;
        float    temperature_c;
        uint8_t  temperature_valid;
        float    accel_correction_weight;
        uint32_t quality_flags;
        uint32_t quality_latched_flags;
        uint32_t spi_error_count;
        uint32_t init_failure_count;
        uint32_t fifo_overflow_count;
        uint32_t timestamp_error_count;
        uint32_t gyro_saturation_count;
        uint32_t accel_anomaly_count;
        uint32_t attitude_invalid_count;
        uint8_t  gyro_calibrated;
        uint8_t  filter_initialized;
        uint8_t  gyro_cal_fail_reason;
        uint8_t  gyro_cal_fail_axis;
        uint16_t gyro_cal_sample_count;
        uint8_t  gyro_auto_cal_enabled;
        uint8_t  gyro_auto_cal_state;
        uint8_t  gyro_auto_cal_attempts;
        uint8_t  gyro_auto_cal_last_result;
        float    gyro_cal_mean_dps[3];
        float    gyro_cal_min_dps[3];
        float    gyro_cal_max_dps[3];
        float    gyro_cal_span_dps[3];
    } imu_bmi270_state_t;

    void    ImuBmi270_Init(void);
    uint8_t ImuBmi270_SetEnabled(uint8_t enabled);
    uint8_t ImuBmi270_SetProfile(imu_bmi270_profile_id_t profile);
    uint8_t ImuBmi270_ProbeNow(void);
    uint8_t ImuBmi270_ConfigNow(void);
    uint8_t ImuBmi270_Update(void);
    void    ImuBmi270_OnDataReadyFromIsr(void);
    uint8_t ImuBmi270_Diagnose(imu_bmi270_diag_t *diag);
    uint8_t ImuBmi270_RawFrameHasSignal(const int16_t accel_raw[3], const int16_t gyro_raw[3]);
    uint8_t
    ImuBmi270_GyroCalSpanWithinLimit(const float min_dps[3], const float max_dps[3], float max_span_dps, uint8_t *axis);
    uint8_t ImuBmi270_AutoCalDue(uint8_t  enabled,
                                 uint8_t  online,
                                 uint8_t  calibrated,
                                 uint8_t  attempts,
                                 uint8_t  max_attempts,
                                 uint32_t now_ms,
                                 uint32_t next_ms);
    uint8_t ImuBmi270_CalibrateGyro(uint16_t samples, uint16_t delay_ms);
    void    ImuBmi270_ServiceCalibration(uint32_t now_ms, uint8_t stationary);
    void    ImuBmi270_ClearCalibration(void);
    void    ImuBmi270_ApplyGyroBias(const float bias_dps[3]);
    uint8_t ImuBmi270_ApplyCalibration(const imu_bmi270_calibration_t *calibration);
    void    ImuBmi270_GetCalibration(imu_bmi270_calibration_t *calibration);
    void    ImuBmi270_GetState(imu_bmi270_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
