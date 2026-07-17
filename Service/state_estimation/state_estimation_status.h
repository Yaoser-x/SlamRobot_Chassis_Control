#ifndef STATE_ESTIMATION_STATUS_H
#define STATE_ESTIMATION_STATUS_H

#include <stdint.h>

#define STATE_ESTIMATION_MOTOR_COUNT                     4U

#define STATE_ESTIMATION_IMU_QUALITY_SPI_ERROR           (1UL << 0)
#define STATE_ESTIMATION_IMU_QUALITY_INIT_FAILED         (1UL << 1)
#define STATE_ESTIMATION_IMU_QUALITY_FIFO_OVERFLOW       (1UL << 2)
#define STATE_ESTIMATION_IMU_QUALITY_TIMESTAMP_ERROR     (1UL << 3)
#define STATE_ESTIMATION_IMU_QUALITY_GYRO_SATURATION     (1UL << 4)
#define STATE_ESTIMATION_IMU_QUALITY_ACCEL_ANOMALY       (1UL << 5)
#define STATE_ESTIMATION_IMU_QUALITY_ATTITUDE_INVALID    (1UL << 6)
#define STATE_ESTIMATION_IMU_QUALITY_POLL_FALLBACK       (1UL << 7)
#define STATE_ESTIMATION_IMU_QUALITY_PROFILE_MISMATCH    (1UL << 8)
#define STATE_ESTIMATION_IMU_QUALITY_TEMPERATURE_INVALID (1UL << 9)

typedef enum
{
    STATE_ESTIMATION_IMU_ERROR_NONE           = 0,
    STATE_ESTIMATION_IMU_ERROR_CHIP_ID        = 1,
    STATE_ESTIMATION_IMU_ERROR_SPI            = 2,
    STATE_ESTIMATION_IMU_ERROR_CONFIG         = 3,
    STATE_ESTIMATION_IMU_ERROR_READ           = 4,
    STATE_ESTIMATION_IMU_ERROR_PROFILE_VERIFY = 5,
    STATE_ESTIMATION_IMU_ERROR_FIFO           = 6,
    STATE_ESTIMATION_IMU_ERROR_TIMESTAMP      = 7,
    STATE_ESTIMATION_IMU_ERROR_INVALID_FRAME  = 8
} state_estimation_imu_error_t;

typedef enum
{
    STATE_ESTIMATION_IMU_INIT_STATE_RESET          = 0,
    STATE_ESTIMATION_IMU_INIT_STATE_PROBE          = 1,
    STATE_ESTIMATION_IMU_INIT_STATE_LOAD_CONFIG    = 2,
    STATE_ESTIMATION_IMU_INIT_STATE_VERIFY_PROFILE = 3,
    STATE_ESTIMATION_IMU_INIT_STATE_SAMPLING       = 4,
    STATE_ESTIMATION_IMU_INIT_STATE_RETRY_WAIT     = 5,
    STATE_ESTIMATION_IMU_INIT_STATE_DISABLED       = 6
} state_estimation_imu_init_state_t;

typedef enum
{
    STATE_ESTIMATION_IMU_PROFILE_NORMAL      = 0,
    STATE_ESTIMATION_IMU_PROFILE_PERFORMANCE = 1,
    STATE_ESTIMATION_IMU_PROFILE_DEBUG       = 2
} state_estimation_imu_profile_t;

typedef enum
{
    STATE_ESTIMATION_IMU_AUTO_CAL_DISABLED        = 0,
    STATE_ESTIMATION_IMU_AUTO_CAL_WAIT_ONLINE     = 1,
    STATE_ESTIMATION_IMU_AUTO_CAL_WAIT_STATIONARY = 2,
    STATE_ESTIMATION_IMU_AUTO_CAL_COLLECTING      = 3,
    STATE_ESTIMATION_IMU_AUTO_CAL_SUCCESS         = 4,
    STATE_ESTIMATION_IMU_AUTO_CAL_RETRY_WAIT      = 5,
    STATE_ESTIMATION_IMU_AUTO_CAL_FAILED          = 6
} state_estimation_imu_auto_cal_state_t;

typedef struct
{
    uint8_t hal_status[2];
    uint8_t hal_rx[2][3];
    uint8_t bitbang_rx[3];
    uint8_t miso_nopull;
    uint8_t miso_pullup;
    uint8_t miso_pulldown;
} state_estimation_imu_diagnostic_t;

typedef struct
{
    int32_t  count[STATE_ESTIMATION_MOTOR_COUNT];
    int32_t  delta[STATE_ESTIMATION_MOTOR_COUNT];
    float    speed_mps[STATE_ESTIMATION_MOTOR_COUNT];
    uint8_t  speed_valid[STATE_ESTIMATION_MOTOR_COUNT];
    uint8_t  reject_streak[STATE_ESTIMATION_MOTOR_COUNT];
    uint16_t window_rebuild_count[STATE_ESTIMATION_MOTOR_COUNT];
    uint16_t anomaly_count[STATE_ESTIMATION_MOTOR_COUNT];
    uint8_t  consecutive_anomalies[STATE_ESTIMATION_MOTOR_COUNT];
    int32_t  left_count;
    int32_t  right_count;
    int32_t  left_delta;
    int32_t  right_delta;
    float    left_speed_mps;
    float    right_speed_mps;
    uint8_t  left_speed_valid;
    uint8_t  right_speed_valid;
    uint8_t  speed_valid_all;
    uint32_t side_consistency_flags;
    uint32_t last_update_ms;
} state_estimation_wheel_status_t;

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
} state_estimation_imu_status_t;

/** @brief Wheel and IMU snapshots with independent freshness and generation. */
typedef struct
{
    state_estimation_wheel_status_t wheel;
    state_estimation_imu_status_t   imu;
    uint32_t                        wheel_generation;
    uint32_t                        imu_generation;
    uint8_t                         wheel_fresh;
    uint8_t                         imu_fresh;
} state_estimation_status_t;

#endif /* STATE_ESTIMATION_STATUS_H */
