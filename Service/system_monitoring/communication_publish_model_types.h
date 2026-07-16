#ifndef SYSTEM_PUBLISH_SNAPSHOT_TYPES_H
#define SYSTEM_PUBLISH_SNAPSHOT_TYPES_H

#include <stdint.h>

#define COMMUNICATION_PUBLISH_MOTOR_COUNT 4U
#define COMMUNICATION_PUBLISH_TASK_COUNT  9U

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

    typedef enum
    {
        SYSTEM_IMU_CAL_DISABLED = 0,
        SYSTEM_IMU_CAL_WAIT,
        SYSTEM_IMU_CAL_RUNNING,
        SYSTEM_IMU_CAL_DONE,
        SYSTEM_IMU_CAL_RETRY_WAIT,
        SYSTEM_IMU_CAL_FAILED
    } system_imu_calibration_state_t;

    typedef enum
    {
        SYSTEM_IMU_CAL_FAIL_NONE = 0,
        SYSTEM_IMU_CAL_FAIL_CONFIG,
        SYSTEM_IMU_CAL_FAIL_READ,
        SYSTEM_IMU_CAL_FAIL_ABS,
        SYSTEM_IMU_CAL_FAIL_SPAN,
        SYSTEM_IMU_CAL_FAIL_MOTION
    } system_imu_calibration_fail_t;

    typedef struct
    {
        uint8_t imu_online;
        uint8_t encoder_online;
        uint8_t motor_online;
        uint8_t adc_online;
        uint8_t upper_online;
        uint8_t esp12f_online;
        uint8_t line_online;
        uint8_t ps2_online;
    } communication_publish_module_health_t;

    typedef struct
    {
        float   motor_target_mps[COMMUNICATION_PUBLISH_MOTOR_COUNT];
        float   motor_actual_mps[COMMUNICATION_PUBLISH_MOTOR_COUNT];
        int16_t motor_output_permille[COMMUNICATION_PUBLISH_MOTOR_COUNT];
        uint8_t motor_enabled_mask;
    } communication_publish_chassis_t;

    typedef struct
    {
        int32_t count[COMMUNICATION_PUBLISH_MOTOR_COUNT];
        float   speed_mps[COMMUNICATION_PUBLISH_MOTOR_COUNT];
        uint8_t speed_valid[COMMUNICATION_PUBLISH_MOTOR_COUNT];
        uint8_t speed_valid_all;
        uint8_t anomaly_mask;
    } communication_publish_encoder_t;

    typedef struct
    {
        float    battery_voltage;
        float    motor_current_a[COMMUNICATION_PUBLISH_MOTOR_COUNT];
        uint32_t error_flags;
        uint32_t latched_error_flags;
        uint16_t task_timeout_mask;
        uint8_t  control_mode;
        uint8_t  current_control_valid;
        uint8_t  tim_break_active;
        uint8_t  motor_fault_mask;
    } communication_publish_safety_t;

    typedef struct
    {
        uint8_t  online;
        uint8_t  chip_id;
        uint8_t  calibrated;
        uint8_t  sensor_time_valid;
        uint8_t  last_error;
        float    accel_g[3];
        float    gyro_dps[3];
        float    quaternion[4];
        float    roll_deg;
        float    pitch_deg;
        float    yaw_deg;
        float    temperature_c;
        uint32_t timestamp_ms;
        uint32_t sensor_time;
        uint32_t sample_count;
        uint32_t quality_flags;
        uint32_t quality_counters[7];
        uint8_t  calibration_state;
        uint8_t  calibration_fail_reason;
        uint16_t calibration_sample_count;
    } communication_publish_imu_t;

    typedef struct
    {
        uint32_t invalid_reason_flags;
        uint32_t valid_flags;
        uint8_t  current_valid;
    } communication_publish_current_t;

    typedef struct
    {
        uint8_t  emergency_stop;
        uint8_t  fault_stop;
        uint8_t  line_enabled;
        uint8_t  active_source;
        uint32_t reset_reason_flags;
    } communication_publish_control_t;

    typedef struct
    {
        uint32_t rx_frames;
        uint32_t checksum_errors;
        uint32_t length_errors;
        uint32_t timeout_resets;
        uint32_t rx_overflows;
        uint32_t tx_drops;
        uint32_t last_valid_frame_ms;
        uint8_t  download_mode;
    } communication_publish_link_health_t;

    typedef struct
    {
        communication_publish_link_health_t upper;
        communication_publish_link_health_t esp12f;
        uint32_t                            upper_last_rx_timestamp_ms;
    } communication_publish_health_t;

    typedef struct
    {
        uint32_t last_heartbeat_ms[COMMUNICATION_PUBLISH_TASK_COUNT];
        uint32_t timeout_count[COMMUNICATION_PUBLISH_TASK_COUNT];
        uint32_t missed_count[COMMUNICATION_PUBLISH_TASK_COUNT];
        uint8_t  timed_out[COMMUNICATION_PUBLISH_TASK_COUNT];
    } communication_publish_task_health_t;

    typedef struct
    {
        uint8_t  done;
        uint8_t  drv_fault_mask;
        uint8_t  adc_current_valid;
        uint8_t  imu_chip_id;
        uint8_t  encoder_speed_valid_all;
        uint8_t  drv_status;
        uint8_t  adc_status;
        uint8_t  imu_status;
        uint8_t  encoder_status;
        uint32_t error_flags;
    } communication_publish_post_t;

    typedef struct
    {
        uint32_t                              generation;
        uint32_t                              timestamp_ms;
        communication_publish_chassis_t       chassis;
        communication_publish_safety_t        safety;
        communication_publish_encoder_t       encoder;
        communication_publish_imu_t           imu;
        communication_publish_current_t       current;
        communication_publish_control_t       control;
        communication_publish_task_health_t   task_health;
        communication_publish_post_t          post;
        communication_publish_health_t        communication;
        communication_publish_module_health_t modules;
    } communication_publish_model_t;

#ifdef __cplusplus
}
#endif /* SYSTEM_PUBLISH_SNAPSHOT_TYPES_H */

#endif /* COMMUNICATION_PUBLISH_MODEL_H */
