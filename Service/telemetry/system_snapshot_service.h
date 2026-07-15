#ifndef SYSTEM_SNAPSHOT_SERVICE_H
#define SYSTEM_SNAPSHOT_SERVICE_H

#include <stdint.h>

#include "motor_types.h"
#include "post_service.h"
#include "task_health_service.h"

#ifdef __cplusplus
extern "C"
{
#endif

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
    } module_health_snapshot_t;

    typedef struct
    {
        float   motor_target_mps[MOTOR_ID_COUNT];
        float   motor_actual_mps[MOTOR_ID_COUNT];
        int16_t motor_output_permille[MOTOR_ID_COUNT];
        uint8_t motor_enabled_mask;
    } system_chassis_snapshot_t;

    typedef struct
    {
        int32_t count[MOTOR_ID_COUNT];
        float   speed_mps[MOTOR_ID_COUNT];
        uint8_t speed_valid[MOTOR_ID_COUNT];
        uint8_t speed_valid_all;
        uint8_t anomaly_mask;
    } system_encoder_snapshot_t;

    typedef struct
    {
        float    battery_voltage;
        float    motor_current_a[MOTOR_ID_COUNT];
        uint32_t error_flags;
        uint32_t latched_error_flags;
        uint16_t task_timeout_mask;
        uint8_t  control_mode;
        uint8_t  current_control_valid;
        uint8_t  tim_break_active;
        uint8_t  motor_fault_mask;
    } system_safety_snapshot_t;

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
    } system_imu_snapshot_t;

    typedef struct
    {
        uint32_t invalid_reason_flags;
        uint32_t valid_flags;
        uint8_t  current_valid;
    } system_current_snapshot_t;

    typedef struct
    {
        uint8_t  emergency_stop;
        uint8_t  fault_stop;
        uint8_t  line_enabled;
        uint8_t  active_source;
        uint32_t reset_reason_flags;
    } system_control_snapshot_t;

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
    } communication_link_health_t;

    typedef struct
    {
        communication_link_health_t upper;
        communication_link_health_t esp12f;
        uint32_t                    upper_last_rx_timestamp_ms;
    } communication_health_snapshot_t;

    typedef struct
    {
        uint32_t                        generation;
        uint32_t                        timestamp_ms;
        system_chassis_snapshot_t       chassis;
        system_safety_snapshot_t        safety;
        system_encoder_snapshot_t       encoder;
        system_imu_snapshot_t           imu;
        system_current_snapshot_t       current;
        system_control_snapshot_t       control;
        chassis_task_health_t           task_health;
        post_result_t                   post;
        communication_health_snapshot_t communication;
        module_health_snapshot_t        modules;
    } system_snapshot_t;

    void     SystemSnapshotService_Init(void);
    void     SystemSnapshotService_Update(uint32_t now_ms);
    uint32_t SystemSnapshotService_Get(system_snapshot_t *out);

#ifdef __cplusplus
}
#endif

#endif
