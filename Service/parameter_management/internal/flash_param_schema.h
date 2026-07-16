#ifndef FLASH_PARAM_SCHEMA_H
#define FLASH_PARAM_SCHEMA_H

#include "flash_param.h"
#include "parameter_management_service.h"

typedef struct
{
    uint32_t version;
    float    max_linear_mps;
    float    max_angular_rps;
    float    speed_ramp_mps2;
    float    angular_ramp_rps2;
    float    wheel_radius_m;
    float    track_width_m;
    float    pid_kp[PARAM_MODEL_MOTOR_COUNT];
    float    pid_ki[PARAM_MODEL_MOTOR_COUNT];
    float    pid_kd[PARAM_MODEL_MOTOR_COUNT];
    float    pid_integral_limit;
    int8_t   motor_dir[PARAM_MODEL_MOTOR_COUNT];
    int8_t   encoder_dir[PARAM_MODEL_MOTOR_COUNT];
    uint16_t current_zero_raw[PARAM_MODEL_MOTOR_COUNT];
    uint8_t  current_zero_valid;
    float    imu_gyro_bias_dps[3];
    uint8_t  imu_gyro_bias_valid;
} param_model_v1_t;

typedef struct
{
    param_model_v1_t         params;
    imu_bmi270_calibration_t imu_calibration;
} flash_param_bundle_v2_t;

typedef struct
{
    uint32_t version;
    float    max_linear_mps;
    float    max_angular_rps;
    float    speed_ramp_mps2;
    float    angular_ramp_rps2;
    float    wheel_radius_m;
    float    track_width_m;
    float    pid_kp[PARAM_MODEL_MOTOR_COUNT];
    float    pid_ki[PARAM_MODEL_MOTOR_COUNT];
    float    pid_kd[PARAM_MODEL_MOTOR_COUNT];
    float    pid_integral_limit;
    int8_t   motor_dir[PARAM_MODEL_MOTOR_COUNT];
    int8_t   encoder_dir[PARAM_MODEL_MOTOR_COUNT];
    uint16_t current_zero_raw[PARAM_MODEL_MOTOR_COUNT];
    uint8_t  current_zero_valid;
    float    imu_gyro_bias_dps[3];
    uint8_t  imu_gyro_bias_valid;
    uint16_t line_threshold_raw[PARAMETER_MANAGEMENT_LINE_CHANNELS];
    uint8_t  line_active_low;
    float    line_kp;
    float    line_kd;
    float    line_speed_mps;
    float    line_slowdown_gain;
    uint8_t  line_detect_debounce_frames;
    uint8_t  line_lost_debounce_frames;
    float    current_observe_a[PARAM_MODEL_MOTOR_COUNT];
    float    current_soft_limit_a[PARAM_MODEL_MOTOR_COUNT];
    float    current_fault_a[PARAM_MODEL_MOTOR_COUNT];
    uint16_t current_fault_debounce_ms;
    float    straight_wheel_coupling_gain;
    float    straight_heading_kp;
    uint8_t  straight_heading_hold_enabled;
} param_service_v2_t;

typedef struct
{
    param_service_v2_t       params;
    imu_bmi270_calibration_t imu_calibration;
} flash_param_bundle_v3_t;

typedef struct
{
    uint32_t                magic;
    uint32_t                schema_version;
    uint32_t                sequence;
    uint32_t                payload_size;
    flash_param_bundle_v2_t payload;
    uint32_t                crc32;
    uint32_t                commit_marker;
} flash_param_image_v2_t;

typedef struct
{
    uint32_t                magic;
    uint32_t                schema_version;
    uint32_t                sequence;
    uint32_t                payload_size;
    flash_param_bundle_v3_t payload;
    uint32_t                crc32;
    uint32_t                commit_marker;
} flash_param_image_v3_t;

typedef struct
{
    uint32_t         magic;
    uint32_t         version;
    uint32_t         data_size;
    uint32_t         crc32;
    param_model_v1_t data;
} flash_param_legacy_image_t;

typedef struct
{
    uint32_t             magic;
    uint32_t             schema_version;
    uint32_t             sequence;
    uint32_t             payload_size;
    flash_param_bundle_t payload;
    uint32_t             crc32;
    uint32_t             commit_marker;
} flash_param_image_t;

_Static_assert(sizeof(flash_param_image_t) == FLASH_PARAM_IMAGE_SIZE, "flash parameter image size mismatch");
_Static_assert((FLASH_PARAM_IMAGE_SIZE % 4U) == 0U, "flash parameter image must be word aligned");

#endif /* FLASH_PARAM_SCHEMA_H */
