#ifndef LINE_FOLLOWING_STATUS_H
#define LINE_FOLLOWING_STATUS_H

#include <stdint.h>

typedef enum
{
    LINE_FOLLOWING_RESULT_NONE = 0,
    LINE_FOLLOWING_RESULT_APPLIED,
    LINE_FOLLOWING_RESULT_REJECTED
} line_following_result_t;

typedef struct
{
    float                   line_position;
    float                   error;
    float                   error_derivative;
    uint8_t                 detected_count;
    uint8_t                 sensor_state[8];
    uint16_t                sensor_raw[8];
    uint16_t                threshold_raw[8];
    float                   linear_x;
    float                   angular_z;
    uint8_t                 tracking_active;
    uint8_t                 globally_enabled;
    uint8_t                 active_low;
    uint8_t                 output_saturated;
    uint8_t                 lost_reason;
    uint8_t                 sensor_valid;
    line_following_result_t last_enable_result;
    uint32_t                sensor_timestamp_ms;
    uint32_t                generation;
} line_following_status_t;

#endif /* LINE_FOLLOWING_STATUS_H */
