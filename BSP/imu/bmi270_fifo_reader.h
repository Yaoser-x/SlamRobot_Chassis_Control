#ifndef BMI270_FIFO_READER_H
#define BMI270_FIFO_READER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define IMU_BMI270_FIFO_PARSE_SKIP_FRAME (1UL << 0)
#define IMU_BMI270_FIFO_PARSE_OVERREAD   (1UL << 1)
#define IMU_BMI270_FIFO_PARSE_TRUNCATED  (1UL << 2)
#define IMU_BMI270_FIFO_PARSE_INPUT_CFG  (1UL << 3)

    typedef struct
    {
        int16_t accel_raw[3];
        int16_t gyro_raw[3];
        uint8_t accel_valid;
        uint8_t gyro_valid;
    } imu_bmi270_fifo_sample_t;

    typedef struct
    {
        uint32_t sample_count;
        uint32_t sensor_time;
        uint8_t  sensor_time_valid;
        uint32_t skipped_frame_count;
        uint32_t flags;
    } imu_bmi270_fifo_parse_result_t;

    uint8_t ImuBmi270Fifo_Parse(const uint8_t                  *fifo,
                                uint16_t                        fifo_len,
                                imu_bmi270_fifo_sample_t       *samples,
                                uint16_t                        max_samples,
                                imu_bmi270_fifo_parse_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
