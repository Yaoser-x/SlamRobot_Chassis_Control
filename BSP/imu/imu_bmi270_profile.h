#ifndef IMU_BMI270_PROFILE_H
#define IMU_BMI270_PROFILE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        IMU_BMI270_PROFILE_NORMAL      = 0,
        IMU_BMI270_PROFILE_PERFORMANCE = 1,
        IMU_BMI270_PROFILE_DEBUG       = 2
    } imu_bmi270_profile_id_t;

    typedef struct
    {
        uint8_t  acc_conf;
        uint8_t  acc_range;
        uint8_t  gyr_conf;
        uint8_t  gyr_range;
        uint8_t  pwr_conf;
        uint8_t  pwr_ctrl;
        uint8_t  fifo_downs;
        uint8_t  fifo_wtm_0;
        uint8_t  fifo_wtm_1;
        uint8_t  fifo_config_0;
        uint8_t  fifo_config_1;
        uint8_t  int1_io_ctrl;
        uint8_t  int_map_data;
        uint16_t odr_hz;
        uint16_t fifo_watermark_bytes;
    } imu_bmi270_profile_t;

    typedef struct
    {
        uint8_t acc_conf;
        uint8_t acc_range;
        uint8_t gyr_conf;
        uint8_t gyr_range;
        uint8_t pwr_conf;
        uint8_t pwr_ctrl;
        uint8_t fifo_config_0;
        uint8_t fifo_config_1;
        uint8_t int1_io_ctrl;
        uint8_t int_map_data;
    } imu_bmi270_profile_check_t;

    const imu_bmi270_profile_t *ImuBmi270Profile_Get(imu_bmi270_profile_id_t profile);
    uint8_t ImuBmi270Profile_Check(const imu_bmi270_profile_t *profile, const imu_bmi270_profile_check_t *check);

#ifdef __cplusplus
}
#endif

#endif
