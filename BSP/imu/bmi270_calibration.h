#ifndef BMI270_DRIVER_CALIBRATION_H
#define BMI270_DRIVER_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif /* BMI270_DRIVER_CALIBRATION_H */

#define IMU_BMI270_CALIBRATION_VERSION 1UL

    typedef struct
    {
        uint32_t version;
        float    accel_bias_g[3];
        float    accel_scale[3];
        float    gyro_bias_dps[3];
        float    temperature_offset_c;
        float    temperature_gyro_slope_dps_per_c[3];
        float    sensor_to_body[3][3];
        uint32_t crc;
    } bmi270_calibration_t;

    void     Bmi270Calibration_Default(bmi270_calibration_t *calibration);
    uint32_t Bmi270Calibration_Crc(const bmi270_calibration_t *calibration);
    uint8_t  Bmi270Calibration_Validate(const bmi270_calibration_t *calibration);
    /** Return the calibrated gyro bias at the measured temperature. */
    float Bmi270Calibration_GyroBiasAtTemperature(const bmi270_calibration_t *calibration,
                                                  uint8_t                     axis,
                                                  float                       temperature_c,
                                                  uint8_t                     temperature_valid);

#ifdef __cplusplus
}
#endif

#endif
