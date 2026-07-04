#ifndef IMU_BMI270_TIME_H
#define IMU_BMI270_TIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_BMI270_SENSOR_TIME_MASK 0x00FFFFFFUL
#define IMU_BMI270_SENSOR_TIME_TICKS_PER_SEC 25600.0f
#define IMU_BMI270_SENSOR_TIME_100HZ_TICKS 256UL

uint32_t ImuBmi270Time_DeltaTicks24(uint32_t now, uint32_t previous);
uint8_t ImuBmi270Time_DeltaSeconds(uint32_t now, uint32_t previous, float max_dt_s, float *dt_s);

#ifdef __cplusplus
}
#endif

#endif
