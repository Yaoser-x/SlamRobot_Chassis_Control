#ifndef IMU_BMI270_MATH_H
#define IMU_BMI270_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IMU_BMI270_FUSION_ACCEL_DEGRADED (1UL << 0)
#define IMU_BMI270_FUSION_INVALID_DT     (1UL << 1)

typedef struct
{
  float w;
  float x;
  float y;
  float z;
} imu_bmi270_quaternion_t;

typedef struct
{
  float kp;
  float ki;
  float accel_norm_min_g;
  float accel_norm_max_g;
  float accel_norm_reject_min_g;
  float accel_norm_reject_max_g;
} imu_bmi270_mahony_params_t;

typedef struct
{
  imu_bmi270_quaternion_t q;
  float integral[3];
  float accel_weight;
  uint32_t status_flags;
  uint8_t initialized;
} imu_bmi270_mahony_t;

void ImuBmi270Coordinate_Apply(const float matrix[3][3], const float in[3], float out[3]);
void ImuBmi270Coordinate_BodyToRos(const float body[3], float ros[3]);

imu_bmi270_mahony_params_t ImuBmi270Mahony_DefaultParams(void);
void ImuBmi270Mahony_Init(imu_bmi270_mahony_t *fusion);
void ImuBmi270Mahony_Update(imu_bmi270_mahony_t *fusion,
                            const float gyro_dps[3],
                            const float accel_g[3],
                            float dt_s,
                            const imu_bmi270_mahony_params_t *params);
float ImuBmi270Quaternion_Norm(const imu_bmi270_quaternion_t *q);
void ImuBmi270Quaternion_Normalize(imu_bmi270_quaternion_t *q);
uint8_t ImuBmi270Quaternion_FromAccel(const float accel_g[3], imu_bmi270_quaternion_t *q);
void ImuBmi270Quaternion_ToEulerDeg(const imu_bmi270_quaternion_t *q, float euler_deg[3]);

#ifdef __cplusplus
}
#endif

#endif
