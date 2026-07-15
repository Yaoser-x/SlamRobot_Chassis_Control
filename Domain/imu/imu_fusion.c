#include "imu_fusion.h"

#include <math.h>
#include <string.h>

#define IMU_BMI270_DEG_TO_RAD 0.017453292519943295f
#define IMU_BMI270_RAD_TO_DEG 57.29577951308232f

static float ImuBmi270Math_InvSqrt(float value)
{
    if (value <= 0.0f)
    {
        return 0.0f;
    }
    return 1.0f / sqrtf(value);
}

uint8_t ImuBmi270_TemperatureRawToC(int16_t raw, float *temperature_c)
{
    if (temperature_c == 0 || raw == INT16_MIN)
    {
        return 0U;
    }
    *temperature_c = 23.0f + ((float)raw / 512.0f);
    return 1U;
}

uint8_t ImuBmi270_RawFrameHasSignal(const int16_t accel_raw[3], const int16_t gyro_raw[3])
{
    if (accel_raw == 0 || gyro_raw == 0)
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if (accel_raw[i] != 0 || gyro_raw[i] != 0)
        {
            return 1U;
        }
    }
    return 0U;
}

uint8_t
ImuBmi270_GyroCalSpanWithinLimit(const float min_dps[3], const float max_dps[3], float max_span_dps, uint8_t *axis)
{
    if (min_dps == 0 || max_dps == 0)
    {
        return 0U;
    }
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        if ((max_dps[i] - min_dps[i]) > max_span_dps)
        {
            if (axis != 0)
            {
                *axis = i;
            }
            return 0U;
        }
    }
    return 1U;
}

uint8_t ImuBmi270_AutoCalDue(uint8_t  enabled,
                             uint8_t  online,
                             uint8_t  calibrated,
                             uint8_t  attempts,
                             uint8_t  max_attempts,
                             uint32_t now_ms,
                             uint32_t next_ms)
{
    if (enabled == 0U || online == 0U || calibrated != 0U)
    {
        return 0U;
    }
    if (attempts >= max_attempts)
    {
        return 0U;
    }
    return (((int32_t)(now_ms - next_ms)) >= 0) ? 1U : 0U;
}

void ImuBmi270Coordinate_Apply(float matrix[3][3], const float in[3], float out[3])
{
    float tmp[3];

    if (matrix == 0 || in == 0 || out == 0)
    {
        return;
    }

    for (uint8_t row = 0U; row < 3U; ++row)
    {
        tmp[row] = (matrix[row][0] * in[0]) + (matrix[row][1] * in[1]) + (matrix[row][2] * in[2]);
    }
    out[0] = tmp[0];
    out[1] = tmp[1];
    out[2] = tmp[2];
}

void ImuBmi270Coordinate_BodyToRos(const float body[3], float ros[3])
{
    if (body == 0 || ros == 0)
    {
        return;
    }

    ros[0] = body[0];
    ros[1] = body[1];
    ros[2] = body[2];
}

imu_bmi270_mahony_params_t ImuBmi270Mahony_DefaultParams(void)
{
    imu_bmi270_mahony_params_t params;

    params.kp                      = 1.0f;
    params.ki                      = 0.0f;
    params.accel_norm_min_g        = 0.75f;
    params.accel_norm_max_g        = 1.25f;
    params.accel_norm_reject_min_g = 0.40f;
    params.accel_norm_reject_max_g = 1.80f;
    return params;
}

void ImuBmi270Mahony_Init(imu_bmi270_mahony_t *fusion)
{
    if (fusion == 0)
    {
        return;
    }

    memset(fusion, 0, sizeof(*fusion));
    fusion->q.w          = 1.0f;
    fusion->accel_weight = 1.0f;
    fusion->initialized  = 1U;
}

float ImuBmi270Quaternion_Norm(const imu_bmi270_quaternion_t *q)
{
    if (q == 0)
    {
        return 0.0f;
    }
    return sqrtf((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));
}

void ImuBmi270Quaternion_Normalize(imu_bmi270_quaternion_t *q)
{
    float inv_norm;

    if (q == 0)
    {
        return;
    }

    inv_norm = ImuBmi270Math_InvSqrt((q->w * q->w) + (q->x * q->x) + (q->y * q->y) + (q->z * q->z));
    if (inv_norm <= 0.0f)
    {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
        return;
    }

    q->w *= inv_norm;
    q->x *= inv_norm;
    q->y *= inv_norm;
    q->z *= inv_norm;
}

uint8_t ImuBmi270Quaternion_FromAccel(const float accel_g[3], imu_bmi270_quaternion_t *q)
{
    float ax;
    float ay;
    float az;
    float norm;
    float roll;
    float pitch;
    float cr;
    float sr;
    float cp;
    float sp;

    if (accel_g == 0 || q == 0)
    {
        return 0U;
    }

    ax   = accel_g[0];
    ay   = accel_g[1];
    az   = accel_g[2];
    norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
    if (norm <= 0.000001f)
    {
        return 0U;
    }

    ax /= norm;
    ay /= norm;
    az /= norm;
    roll  = atan2f(ay, az);
    pitch = atan2f(-ax, sqrtf((ay * ay) + (az * az)));

    cr = cosf(roll * 0.5f);
    sr = sinf(roll * 0.5f);
    cp = cosf(pitch * 0.5f);
    sp = sinf(pitch * 0.5f);

    q->w = cr * cp;
    q->x = sr * cp;
    q->y = cr * sp;
    q->z = -sr * sp;
    ImuBmi270Quaternion_Normalize(q);
    return 1U;
}

void ImuBmi270Mahony_Update(imu_bmi270_mahony_t              *fusion,
                            const float                       gyro_dps[3],
                            const float                       accel_g[3],
                            float                             dt_s,
                            const imu_bmi270_mahony_params_t *params)
{
    imu_bmi270_mahony_params_t local_params;
    float                      gx;
    float                      gy;
    float                      gz;
    float                      ax;
    float                      ay;
    float                      az;
    float                      norm;
    float                      half_vx;
    float                      half_vy;
    float                      half_vz;
    float                      half_ex;
    float                      half_ey;
    float                      half_ez;
    float                      accel_weight = 0.0f;
    imu_bmi270_quaternion_t   *q;

    if (fusion == 0 || gyro_dps == 0 || accel_g == 0)
    {
        return;
    }
    if (fusion->initialized == 0U)
    {
        ImuBmi270Mahony_Init(fusion);
    }
    if (dt_s <= 0.0f || dt_s > 0.25f)
    {
        fusion->status_flags |= IMU_BMI270_FUSION_INVALID_DT;
        return;
    }

    local_params = (params != 0) ? *params : ImuBmi270Mahony_DefaultParams();
    fusion->status_flags &= ~(IMU_BMI270_FUSION_ACCEL_DEGRADED | IMU_BMI270_FUSION_INVALID_DT);
    q = &fusion->q;

    gx = gyro_dps[0] * IMU_BMI270_DEG_TO_RAD;
    gy = gyro_dps[1] * IMU_BMI270_DEG_TO_RAD;
    gz = gyro_dps[2] * IMU_BMI270_DEG_TO_RAD;

    ax   = accel_g[0];
    ay   = accel_g[1];
    az   = accel_g[2];
    norm = sqrtf((ax * ax) + (ay * ay) + (az * az));
    if (norm >= local_params.accel_norm_min_g && norm <= local_params.accel_norm_max_g)
    {
        accel_weight = 1.0f;
    }
    else if (norm >= local_params.accel_norm_reject_min_g && norm <= local_params.accel_norm_reject_max_g)
    {
        accel_weight = 0.1f;
        fusion->status_flags |= IMU_BMI270_FUSION_ACCEL_DEGRADED;
    }
    else
    {
        accel_weight = 0.0f;
        fusion->status_flags |= IMU_BMI270_FUSION_ACCEL_DEGRADED;
    }
    fusion->accel_weight = accel_weight;

    if (accel_weight > 0.0f)
    {
        norm = ImuBmi270Math_InvSqrt((ax * ax) + (ay * ay) + (az * az));
        ax *= norm;
        ay *= norm;
        az *= norm;

        half_vx = q->x * q->z - q->w * q->y;
        half_vy = q->w * q->x + q->y * q->z;
        half_vz = q->w * q->w - 0.5f + q->z * q->z;

        half_ex = (ay * half_vz - az * half_vy) * accel_weight;
        half_ey = (az * half_vx - ax * half_vz) * accel_weight;
        half_ez = (ax * half_vy - ay * half_vx) * accel_weight;

        if (local_params.ki > 0.0f)
        {
            fusion->integral[0] += local_params.ki * half_ex * dt_s;
            fusion->integral[1] += local_params.ki * half_ey * dt_s;
            fusion->integral[2] += local_params.ki * half_ez * dt_s;
            gx += fusion->integral[0];
            gy += fusion->integral[1];
            gz += fusion->integral[2];
        }
        else
        {
            fusion->integral[0] = 0.0f;
            fusion->integral[1] = 0.0f;
            fusion->integral[2] = 0.0f;
        }

        gx += local_params.kp * half_ex;
        gy += local_params.kp * half_ey;
        gz += local_params.kp * half_ez;
    }

    gx *= 0.5f * dt_s;
    gy *= 0.5f * dt_s;
    gz *= 0.5f * dt_s;

    {
        float qa = q->w;
        float qb = q->x;
        float qc = q->y;

        q->w += (-qb * gx - qc * gy - q->z * gz);
        q->x += (qa * gx + qc * gz - q->z * gy);
        q->y += (qa * gy - qb * gz + q->z * gx);
        q->z += (qa * gz + qb * gy - qc * gx);
    }
    ImuBmi270Quaternion_Normalize(q);
}

void ImuBmi270Quaternion_ToEulerDeg(const imu_bmi270_quaternion_t *q, float euler_deg[3])
{
    float sinr_cosp;
    float cosr_cosp;
    float sinp;
    float siny_cosp;
    float cosy_cosp;

    if (q == 0 || euler_deg == 0)
    {
        return;
    }

    sinr_cosp    = 2.0f * ((q->w * q->x) + (q->y * q->z));
    cosr_cosp    = 1.0f - (2.0f * ((q->x * q->x) + (q->y * q->y)));
    euler_deg[0] = atan2f(sinr_cosp, cosr_cosp) * IMU_BMI270_RAD_TO_DEG;

    sinp = 2.0f * ((q->w * q->y) - (q->z * q->x));
    if (sinp >= 1.0f)
    {
        euler_deg[1] = 90.0f;
    }
    else if (sinp <= -1.0f)
    {
        euler_deg[1] = -90.0f;
    }
    else
    {
        euler_deg[1] = asinf(sinp) * IMU_BMI270_RAD_TO_DEG;
    }

    siny_cosp    = 2.0f * ((q->w * q->z) + (q->x * q->y));
    cosy_cosp    = 1.0f - (2.0f * ((q->y * q->y) + (q->z * q->z)));
    euler_deg[2] = atan2f(siny_cosp, cosy_cosp) * IMU_BMI270_RAD_TO_DEG;
}
