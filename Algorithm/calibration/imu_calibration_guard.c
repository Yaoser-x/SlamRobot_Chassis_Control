#include "imu_calibration_guard.h"

#include <math.h>
#include <string.h>

#define IMU_CALIBRATION_GATE_MAX_SPEED_MPS          0.02f
#define IMU_CALIBRATION_GATE_ACCEL_MIN_G            0.95f
#define IMU_CALIBRATION_GATE_ACCEL_MAX_G            1.05f
#define IMU_CALIBRATION_GATE_ACCEL_VARIANCE_MAX_G2  0.0004f
#define IMU_CALIBRATION_GATE_GYRO_VARIANCE_MAX_DPS2 0.25f

static void ImuCalibrationGuard_ResetEvidence(imu_calibration_guard_t *gate, uint32_t imu_sample_count)
{
    memset(gate, 0, sizeof(*gate));
    gate->last_sample_count = imu_sample_count;
    gate->has_last_sample   = 1U;
}

static uint8_t ImuCalibrationGuard_EvidenceStable(const imu_calibration_guard_t *gate)
{
    float sample_count;
    float mean;
    float variance;

    if (gate->sample_count < IMU_CALIBRATION_GATE_WINDOW_SAMPLES)
    {
        return 0U;
    }
    sample_count = (float)gate->sample_count;
    mean         = gate->accel_sum / sample_count;
    variance     = (gate->accel_sum_sq / sample_count) - (mean * mean);
    if (variance > IMU_CALIBRATION_GATE_ACCEL_VARIANCE_MAX_G2)
    {
        return 0U;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        mean     = gate->gyro_sum[axis] / sample_count;
        variance = (gate->gyro_sum_sq[axis] / sample_count) - (mean * mean);
        if (variance > IMU_CALIBRATION_GATE_GYRO_VARIANCE_MAX_DPS2)
        {
            return 0U;
        }
    }
    return 1U;
}

void ImuCalibrationGuard_Init(imu_calibration_guard_t *gate)
{
    if (gate != 0)
    {
        memset(gate, 0, sizeof(*gate));
    }
}

uint8_t ImuCalibrationGuard_Update(imu_calibration_guard_t *gate,
                                   const int16_t            effective_pwm[IMU_CALIBRATION_GATE_MOTOR_COUNT],
                                   const float              speed_mps[IMU_CALIBRATION_GATE_MOTOR_COUNT],
                                   const uint8_t            speed_valid[IMU_CALIBRATION_GATE_MOTOR_COUNT],
                                   uint8_t                  motor_enabled_mask,
                                   const float              accel_g[3],
                                   const float              gyro_dps[3],
                                   uint32_t                 imu_sample_count)
{
    float accel_norm;

    if (gate == 0 || effective_pwm == 0 || speed_mps == 0 || speed_valid == 0 || accel_g == 0 || gyro_dps == 0
        || motor_enabled_mask == 0U)
    {
        return 0U;
    }
    for (uint8_t motor = 0U; motor < IMU_CALIBRATION_GATE_MOTOR_COUNT; ++motor)
    {
        if (effective_pwm[motor] != 0)
        {
            ImuCalibrationGuard_ResetEvidence(gate, imu_sample_count);
            return 0U;
        }
        if ((motor_enabled_mask & (uint8_t)(1U << motor)) != 0U
            && (speed_valid[motor] == 0U || speed_mps[motor] < -IMU_CALIBRATION_GATE_MAX_SPEED_MPS
                || speed_mps[motor] > IMU_CALIBRATION_GATE_MAX_SPEED_MPS))
        {
            ImuCalibrationGuard_ResetEvidence(gate, imu_sample_count);
            return 0U;
        }
    }
    accel_norm = sqrtf((accel_g[0] * accel_g[0]) + (accel_g[1] * accel_g[1]) + (accel_g[2] * accel_g[2]));
    if (accel_norm < IMU_CALIBRATION_GATE_ACCEL_MIN_G || accel_norm > IMU_CALIBRATION_GATE_ACCEL_MAX_G)
    {
        ImuCalibrationGuard_ResetEvidence(gate, imu_sample_count);
        return 0U;
    }
    if (gate->has_last_sample != 0U && gate->last_sample_count == imu_sample_count)
    {
        return ImuCalibrationGuard_EvidenceStable(gate);
    }
    if (gate->sample_count >= IMU_CALIBRATION_GATE_WINDOW_SAMPLES)
    {
        float old_accel = gate->accel_norm_window[gate->write_index];

        gate->accel_sum -= old_accel;
        gate->accel_sum_sq -= old_accel * old_accel;
        for (uint8_t axis = 0U; axis < 3U; ++axis)
        {
            float old_gyro = gate->gyro_window[axis][gate->write_index];

            gate->gyro_sum[axis] -= old_gyro;
            gate->gyro_sum_sq[axis] -= old_gyro * old_gyro;
        }
    }
    else
    {
        gate->sample_count++;
    }
    gate->accel_norm_window[gate->write_index] = accel_norm;
    gate->accel_sum += accel_norm;
    gate->accel_sum_sq += accel_norm * accel_norm;
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        gate->gyro_window[axis][gate->write_index] = gyro_dps[axis];
        gate->gyro_sum[axis] += gyro_dps[axis];
        gate->gyro_sum_sq[axis] += gyro_dps[axis] * gyro_dps[axis];
    }
    gate->write_index++;
    if (gate->write_index >= IMU_CALIBRATION_GATE_WINDOW_SAMPLES)
    {
        gate->write_index = 0U;
    }
    gate->last_sample_count = imu_sample_count;
    gate->has_last_sample   = 1U;
    return ImuCalibrationGuard_EvidenceStable(gate);
}
