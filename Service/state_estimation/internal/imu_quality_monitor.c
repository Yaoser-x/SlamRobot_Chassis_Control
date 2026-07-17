#include "imu_quality_monitor.h"

void ImuQualityMonitor_ResetBaseline(const bmi270_driver_state_t *device, imu_device_event_baseline_t *baseline)
{
    if (device == 0 || baseline == 0)
    {
        return;
    }
    baseline->previous_spi_error_count     = device->spi_error_count;
    baseline->previous_init_failure_count  = device->init_failure_count;
    baseline->previous_fifo_error_count    = device->fifo_error_count;
    baseline->previous_poll_fallback_count = device->poll_fallback_count;
    baseline->previous_last_error          = device->last_error;
}

void ImuQualityMonitor_BeginCycle(const bmi270_driver_state_t   *device,
                                  imu_device_event_baseline_t   *baseline,
                                  state_estimation_imu_status_t *status)
{
    if (device == 0 || baseline == 0 || status == 0)
    {
        return;
    }
    status->quality_flags           = 0UL;
    status->enabled                 = device->enabled;
    status->online                  = device->online;
    status->chip_id                 = device->chip_id;
    status->last_error              = device->last_error;
    status->init_state              = device->init_state;
    status->profile                 = device->profile;
    status->error_count             = device->error_count;
    status->last_update_ms          = device->last_update_ms;
    status->sample_count            = device->sample_count;
    status->drdy_count              = device->drdy_count;
    status->poll_fallback_count     = device->poll_fallback_count;
    status->spi_error_count         = device->spi_error_count;
    status->init_failure_count      = device->init_failure_count;
    status->fifo_overflow_count     = device->fifo_error_count;
    status->temperature_c           = device->temperature_c;
    status->temperature_valid       = device->temperature_valid;
    status->temperature_error_count = device->temperature_error_count;

    if (device->spi_error_count != baseline->previous_spi_error_count)
    {
        status->quality_flags |= STATE_ESTIMATION_IMU_QUALITY_SPI_ERROR;
    }
    if (device->init_failure_count != baseline->previous_init_failure_count)
    {
        status->quality_flags |= STATE_ESTIMATION_IMU_QUALITY_INIT_FAILED;
    }
    if (device->fifo_error_count != baseline->previous_fifo_error_count)
    {
        status->quality_flags |= STATE_ESTIMATION_IMU_QUALITY_FIFO_OVERFLOW;
    }
    if (device->last_error == STATE_ESTIMATION_IMU_ERROR_PROFILE_VERIFY
        && baseline->previous_last_error != STATE_ESTIMATION_IMU_ERROR_PROFILE_VERIFY)
    {
        status->quality_flags |= STATE_ESTIMATION_IMU_QUALITY_PROFILE_MISMATCH;
    }
    if (device->poll_fallback_count != baseline->previous_poll_fallback_count)
    {
        status->quality_flags |= STATE_ESTIMATION_IMU_QUALITY_POLL_FALLBACK;
    }
    if (device->enabled != 0U && device->online != 0U && device->temperature_sampled != 0U
        && device->temperature_valid == 0U)
    {
        status->quality_flags |= STATE_ESTIMATION_IMU_QUALITY_TEMPERATURE_INVALID;
    }
    ImuQualityMonitor_ResetBaseline(device, baseline);
}

void ImuQualityMonitor_RecordSample(uint32_t flags, state_estimation_imu_status_t *status)
{
    if (status == 0)
    {
        return;
    }
    status->quality_flags |= flags;
    if ((flags & STATE_ESTIMATION_IMU_QUALITY_TIMESTAMP_ERROR) != 0UL)
    {
        status->timestamp_error_count++;
    }
    if ((flags & STATE_ESTIMATION_IMU_QUALITY_GYRO_SATURATION) != 0UL)
    {
        status->gyro_saturation_count++;
    }
    if ((flags & STATE_ESTIMATION_IMU_QUALITY_ACCEL_ANOMALY) != 0UL)
    {
        status->accel_anomaly_count++;
    }
    if ((flags & STATE_ESTIMATION_IMU_QUALITY_ATTITUDE_INVALID) != 0UL)
    {
        status->attitude_invalid_count++;
    }
}

void ImuQualityMonitor_EndCycle(state_estimation_imu_status_t *status)
{
    if (status != 0)
    {
        status->quality_latched_flags |= status->quality_flags;
    }
}
