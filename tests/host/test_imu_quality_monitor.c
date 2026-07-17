#include "imu_quality_monitor.h"

#include <stdio.h>
#include <stdlib.h>

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    bmi270_driver_state_t         device   = {0};
    imu_device_event_baseline_t   baseline = {0};
    state_estimation_imu_status_t status   = {0};
    const uint32_t                sample_flags =
        STATE_ESTIMATION_IMU_QUALITY_TIMESTAMP_ERROR | STATE_ESTIMATION_IMU_QUALITY_GYRO_SATURATION;

    device.spi_error_count     = 1U;
    device.poll_fallback_count = 1U;
    ImuQualityMonitor_BeginCycle(&device, &baseline, &status);
    ImuQualityMonitor_RecordSample(sample_flags, &status);
    ImuQualityMonitor_EndCycle(&status);
    require_int(status.spi_error_count == 1U && status.timestamp_error_count == 1U,
                "new device and sample events increment once");
    require_int((status.quality_latched_flags & sample_flags) == sample_flags, "sample faults are latched");

    device.last_error        = STATE_ESTIMATION_IMU_ERROR_NONE;
    device.temperature_valid = 1U;
    ImuQualityMonitor_BeginCycle(&device, &baseline, &status);
    ImuQualityMonitor_EndCycle(&status);
    require_int(status.quality_flags == 0U, "transient flags clear on a clean cycle");
    require_int(status.timestamp_error_count == 1U && status.spi_error_count == 1U,
                "no new sample does not repeat counters");
    require_int((status.quality_latched_flags & sample_flags) == sample_flags, "latched history survives clean cycles");

    ImuQualityMonitor_BeginCycle(&device, &baseline, &status);
    require_int((status.quality_flags & STATE_ESTIMATION_IMU_QUALITY_POLL_FALLBACK) == 0U,
                "synchronized reset baseline does not replay poll fallback");

    device.enabled = 1U;
    device.online  = 1U;
    device.init_failure_count++;
    device.temperature_sampled = 1U;
    device.temperature_valid   = 0U;
    ImuQualityMonitor_BeginCycle(&device, &baseline, &status);
    require_int((status.quality_flags & STATE_ESTIMATION_IMU_QUALITY_INIT_FAILED) != 0U,
                "configuration failure maps to init quality");
    require_int((status.quality_flags & STATE_ESTIMATION_IMU_QUALITY_TEMPERATURE_INVALID) != 0U,
                "invalid temperature maps to quality");
    device.last_error = STATE_ESTIMATION_IMU_ERROR_NONE;
    device.fifo_error_count++;
    ImuQualityMonitor_BeginCycle(&device, &baseline, &status);
    require_int((status.quality_flags & STATE_ESTIMATION_IMU_QUALITY_FIFO_OVERFLOW) != 0U,
                "FIFO failure maps to quality");
    device.last_error = STATE_ESTIMATION_IMU_ERROR_PROFILE_VERIFY;
    ImuQualityMonitor_BeginCycle(&device, &baseline, &status);
    require_int((status.quality_flags & STATE_ESTIMATION_IMU_QUALITY_PROFILE_MISMATCH) != 0U,
                "profile verification transition maps to quality");
    ImuQualityMonitor_EndCycle(&status);
    require_int((status.quality_latched_flags & STATE_ESTIMATION_IMU_QUALITY_PROFILE_MISMATCH) != 0U,
                "profile mismatch is latched after transition");
    /* Sustained PROFILE_VERIFY must NOT re-fire PROFILE_MISMATCH (transition detection). */
    ImuQualityMonitor_BeginCycle(&device, &baseline, &status);
    require_int((status.quality_flags & STATE_ESTIMATION_IMU_QUALITY_PROFILE_MISMATCH) == 0U,
                "sustained profile verify does not re-fire mismatch flag");
    /* Clear and re-assert PROFILE_VERIFY → new transition. */
    device.last_error = STATE_ESTIMATION_IMU_ERROR_NONE;
    ImuQualityMonitor_BeginCycle(&device, &baseline, &status);
    device.last_error = STATE_ESTIMATION_IMU_ERROR_PROFILE_VERIFY;
    ImuQualityMonitor_BeginCycle(&device, &baseline, &status);
    require_int((status.quality_flags & STATE_ESTIMATION_IMU_QUALITY_PROFILE_MISMATCH) != 0U,
                "re-asserted profile verify after clear fires new mismatch");
    ImuQualityMonitor_RecordSample(STATE_ESTIMATION_IMU_QUALITY_ACCEL_ANOMALY
                                       | STATE_ESTIMATION_IMU_QUALITY_ATTITUDE_INVALID,
                                   &status);
    require_int(status.accel_anomaly_count == 1U && status.attitude_invalid_count == 1U,
                "sample anomaly counters advance once");

    device = (bmi270_driver_state_t){
        .enabled             = 1U,
        .online              = 1U,
        .temperature_sampled = 0U,
    };
    baseline = (imu_device_event_baseline_t){0};
    status   = (state_estimation_imu_status_t){0};
    ImuQualityMonitor_BeginCycle(&device, &baseline, &status);
    ImuQualityMonitor_EndCycle(&status);
    require_int((status.quality_latched_flags & STATE_ESTIMATION_IMU_QUALITY_TEMPERATURE_INVALID) == 0U,
                "unsampled temperature is not latched as invalid");

    device.fifo_error_count = 1U;
    device.last_error       = STATE_ESTIMATION_IMU_ERROR_NONE;
    ImuQualityMonitor_BeginCycle(&device, &baseline, &status);
    ImuQualityMonitor_EndCycle(&status);
    require_int((status.quality_latched_flags & STATE_ESTIMATION_IMU_QUALITY_FIFO_OVERFLOW) != 0U,
                "FIFO event remains visible after successful fallback clears last error");
    (void)puts("imu quality monitor tests passed");
    return 0;
}
