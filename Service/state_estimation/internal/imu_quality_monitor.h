#ifndef IMU_QUALITY_MONITOR_H
#define IMU_QUALITY_MONITOR_H

#include "bmi270_types.h"
#include "state_estimation_status.h"

typedef struct
{
    uint32_t previous_spi_error_count;
    uint32_t previous_init_failure_count;
    uint32_t previous_fifo_error_count;
    uint32_t previous_poll_fallback_count;
    uint8_t  previous_last_error;
} imu_device_event_baseline_t;

/** Start one quality cycle and map device-owned facts into the Service status. */
void ImuQualityMonitor_BeginCycle(const bmi270_driver_state_t   *device,
                                  imu_device_event_baseline_t   *baseline,
                                  state_estimation_imu_status_t *status);
/** Synchronize device event counters without replaying events after a runtime reset. */
void ImuQualityMonitor_ResetBaseline(const bmi270_driver_state_t *device, imu_device_event_baseline_t *baseline);
/** Record quality events produced by one newly consumed sample. */
void ImuQualityMonitor_RecordSample(uint32_t flags, state_estimation_imu_status_t *status);
/** Latch all transient flags observed in the completed cycle. */
void ImuQualityMonitor_EndCycle(state_estimation_imu_status_t *status);

#endif /* IMU_QUALITY_MONITOR_H */
