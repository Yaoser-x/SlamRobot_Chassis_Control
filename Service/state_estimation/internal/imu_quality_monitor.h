#ifndef IMU_QUALITY_MONITOR_H
#define IMU_QUALITY_MONITOR_H

#include "bmi270_types.h"
#include "state_estimation_status.h"

/** Start one quality cycle and map device-owned facts into the Service status. */
void ImuQualityMonitor_BeginCycle(const bmi270_driver_state_t *device, state_estimation_imu_status_t *status);
/** Record quality events produced by one newly consumed sample. */
void ImuQualityMonitor_RecordSample(uint32_t flags, state_estimation_imu_status_t *status);
/** Latch all transient flags observed in the completed cycle. */
void ImuQualityMonitor_EndCycle(state_estimation_imu_status_t *status);

#endif /* IMU_QUALITY_MONITOR_H */
