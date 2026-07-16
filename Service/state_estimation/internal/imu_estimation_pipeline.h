#ifndef IMU_ESTIMATION_PIPELINE_H
#define IMU_ESTIMATION_PIPELINE_H

#include "bmi270_types.h"
#include "imu_calibration.h"
#include "parameter_imu_calibration_types.h"
#include "state_estimation_status.h"

/** @brief Reset all service-owned IMU filters, attitude, and calibration state. */
void ImuEstimationPipeline_Init(void);
/** @brief Explicitly map device facts and estimate calibrated robot attitude. */
void ImuEstimationPipeline_Process(const bmi270_sample_t        *sample,
                                   const bmi270_driver_state_t  *device,
                                   state_estimation_imu_status_t *status);
uint8_t ImuEstimationPipeline_ApplyCalibration(const imu_calibration_t *calibration);
void    ImuEstimationPipeline_ClearCalibration(void);
void    ImuEstimationPipeline_GetCalibration(imu_calibration_t *calibration);
uint8_t ImuEstimationPipeline_BeginCalibration(uint16_t samples, uint16_t interval_ms, uint8_t automatic);
void ImuEstimationPipeline_ServiceCalibration(uint32_t now_ms,
                                              uint8_t stationary,
                                              state_estimation_imu_status_t *status);

#endif /* IMU_ESTIMATION_PIPELINE_H */
