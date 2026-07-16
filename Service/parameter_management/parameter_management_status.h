#ifndef PARAMETER_MANAGEMENT_STATUS_H
#define PARAMETER_MANAGEMENT_STATUS_H

#include <stdint.h>

#include "parameter_management_types.h"
#include "imu_calibration.h"

/** @brief Consistent RAM parameter snapshot and its monotonic generation. */
typedef struct
{
    param_model_t            params;
    uint32_t                 generation;
    uint8_t                  initialized;
    uint8_t                  flash_loaded;
    imu_bmi270_calibration_t imu_calibration;
} parameter_management_status_t;

#endif /* PARAMETER_MANAGEMENT_STATUS_H */
