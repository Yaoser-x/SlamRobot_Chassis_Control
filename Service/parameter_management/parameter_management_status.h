#ifndef PARAMETER_MANAGEMENT_STATUS_H
#define PARAMETER_MANAGEMENT_STATUS_H

#include <stdint.h>

#include "parameter_management_types.h"
#include "parameter_imu_calibration_types.h"

#define PARAM_NORMALIZED_PID_KD            (1UL << 0)
#define PARAM_PERSISTED_EFFECTIVE_MISMATCH (1UL << 1)

/** @brief Consistent RAM parameter snapshot and its monotonic generation. */
typedef struct
{
    param_model_t     params;
    uint32_t          generation;
    uint32_t          persisted_parameter_crc32;
    uint32_t          effective_parameter_crc32;
    uint32_t          diagnostic_flags;
    uint8_t           initialized;
    uint8_t           flash_loaded;
    uint8_t           persisted_model_valid;
    imu_calibration_t imu_calibration;
} parameter_management_status_t;

#endif /* PARAMETER_MANAGEMENT_STATUS_H */
