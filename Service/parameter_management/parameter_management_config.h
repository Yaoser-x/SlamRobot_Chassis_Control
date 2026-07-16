#ifndef PARAMETER_MANAGEMENT_CONFIG_H
#define PARAMETER_MANAGEMENT_CONFIG_H

#include <stdint.h>

#include "parameter_management_types.h"

/** @brief Boot-time persistence policy owned by Parameter Management. */
typedef struct
{
    param_model_t factory_defaults;
    uint8_t       load_flash_on_boot;
    uint8_t       persist_imu_calibration;
    uint8_t       persist_current_zero;
} parameter_management_config_t;

#endif
