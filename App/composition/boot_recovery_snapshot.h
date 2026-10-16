#ifndef APP_BOOT_RECOVERY_SNAPSHOT_H
#define APP_BOOT_RECOVERY_SNAPSHOT_H
#include "parameter_management_status.h"

typedef struct
{
    param_model_t     parameters;
    imu_calibration_t loaded_calibration;
    imu_calibration_t applied_calibration;
    uint32_t          sequence;
    uint32_t          persisted_crc32;
    uint32_t          effective_crc32;
    uint8_t           captured;
    uint8_t           flash_loaded;
    uint8_t           sequence_valid;
    uint8_t           calibration_applied;
    uint8_t           estimator_read_ok;
} app_boot_recovery_snapshot_t;

/** Capture once before automatic calibration; later saves cannot replace boot evidence. */
void AppBootRecoverySnapshot_Capture(const parameter_management_status_t *parameters,
                                     const imu_calibration_t             *applied,
                                     uint8_t                              calibration_applied,
                                     uint8_t                              estimator_read_ok);
void AppBootRecoverySnapshot_Get(app_boot_recovery_snapshot_t *snapshot);
#endif
