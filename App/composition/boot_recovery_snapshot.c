#include "boot_recovery_snapshot.h"
#include "platform_critical.h"

static app_boot_recovery_snapshot_t boot_snapshot;

void AppBootRecoverySnapshot_Capture(const parameter_management_status_t *parameters,
                                     const imu_calibration_t             *applied,
                                     uint8_t                              calibration_applied,
                                     uint8_t                              estimator_read_ok)
{
    platform_critical_state_t critical;
    if (parameters == 0 || applied == 0)
        return;
    critical = PlatformCritical_Enter();
    if (boot_snapshot.captured == 0U)
    {
        boot_snapshot.parameters          = parameters->params;
        boot_snapshot.loaded_calibration  = parameters->imu_calibration;
        boot_snapshot.applied_calibration = *applied;
        boot_snapshot.sequence            = parameters->loaded_sequence;
        boot_snapshot.persisted_crc32     = parameters->persisted_parameter_crc32;
        boot_snapshot.effective_crc32     = parameters->effective_parameter_crc32;
        boot_snapshot.flash_loaded        = parameters->flash_loaded;
        boot_snapshot.sequence_valid      = parameters->loaded_sequence_valid;
        boot_snapshot.calibration_applied = calibration_applied;
        boot_snapshot.estimator_read_ok   = estimator_read_ok;
        boot_snapshot.captured            = 1U;
    }
    PlatformCritical_Exit(critical);
}

void AppBootRecoverySnapshot_Get(app_boot_recovery_snapshot_t *snapshot)
{
    if (snapshot != 0)
    {
        platform_critical_state_t critical = PlatformCritical_Enter();
        *snapshot                          = boot_snapshot;
        PlatformCritical_Exit(critical);
    }
}
