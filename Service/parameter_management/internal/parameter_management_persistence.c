#include "parameter_management_internal.h"

#include "parameter_persistence_backend.h"
#include "parameter_management_service.h"

uint8_t ParameterManagement_Load(void)
{
    flash_param_bundle_t bundle;

    if (ParamPersistence_Load(&bundle) != FLASH_PARAM_STATUS_OK)
    {
        return 0U;
    }
    ParameterManagementInternal_ApplyLoaded(&bundle);
    return 1U;
}

uint8_t ParameterManagement_Save(void)
{
    flash_param_bundle_t bundle;

    ParameterManagementInternal_BuildBundle(&bundle);
    return (ParamPersistence_Save(&bundle) == FLASH_PARAM_STATUS_OK) ? 1U : 0U;
}

uint8_t ParameterManagement_ResetAndSaveDefaults(void)
{
    flash_param_bundle_t bundle;

    ParameterManagement_Defaults(&bundle.params);
    ParameterImuCalibration_Default(&bundle.imu_calibration);
    if (ParamPersistence_Save(&bundle) != FLASH_PARAM_STATUS_OK)
    {
        return 0U;
    }
    ParameterManagement_ResetToDefaults();
    ParameterManagement_SetImuCalibration(&bundle.imu_calibration);
    return 1U;
}
