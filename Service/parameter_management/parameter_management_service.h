#ifndef PARAMETER_MANAGEMENT_SERVICE_H
#define PARAMETER_MANAGEMENT_SERVICE_H

#include <stdint.h>

#include "parameter_management_config.h"
#include "parameter_management_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PARAMETER_MANAGEMENT_VERSION       PARAM_MODEL_VERSION
#define PARAMETER_MANAGEMENT_LINE_CHANNELS PARAM_MODEL_LINE_CHANNELS

    /** Initialize the unique RAM parameter owner from injected factory defaults. */
    uint8_t ParameterManagement_Init(const parameter_management_config_t *config);
    /** Copy the injected factory defaults without changing runtime state. */
    void ParameterManagement_Defaults(param_model_t *params);
    /** Reset RAM parameters to the injected factory defaults. */
    void ParameterManagement_ResetToDefaults(void);
    /** Read one complete RAM parameter snapshot and generation. */
    uint32_t ParameterManagement_GetSnapshot(param_model_t *params);
    /** Read owner status in one consistent snapshot. */
    uint32_t ParameterManagement_GetStatus(parameter_management_status_t *status);
    /** Validate and publish one complete RAM override. */
    uint8_t ParameterManagement_Set(const param_model_t *params);
    /** Replace the calibration portion of the schema-4 persistence bundle in RAM. */
    void ParameterManagement_SetImuCalibration(const imu_bmi270_calibration_t *calibration);
    /** Read the calibration loaded from or staged for schema-4 persistence. */
    uint8_t ParameterManagement_GetImuCalibration(imu_bmi270_calibration_t *calibration);
    /** Update whether current-zero fields are included in the next explicit save. */
    void ParameterManagement_SetCurrentZeroPersistence(uint8_t enabled);
    /** Load and publish the complete schema-4 bundle from Flash. */
    uint8_t ParameterManagement_Load(void);
    /** Explicitly persist the complete current schema-4 bundle. */
    uint8_t ParameterManagement_Save(void);
    /** Persist factory defaults first, then publish them to RAM on success. */
    uint8_t ParameterManagement_ResetAndSaveDefaults(void);
    /** Validate a parameter model without mutating owner state. */
    uint8_t ParameterManagement_Validate(const param_model_t *params);
    uint8_t ParameterManagement_GetFloat(const param_model_t *params, const char *name, float *value);
    uint8_t ParameterManagement_SetFloat(param_model_t *params, const char *name, float value);
    uint8_t ParameterManagement_GetInt(const param_model_t *params, const char *name, int32_t *value);
    uint8_t ParameterManagement_SetInt(param_model_t *params, const char *name, int32_t value);

#ifdef __cplusplus
}
#endif

#endif /* PARAMETER_MANAGEMENT_SERVICE_H */
