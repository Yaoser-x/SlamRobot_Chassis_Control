#ifndef PARAM_SERVICE_H
#define PARAM_SERVICE_H

#include <stdint.h>

#include "parameter_management_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PARAM_SERVICE_VERSION       PARAM_MODEL_VERSION
#define PARAM_SERVICE_LINE_CHANNELS PARAM_MODEL_LINE_CHANNELS

    /** @deprecated Use ParameterManagement_*; this header is a stateless Beta5 migration wrapper. */
    void     ParamService_Defaults(param_model_t *params);
    void     ParamService_SetDefaults(void);
    void     ParamService_Get(param_model_t *params);
    uint32_t ParamService_GetSnapshot(param_model_t *params);
    uint8_t  ParamService_Set(const param_model_t *params);
    uint8_t  ParamService_Validate(const param_model_t *params);
    uint8_t  ParamService_GetFloat(const param_model_t *params, const char *name, float *value);
    uint8_t  ParamService_SetFloat(param_model_t *params, const char *name, float value);
    uint8_t  ParamService_GetInt(const param_model_t *params, const char *name, int32_t *value);
    uint8_t  ParamService_SetInt(param_model_t *params, const char *name, int32_t value);

#ifdef __cplusplus
}
#endif

#endif /* PARAM_SERVICE_H */
