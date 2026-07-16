#ifndef APP_PUBLISH_MODEL_H
#define APP_PUBLISH_MODEL_H

#include <stdint.h>

#include "communication_publish_model_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint32_t host_timeout_ms;
        uint32_t esp12f_timeout_ms;
        uint32_t line_timeout_ms;
    } app_publish_model_config_t;

    /** Initialize the App-owned double-buffered communication and display model. */
    uint8_t AppPublishModel_Init(const app_publish_model_config_t *config);
    /** Assemble one consistent product read model and publish it atomically. */
    void AppPublishModel_Update(uint32_t now_ms);
    /** Copy the latest complete read model and return its generation. */
    uint32_t AppPublishModel_Get(communication_publish_model_t *out);

#ifdef __cplusplus
}
#endif

#endif /* APP_PUBLISH_MODEL_H */
