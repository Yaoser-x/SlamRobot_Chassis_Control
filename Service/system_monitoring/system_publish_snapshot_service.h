#ifndef SYSTEM_PUBLISH_SNAPSHOT_SERVICE_H
#define SYSTEM_PUBLISH_SNAPSHOT_SERVICE_H

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
    } communication_publish_model_config_t;

    typedef void (*system_publish_snapshot_provider_t)(uint32_t                                    now_ms,
                                                       const communication_publish_model_config_t *config,
                                                       communication_publish_model_t              *snapshot);

    /** Initialize the System Monitoring-owned double-buffered publish snapshot. */
    uint8_t CommunicationPublishModel_Init(const communication_publish_model_config_t *config,
                                           system_publish_snapshot_provider_t          provider);
    /** Assemble one consistent product read model and publish it atomically. */
    void CommunicationPublishModel_Update(uint32_t now_ms);
    /** Copy the latest complete read model and return its generation. */
    uint32_t CommunicationPublishModel_Get(communication_publish_model_t *out);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_PUBLISH_SNAPSHOT_SERVICE_H */
