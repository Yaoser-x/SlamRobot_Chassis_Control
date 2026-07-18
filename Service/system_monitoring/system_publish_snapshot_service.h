#ifndef SYSTEM_PUBLISH_SNAPSHOT_SERVICE_H
#define SYSTEM_PUBLISH_SNAPSHOT_SERVICE_H

#include <stdint.h>

#include "communication_publish_model_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** Initialize the System Monitoring-owned double-buffered publish snapshot. */
    void SystemPublishSnapshot_Init(void);
    /** Publish one App-assembled product read model atomically. */
    void SystemPublishSnapshot_Publish(const communication_publish_model_t *snapshot);
    /** Copy the latest complete read model and return its generation. */
    uint32_t SystemPublishSnapshot_Get(communication_publish_model_t *out);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_PUBLISH_SNAPSHOT_SERVICE_H */
