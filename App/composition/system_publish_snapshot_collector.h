#ifndef SYSTEM_PUBLISH_SNAPSHOT_COLLECTOR_H
#define SYSTEM_PUBLISH_SNAPSHOT_COLLECTOR_H

#include <stdint.h>

#include "communication_publish_model_types.h"

typedef struct
{
    uint32_t host_timeout_ms;
    uint32_t esp12f_timeout_ms;
    uint32_t line_timeout_ms;
} communication_publish_model_config_t;

/** @brief Collect cross-Service facts for the System Monitoring snapshot owner. */
void AppSystemPublishSnapshot_Collect(uint32_t                                    now_ms,
                                      const communication_publish_model_config_t *config,
                                      communication_publish_model_t              *snapshot);

#endif /* SYSTEM_PUBLISH_SNAPSHOT_COLLECTOR_H */
