#ifndef SYSTEM_PUBLISH_SNAPSHOT_PROVIDER_H
#define SYSTEM_PUBLISH_SNAPSHOT_PROVIDER_H

#include "system_publish_snapshot_service.h"

/** @brief Collect cross-Service facts for the System Monitoring snapshot owner. */
void AppSystemPublishSnapshot_Collect(uint32_t                                    now_ms,
                                      const communication_publish_model_config_t *config,
                                      communication_publish_model_t              *snapshot);

#endif /* SYSTEM_PUBLISH_SNAPSHOT_PROVIDER_H */
