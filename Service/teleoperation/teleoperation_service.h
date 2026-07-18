#ifndef TELEOPERATION_SERVICE_H
#define TELEOPERATION_SERVICE_H

#include <stdint.h>

#include "teleoperation_action_types.h"
#include "teleoperation_config.h"
#include "teleoperation_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

    uint8_t  Teleoperation_ValidateConfig(const teleoperation_config_t *config);
    uint8_t  Teleoperation_Init(const teleoperation_config_t *config);
    /** @brief Execute one Teleoperation cycle. Writes at most one action to *action. Speed commands are submitted
     * internally. */
    void     Teleoperation_Update(teleoperation_action_t *action);
    uint32_t Teleoperation_GetStatus(teleoperation_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* TELEOPERATION_SERVICE_H */
