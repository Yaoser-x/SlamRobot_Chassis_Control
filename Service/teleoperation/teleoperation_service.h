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

    uint8_t Teleoperation_ValidateConfig(const teleoperation_config_t *config);
    uint8_t Teleoperation_Init(const teleoperation_config_t *config);
    /** @brief Execute one Teleoperation cycle from App-provided facts and emit at most one action. */
    void Teleoperation_Update(uint8_t line_tracking_enabled, teleoperation_action_t *action);
    /** Adopt the App-issued generation after a validated PS2 takeover transition. */
    void     Teleoperation_OnManualModeEntered(uint32_t motion_revoke_generation);
    uint32_t Teleoperation_GetStatus(teleoperation_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* TELEOPERATION_SERVICE_H */
