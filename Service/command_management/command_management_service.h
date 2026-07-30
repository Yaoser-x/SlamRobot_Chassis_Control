#ifndef COMMAND_MANAGEMENT_SERVICE_H
#define COMMAND_MANAGEMENT_SERVICE_H

#include <stdint.h>

#include "command_management_config.h"
#include "command_management_status.h"
#include "command_management_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    uint8_t                CommandManagement_Init(const command_management_config_t *config);
    uint8_t                CommandManagement_IsInitialized(void);
    command_result_t       CommandManagement_Set(const command_velocity_t *command);
    command_result_t       CommandManagement_SetForGeneration(const command_velocity_t *command,
                                                              uint32_t                  expected_generation);
    command_apply_result_t CommandManagement_Apply(const command_velocity_t *command);
    command_apply_result_t CommandManagement_ApplyForGeneration(const command_velocity_t *command,
                                                                uint32_t                  expected_generation);
    void                   CommandManagement_ClearAll(void);
    void                   CommandManagement_ClearSource(command_source_t source);
    /** Clear and rearm Host/ESP only when Safety's motion gate is open. */
    command_result_t CommandManagement_DisableRemoteSource(command_source_t source);
    /** Qualify one source after its source-specific release condition has been observed. */
    command_apply_result_t CommandManagement_QualifyRearm(command_source_t source);
    /** Refresh one still-valid source lease without changing its target or generation. */
    uint8_t          CommandManagement_RefreshSource(command_source_t source, uint32_t now_ms);
    uint8_t          CommandManagement_GetActive(command_velocity_t *command, uint32_t now_ms);
    command_source_t CommandManagement_GetActiveSource(uint32_t now_ms);
    uint32_t         CommandManagement_GetMotionRevokeGeneration(void);
    uint32_t         CommandManagement_GetStatus(uint32_t now_ms, command_management_status_t *status);
    /** Apply Safety's motion decision; closing the gate revokes and clears every source. */
    void CommandManagement_SetMotionGate(uint8_t allowed, uint32_t decision_generation);
    /** Report the controlled gate state supplied by Safety Management. */
    uint8_t CommandManagement_IsMotionGateOpen(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMAND_MANAGEMENT_SERVICE_H */
