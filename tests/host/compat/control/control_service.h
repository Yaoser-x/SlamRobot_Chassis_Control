#ifndef CONTROL_SERVICE_H
#define CONTROL_SERVICE_H

#include <stdint.h>

#include "command_management_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef command_source_t   control_source_t;
    typedef command_result_t   control_command_result_t;
    typedef command_velocity_t chassis_cmd_t;

#define CONTROL_SOURCE_NONE                  COMMAND_SOURCE_NONE
#define CONTROL_SOURCE_UPPER                 COMMAND_SOURCE_HOST
#define CONTROL_SOURCE_PS2                   COMMAND_SOURCE_PS2
#define CONTROL_SOURCE_ESP12F                COMMAND_SOURCE_ESP12F
#define CONTROL_SOURCE_DEBUG                 COMMAND_SOURCE_DEBUG
#define CONTROL_SOURCE_LINE                  COMMAND_SOURCE_LINE

#define CONTROL_COMMAND_REJECTED             COMMAND_RESULT_REJECTED
#define CONTROL_COMMAND_ACCEPTED             COMMAND_RESULT_ACCEPTED
#define CONTROL_COMMAND_REJECTED_AND_STOPPED COMMAND_RESULT_REJECTED_AND_STOPPED

    void                     ControlService_Init(void);
    control_command_result_t ControlService_SetCommand(const chassis_cmd_t *cmd);
    control_command_result_t ControlService_SetCommandForGeneration(const chassis_cmd_t *cmd,
                                                                    uint32_t             expected_generation);
    void                     ControlService_SetEmergencyStop(uint8_t enabled);
    void                     ControlService_SetFaultStop(uint8_t enabled);
    uint8_t                  ControlService_BeginMaintenance(void);
    void                     ControlService_EndMaintenance(void);
    uint8_t                  ControlService_IsMaintenanceLocked(void);
    uint32_t                 ControlService_GetMotionRevokeGeneration(void);
    void                     ControlService_ClearCommand(void);
    void                     ControlService_ClearSource(uint8_t source);
    uint8_t                  ControlService_GetCommand(chassis_cmd_t *cmd, uint32_t now_ms);
    uint8_t                  ControlService_IsEmergencyStop(void);
    uint8_t                  ControlService_IsFaultStop(void);
    uint8_t                  ControlService_GetActiveSource(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_SERVICE_H */
