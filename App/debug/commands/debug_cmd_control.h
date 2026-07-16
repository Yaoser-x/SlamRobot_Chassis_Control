#ifndef DEBUG_CMD_CONTROL_H
#define DEBUG_CMD_CONTROL_H

#include <stdint.h>

#include "command_management_types.h"

typedef struct
{
    uint8_t            *velocity_enabled;
    uint32_t           *velocity_generation;
    command_velocity_t *velocity_command;
    uint8_t (*motor_test_allowed)(void);
    void (*revoke_maintenance)(void);
} debug_cmd_control_context_t;

uint8_t DebugCmdControl_TryHandle(const char *line, const debug_cmd_control_context_t *context);

#endif
