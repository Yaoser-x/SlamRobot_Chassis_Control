#ifndef DEBUG_CMD_CONTROL_H
#define DEBUG_CMD_CONTROL_H

#include <stdint.h>

#include "control_service.h"

typedef struct
{
    uint8_t       *velocity_enabled;
    uint32_t      *velocity_generation;
    chassis_cmd_t *velocity_command;
    uint8_t (*motor_test_allowed)(void);
    void (*revoke_maintenance)(void);
} debug_cmd_control_context_t;

uint8_t DebugCmdControl_TryHandle(const char *line, const debug_cmd_control_context_t *context);

#endif
