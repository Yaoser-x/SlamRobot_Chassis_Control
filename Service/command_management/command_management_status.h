#ifndef COMMAND_MANAGEMENT_STATUS_H
#define COMMAND_MANAGEMENT_STATUS_H

#include <stdint.h>

#include "command_management_types.h"

typedef struct
{
    command_source_t active_source;
    uint8_t          motion_allowed;
    uint8_t          permitted_source_mask;
    uint8_t          rearm_required_mask;
    uint32_t         motion_revoke_generation;
    uint32_t         mode_generation;
    uint32_t         generation;
} command_management_status_t;

#endif /* COMMAND_MANAGEMENT_STATUS_H */
