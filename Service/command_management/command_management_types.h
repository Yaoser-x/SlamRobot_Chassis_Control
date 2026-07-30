#ifndef COMMAND_MANAGEMENT_TYPES_H
#define COMMAND_MANAGEMENT_TYPES_H

#include <stdint.h>

typedef enum
{
    COMMAND_SOURCE_NONE   = 0,
    COMMAND_SOURCE_HOST   = 1,
    COMMAND_SOURCE_PS2    = 2,
    COMMAND_SOURCE_ESP12F = 3,
    COMMAND_SOURCE_DEBUG  = 4,
    COMMAND_SOURCE_LINE   = 5,
    COMMAND_SOURCE_COUNT
} command_source_t;

typedef enum
{
    COMMAND_RESULT_REJECTED             = 0,
    COMMAND_RESULT_ACCEPTED             = 1,
    COMMAND_RESULT_REJECTED_AND_STOPPED = 2
} command_result_t;

typedef enum
{
    COMMAND_OUTCOME_ACTIVE_ACCEPTED = 0,
    COMMAND_OUTCOME_RELEASE_ACCEPTED,
    COMMAND_OUTCOME_GATE_CLOSED,
    COMMAND_OUTCOME_REARM_REQUIRED,
    COMMAND_OUTCOME_INVALID,
    COMMAND_OUTCOME_EXPIRED,
    COMMAND_OUTCOME_GENERATION_CONFLICT
} command_outcome_t;

typedef struct
{
    command_outcome_t outcome;
    uint8_t           source_cleared;
    uint32_t          slot_generation;
} command_apply_result_t;

typedef struct
{
    float            linear_x;
    float            angular_z;
    uint8_t          enable;
    command_source_t source;
    uint32_t         timestamp_ms;
} command_velocity_t;

#endif /* COMMAND_MANAGEMENT_TYPES_H */
