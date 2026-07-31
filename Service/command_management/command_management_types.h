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
    COMMAND_OUTCOME_GENERATION_CONFLICT,
    COMMAND_OUTCOME_SOURCE_NOT_ALLOWED
} command_outcome_t;

#define COMMAND_SOURCE_BIT(source) ((uint8_t)(1U << (source)))
#define COMMAND_SOURCE_MASK_HOST   COMMAND_SOURCE_BIT(COMMAND_SOURCE_HOST)
#define COMMAND_SOURCE_MASK_PS2    COMMAND_SOURCE_BIT(COMMAND_SOURCE_PS2)
#define COMMAND_SOURCE_MASK_ESP12F COMMAND_SOURCE_BIT(COMMAND_SOURCE_ESP12F)
#define COMMAND_SOURCE_MASK_DEBUG  COMMAND_SOURCE_BIT(COMMAND_SOURCE_DEBUG)
#define COMMAND_SOURCE_MASK_LINE   COMMAND_SOURCE_BIT(COMMAND_SOURCE_LINE)
#define COMMAND_SOURCE_MASK_REMOTE ((uint8_t)(COMMAND_SOURCE_MASK_HOST | COMMAND_SOURCE_MASK_ESP12F))
#define COMMAND_ALL_SOURCE_MASK                                                                                        \
    ((uint8_t)(COMMAND_SOURCE_MASK_HOST | COMMAND_SOURCE_MASK_PS2 | COMMAND_SOURCE_MASK_ESP12F                         \
               | COMMAND_SOURCE_MASK_DEBUG | COMMAND_SOURCE_MASK_LINE))

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

typedef enum
{
    COMMAND_INTENT_NONE = 0,
    COMMAND_INTENT_ACTIVE,
    COMMAND_INTENT_NEUTRAL,
    COMMAND_INTENT_RELEASE,
    COMMAND_INTENT_REARM,
    COMMAND_INTENT_REMOTE_DISABLE
} command_intent_kind_t;

/** Source-agnostic producer intent; wire session state remains in Communication. */
typedef struct
{
    command_intent_kind_t kind;
    command_source_t      source;
    float                 linear_x;
    float                 angular_z;
    uint32_t              sample_time_ms;
    uint32_t              producer_generation;
    uint32_t              expected_revoke_generation;
    uint8_t               mode;
} command_intent_t;

typedef struct
{
    command_source_t source;
    uint32_t         slot_generation;
    uint32_t         accepted_command_id;
    uint32_t         revoke_generation;
    uint32_t         mode_generation;
    float            linear_x;
    float            angular_z;
    uint8_t          mode;
} command_refresh_token_t;

#endif /* COMMAND_MANAGEMENT_TYPES_H */
