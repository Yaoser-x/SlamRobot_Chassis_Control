#ifndef COMMUNICATION_TYPES_H
#define COMMUNICATION_TYPES_H

#include <stdint.h>

#define COMMUNICATION_GIT_COMMIT_LENGTH 20U

typedef enum
{
    COMMUNICATION_LINK_UPPER = 0,
    COMMUNICATION_LINK_ESP12F,
    COMMUNICATION_LINK_COUNT
} communication_link_t;

typedef enum
{
    COMMUNICATION_OPERATION_ESTOP = 0,
    COMMUNICATION_OPERATION_CLEAR_FAULT,
    COMMUNICATION_OPERATION_LINE_CTRL
} communication_operation_kind_t;

typedef enum
{
    COMMUNICATION_OPERATION_FRAME_ACCEPTED = 0,
    COMMUNICATION_OPERATION_REQUEST_DISPATCHED,
    COMMUNICATION_OPERATION_BUSINESS_APPLIED,
    COMMUNICATION_OPERATION_BUSINESS_REJECTED,
    COMMUNICATION_OPERATION_CONDITION_NOT_CLEARED,
    COMMUNICATION_OPERATION_TIMEOUT
} communication_operation_stage_t;

typedef struct
{
    communication_operation_kind_t kind;
    communication_link_t           link;
    uint32_t                       generation;
    uint32_t                       received_at_ms;
    uint8_t                        value;
} communication_operation_request_t;

typedef struct
{
    communication_operation_kind_t  kind;
    communication_link_t            link;
    communication_operation_stage_t stage;
    uint32_t                        generation;
    uint32_t                        updated_at_ms;
    uint32_t                        detail_mask;
} communication_operation_result_t;

typedef struct
{
    uint8_t  git_commit[COMMUNICATION_GIT_COMMIT_LENGTH];
    uint32_t hardware_revision;
    uint32_t capabilities;
} communication_firmware_identity_t;

typedef struct
{
    uint64_t session_id;
    uint32_t received_sequence;
    uint32_t applied_sequence;
    uint32_t generation;
    uint32_t last_valid_receive_ms;
    uint8_t  reject_reason;
    uint8_t  ack_flags;
} communication_session_snapshot_t;

#endif
