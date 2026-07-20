#ifndef COMMUNICATION_TYPES_H
#define COMMUNICATION_TYPES_H

#include <stdint.h>

#define COMMUNICATION_GIT_COMMIT_LENGTH 20U

typedef enum
{
    COMMUNICATION_LINK_UPPER = 0,
    COMMUNICATION_LINK_ESP12F
} communication_link_t;

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
