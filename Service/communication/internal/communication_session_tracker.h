#ifndef COMMUNICATION_SESSION_TRACKER_H
#define COMMUNICATION_SESSION_TRACKER_H

#include "communication_types.h"
#include "command_management_types.h"

#include <stdint.h>

typedef struct
{
    float    linear_x;
    float    angular_z;
    uint64_t session_id;
    uint32_t sequence;
    uint8_t  enable;
    uint8_t  mode;
} communication_wire_target_t;

typedef enum
{
    COMMUNICATION_SESSION_NEW_COMMAND = 0,
    COMMUNICATION_SESSION_DUPLICATE_KEEPALIVE,
    COMMUNICATION_SESSION_DISABLE,
    COMMUNICATION_SESSION_REJECT_STALE,
    COMMUNICATION_SESSION_REJECT_OUT_OF_ORDER,
    COMMUNICATION_SESSION_REJECT_MALFORMED
} communication_session_decision_t;

void                             CommunicationSessionTracker_Init(void);
communication_session_decision_t CommunicationSessionTracker_Evaluate(communication_link_t               link,
                                                                      const communication_wire_target_t *target,
                                                                      uint32_t                           now_ms);
void                             CommunicationSessionTracker_Complete(communication_link_t link,
                                                                      uint32_t             sequence,
                                                                      uint8_t              applied,
                                                                      uint8_t              reject_reason);
void    CommunicationSessionTracker_RecordReject(communication_link_t link, uint8_t reject_reason);
uint8_t CommunicationSessionTracker_SetRefreshToken(communication_link_t           link,
                                                    uint32_t                       sequence,
                                                    const command_refresh_token_t *token);
uint8_t CommunicationSessionTracker_GetRefreshToken(communication_link_t     link,
                                                    uint32_t                 sequence,
                                                    command_refresh_token_t *token);
void    CommunicationSessionTracker_GetSnapshot(communication_link_t link, communication_session_snapshot_t *snapshot);

#endif
