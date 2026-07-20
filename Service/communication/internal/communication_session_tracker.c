#include "communication_session_tracker.h"

#include "communication_protocol_types.h"
#include "platform_critical.h"

#include <string.h>

#define COMMUNICATION_SESSION_SLOT_COUNT    2U
#define COMMUNICATION_RETIRED_SESSION_COUNT 8U

typedef struct
{
    communication_wire_target_t last_target;
    uint64_t                    retired[COMMUNICATION_RETIRED_SESSION_COUNT];
    uint64_t                    active_session;
    uint32_t                    received_sequence;
    uint32_t                    applied_sequence;
    uint32_t                    generation;
    uint32_t                    last_valid_receive_ms;
    uint8_t                     retired_write_index;
    uint8_t                     active;
    uint8_t                     target_valid;
    uint8_t                     reject_reason;
    uint8_t                     ack_flags;
} communication_session_slot_t;

static communication_session_slot_t session_slots[COMMUNICATION_SESSION_SLOT_COUNT];
static uint8_t                      session_tracker_initialized;

static uint8_t CommunicationSessionTracker_LinkValid(communication_link_t link)
{
    return ((uint8_t)link < COMMUNICATION_SESSION_SLOT_COUNT) ? 1U : 0U;
}

static uint8_t CommunicationSessionTracker_IsRetired(const communication_session_slot_t *slot, uint64_t session_id)
{
    for (uint8_t i = 0U; i < COMMUNICATION_RETIRED_SESSION_COUNT; ++i)
    {
        if (slot->retired[i] == session_id)
        {
            return 1U;
        }
    }
    return 0U;
}

static uint8_t CommunicationSessionTracker_TargetEqual(const communication_wire_target_t *left,
                                                       const communication_wire_target_t *right)
{
    uint32_t left_linear;
    uint32_t right_linear;
    uint32_t left_angular;
    uint32_t right_angular;

    (void)memcpy(&left_linear, &left->linear_x, sizeof(left_linear));
    (void)memcpy(&right_linear, &right->linear_x, sizeof(right_linear));
    (void)memcpy(&left_angular, &left->angular_z, sizeof(left_angular));
    (void)memcpy(&right_angular, &right->angular_z, sizeof(right_angular));
    return (left_linear == right_linear && left_angular == right_angular && left->enable == right->enable
            && left->mode == right->mode)
               ? 1U
               : 0U;
}

static void CommunicationSessionTracker_Reject(communication_session_slot_t *slot, uint8_t reject_reason)
{
    slot->reject_reason = reject_reason;
    slot->ack_flags     = COMMUNICATION_ACK_REJECTED;
    if (slot->active != 0U)
    {
        slot->ack_flags |= COMMUNICATION_ACK_SESSION_VALID;
    }
    slot->generation++;
}

void CommunicationSessionTracker_Init(void)
{
    platform_critical_state_t critical = PlatformCritical_Enter();

    if (session_tracker_initialized == 0U)
    {
        (void)memset(session_slots, 0, sizeof(session_slots));
        session_tracker_initialized = 1U;
    }
    PlatformCritical_Exit(critical);
}

communication_session_decision_t CommunicationSessionTracker_Evaluate(communication_link_t               link,
                                                                      const communication_wire_target_t *target,
                                                                      uint32_t                           now_ms)
{
    communication_session_slot_t    *slot;
    communication_session_decision_t decision;
    platform_critical_state_t        critical;

    if (CommunicationSessionTracker_LinkValid(link) == 0U || target == 0 || target->session_id == 0ULL)
    {
        return COMMUNICATION_SESSION_REJECT_MALFORMED;
    }

    critical = PlatformCritical_Enter();
    slot     = &session_slots[(uint8_t)link];
    if (CommunicationSessionTracker_IsRetired(slot, target->session_id) != 0U)
    {
        CommunicationSessionTracker_Reject(slot, COMMUNICATION_REJECT_STALE_SESSION);
        decision = COMMUNICATION_SESSION_REJECT_STALE;
    }
    else if (slot->active == 0U || slot->active_session != target->session_id)
    {
        if (target->enable != 0U)
        {
            CommunicationSessionTracker_Reject(slot, COMMUNICATION_REJECT_STALE_SESSION);
            decision = COMMUNICATION_SESSION_REJECT_STALE;
        }
        else
        {
            if (slot->active != 0U)
            {
                slot->retired[slot->retired_write_index] = slot->active_session;
                slot->retired_write_index =
                    (uint8_t)((slot->retired_write_index + 1U) % COMMUNICATION_RETIRED_SESSION_COUNT);
            }
            slot->active                = 1U;
            slot->active_session        = target->session_id;
            slot->received_sequence     = target->sequence;
            slot->last_target           = *target;
            slot->target_valid          = 1U;
            slot->last_valid_receive_ms = now_ms;
            slot->reject_reason         = COMMUNICATION_REJECT_NONE;
            slot->ack_flags             = COMMUNICATION_ACK_SESSION_VALID | COMMUNICATION_ACK_RECEIVED;
            slot->generation++;
            decision = COMMUNICATION_SESSION_DISABLE;
        }
    }
    else
    {
        uint32_t delta = target->sequence - slot->received_sequence;

        if (delta == 0UL)
        {
            if (slot->target_valid != 0U && CommunicationSessionTracker_TargetEqual(&slot->last_target, target) != 0U)
            {
                slot->last_valid_receive_ms = now_ms;
                slot->reject_reason         = COMMUNICATION_REJECT_NONE;
                slot->ack_flags =
                    COMMUNICATION_ACK_SESSION_VALID | COMMUNICATION_ACK_RECEIVED | COMMUNICATION_ACK_DUPLICATE;
                slot->generation++;
                decision = COMMUNICATION_SESSION_DUPLICATE_KEEPALIVE;
            }
            else
            {
                CommunicationSessionTracker_Reject(slot, COMMUNICATION_REJECT_OUT_OF_ORDER);
                decision = COMMUNICATION_SESSION_REJECT_OUT_OF_ORDER;
            }
        }
        else if (delta < 0x80000000UL)
        {
            slot->received_sequence     = target->sequence;
            slot->last_target           = *target;
            slot->target_valid          = 1U;
            slot->last_valid_receive_ms = now_ms;
            slot->reject_reason         = COMMUNICATION_REJECT_NONE;
            slot->ack_flags             = COMMUNICATION_ACK_SESSION_VALID | COMMUNICATION_ACK_RECEIVED;
            slot->generation++;
            decision = (target->enable != 0U) ? COMMUNICATION_SESSION_NEW_COMMAND : COMMUNICATION_SESSION_DISABLE;
        }
        else
        {
            CommunicationSessionTracker_Reject(slot, COMMUNICATION_REJECT_OUT_OF_ORDER);
            decision = COMMUNICATION_SESSION_REJECT_OUT_OF_ORDER;
        }
    }
    PlatformCritical_Exit(critical);
    return decision;
}

void CommunicationSessionTracker_Complete(communication_link_t link,
                                          uint32_t             sequence,
                                          uint8_t              applied,
                                          uint8_t              reject_reason)
{
    communication_session_slot_t *slot;
    platform_critical_state_t     critical;

    if (CommunicationSessionTracker_LinkValid(link) == 0U)
    {
        return;
    }
    critical = PlatformCritical_Enter();
    slot     = &session_slots[(uint8_t)link];
    if (slot->active != 0U && slot->received_sequence == sequence)
    {
        if (applied != 0U)
        {
            slot->applied_sequence = sequence;
            slot->reject_reason    = COMMUNICATION_REJECT_NONE;
            slot->ack_flags |= COMMUNICATION_ACK_APPLIED;
            slot->ack_flags &= (uint8_t)~COMMUNICATION_ACK_REJECTED;
        }
        else
        {
            slot->reject_reason = reject_reason;
            slot->ack_flags |= COMMUNICATION_ACK_REJECTED;
            slot->ack_flags &= (uint8_t)~COMMUNICATION_ACK_APPLIED;
        }
        slot->generation++;
    }
    PlatformCritical_Exit(critical);
}

void CommunicationSessionTracker_RecordReject(communication_link_t link, uint8_t reject_reason)
{
    platform_critical_state_t critical;

    if (CommunicationSessionTracker_LinkValid(link) == 0U)
    {
        return;
    }
    critical = PlatformCritical_Enter();
    CommunicationSessionTracker_Reject(&session_slots[(uint8_t)link], reject_reason);
    PlatformCritical_Exit(critical);
}

void CommunicationSessionTracker_GetSnapshot(communication_link_t link, communication_session_snapshot_t *snapshot)
{
    platform_critical_state_t           critical;
    const communication_session_slot_t *slot;

    if (snapshot == 0)
    {
        return;
    }
    *snapshot = (communication_session_snapshot_t){0};
    if (CommunicationSessionTracker_LinkValid(link) == 0U)
    {
        return;
    }
    critical                        = PlatformCritical_Enter();
    slot                            = &session_slots[(uint8_t)link];
    snapshot->session_id            = slot->active_session;
    snapshot->received_sequence     = slot->received_sequence;
    snapshot->applied_sequence      = slot->applied_sequence;
    snapshot->generation            = slot->generation;
    snapshot->last_valid_receive_ms = slot->last_valid_receive_ms;
    snapshot->reject_reason         = slot->reject_reason;
    snapshot->ack_flags             = slot->ack_flags;
    PlatformCritical_Exit(critical);
}
