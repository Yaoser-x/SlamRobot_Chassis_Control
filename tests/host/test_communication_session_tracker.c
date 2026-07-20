#include "communication_session_tracker.h"

#include "communication_protocol_types.h"

#include <assert.h>
#include <stdio.h>

uint32_t PlatformCritical_Enter(void)
{
    return 0UL;
}

void PlatformCritical_Exit(uint32_t state)
{
    (void)state;
}

static communication_wire_target_t Target(uint64_t session, uint32_t sequence, uint8_t enable)
{
    return (communication_wire_target_t){
        .linear_x   = (enable != 0U) ? 0.1f : 0.0f,
        .angular_z  = (enable != 0U) ? 0.2f : 0.0f,
        .session_id = session,
        .sequence   = sequence,
        .enable     = enable,
        .mode       = 2U,
    };
}

int main(void)
{
    communication_wire_target_t      target;
    communication_session_snapshot_t host;
    communication_session_snapshot_t esp;

    CommunicationSessionTracker_Init();
    target = Target(11ULL, 0xFFFFFFFFUL, 0U);
    assert(CommunicationSessionTracker_Evaluate(COMMUNICATION_LINK_UPPER, &target, 1U)
           == COMMUNICATION_SESSION_DISABLE);
    CommunicationSessionTracker_Complete(COMMUNICATION_LINK_UPPER, target.sequence, 1U, COMMUNICATION_REJECT_NONE);

    target = Target(11ULL, 0U, 1U);
    assert(CommunicationSessionTracker_Evaluate(COMMUNICATION_LINK_UPPER, &target, 2U)
           == COMMUNICATION_SESSION_NEW_COMMAND);
    CommunicationSessionTracker_Complete(COMMUNICATION_LINK_UPPER, target.sequence, 1U, COMMUNICATION_REJECT_NONE);
    assert(CommunicationSessionTracker_Evaluate(COMMUNICATION_LINK_UPPER, &target, 3U)
           == COMMUNICATION_SESSION_DUPLICATE_KEEPALIVE);

    target.linear_x = 0.2f;
    assert(CommunicationSessionTracker_Evaluate(COMMUNICATION_LINK_UPPER, &target, 4U)
           == COMMUNICATION_SESSION_REJECT_OUT_OF_ORDER);

    target = Target(22ULL, 1U, 1U);
    assert(CommunicationSessionTracker_Evaluate(COMMUNICATION_LINK_UPPER, &target, 5U)
           == COMMUNICATION_SESSION_REJECT_STALE);
    target.enable    = 0U;
    target.linear_x  = 0.0f;
    target.angular_z = 0.0f;
    assert(CommunicationSessionTracker_Evaluate(COMMUNICATION_LINK_UPPER, &target, 6U)
           == COMMUNICATION_SESSION_DISABLE);

    target = Target(11ULL, 1U, 0U);
    assert(CommunicationSessionTracker_Evaluate(COMMUNICATION_LINK_UPPER, &target, 7U)
           == COMMUNICATION_SESSION_REJECT_STALE);

    target = Target(33ULL, 1U, 0U);
    assert(CommunicationSessionTracker_Evaluate(COMMUNICATION_LINK_ESP12F, &target, 8U)
           == COMMUNICATION_SESSION_DISABLE);
    CommunicationSessionTracker_GetSnapshot(COMMUNICATION_LINK_UPPER, &host);
    CommunicationSessionTracker_GetSnapshot(COMMUNICATION_LINK_ESP12F, &esp);
    assert(host.session_id == 22ULL);
    assert(esp.session_id == 33ULL);
    puts("PASS: independent Upper Protocol v3 sessions");
    return 0;
}
