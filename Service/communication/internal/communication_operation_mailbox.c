#include "communication_operation_mailbox.h"

#include "platform_critical.h"

typedef struct
{
    communication_operation_request_t queue[COMMUNICATION_OPERATION_QUEUE_CAPACITY];
    communication_operation_request_t sticky_estop;
    communication_operation_result_t  last_result;
    uint32_t                          next_generation;
    uint8_t                           head;
    uint8_t                           count;
    uint8_t                           estop_pending;
} communication_operation_mailbox_t;

static communication_operation_mailbox_t operation_mailboxes[COMMUNICATION_LINK_COUNT];

static uint8_t CommunicationOperationMailbox_LinkValid(communication_link_t link)
{
    return (link < COMMUNICATION_LINK_COUNT) ? 1U : 0U;
}

static uint32_t CommunicationOperationMailbox_NextGeneration(communication_operation_mailbox_t *mailbox)
{
    mailbox->next_generation++;
    if (mailbox->next_generation == 0U)
    {
        mailbox->next_generation = 1U;
    }
    return mailbox->next_generation;
}

static void CommunicationOperationMailbox_Record(communication_operation_mailbox_t       *mailbox,
                                                 const communication_operation_request_t *request,
                                                 communication_operation_stage_t          stage,
                                                 uint32_t                                 detail_mask,
                                                 uint32_t                                 now_ms)
{
    mailbox->last_result = (communication_operation_result_t){
        .kind          = request->kind,
        .link          = request->link,
        .stage         = stage,
        .generation    = request->generation,
        .updated_at_ms = now_ms,
        .detail_mask   = detail_mask,
    };
}

void CommunicationOperationMailbox_ResetLink(communication_link_t link)
{
    platform_critical_state_t critical;

    if (CommunicationOperationMailbox_LinkValid(link) == 0U)
    {
        return;
    }
    critical                  = PlatformCritical_Enter();
    operation_mailboxes[link] = (communication_operation_mailbox_t){0};
    PlatformCritical_Exit(critical);
}

uint8_t CommunicationOperationMailbox_Enqueue(communication_link_t           link,
                                              communication_operation_kind_t kind,
                                              uint8_t                        value,
                                              uint32_t                       now_ms)
{
    communication_operation_mailbox_t *mailbox;
    communication_operation_request_t  request;
    platform_critical_state_t          critical;

    if (CommunicationOperationMailbox_LinkValid(link) == 0U || kind > COMMUNICATION_OPERATION_LINE_CTRL)
    {
        return 0U;
    }
    critical = PlatformCritical_Enter();
    mailbox  = &operation_mailboxes[link];
    request  = (communication_operation_request_t){
         .kind           = kind,
         .link           = link,
         .generation     = CommunicationOperationMailbox_NextGeneration(mailbox),
         .received_at_ms = now_ms,
         .value          = value,
    };
    if (kind == COMMUNICATION_OPERATION_ESTOP)
    {
        mailbox->sticky_estop  = request;
        mailbox->estop_pending = 1U;
    }
    else
    {
        uint8_t tail;

        if (mailbox->count >= COMMUNICATION_OPERATION_QUEUE_CAPACITY)
        {
            CommunicationOperationMailbox_Record(mailbox,
                                                 &request,
                                                 COMMUNICATION_OPERATION_BUSINESS_REJECTED,
                                                 0U,
                                                 now_ms);
            PlatformCritical_Exit(critical);
            return 0U;
        }
        tail                 = (uint8_t)((mailbox->head + mailbox->count) % COMMUNICATION_OPERATION_QUEUE_CAPACITY);
        mailbox->queue[tail] = request;
        mailbox->count++;
    }
    CommunicationOperationMailbox_Record(mailbox, &request, COMMUNICATION_OPERATION_FRAME_ACCEPTED, 0U, now_ms);
    PlatformCritical_Exit(critical);
    return 1U;
}

uint8_t CommunicationOperationMailbox_Take(communication_link_t               link,
                                           uint32_t                           now_ms,
                                           communication_operation_request_t *request)
{
    communication_operation_mailbox_t *mailbox;
    platform_critical_state_t          critical;

    if (request == 0 || CommunicationOperationMailbox_LinkValid(link) == 0U)
    {
        return 0U;
    }
    critical = PlatformCritical_Enter();
    mailbox  = &operation_mailboxes[link];
    if (mailbox->estop_pending != 0U)
    {
        *request               = mailbox->sticky_estop;
        mailbox->estop_pending = 0U;
        CommunicationOperationMailbox_Record(mailbox, request, COMMUNICATION_OPERATION_REQUEST_DISPATCHED, 0U, now_ms);
        PlatformCritical_Exit(critical);
        return 1U;
    }
    while (mailbox->count != 0U)
    {
        *request      = mailbox->queue[mailbox->head];
        mailbox->head = (uint8_t)((mailbox->head + 1U) % COMMUNICATION_OPERATION_QUEUE_CAPACITY);
        mailbox->count--;
        if ((uint32_t)(now_ms - request->received_at_ms) > COMMUNICATION_OPERATION_TIMEOUT_MS)
        {
            CommunicationOperationMailbox_Record(mailbox, request, COMMUNICATION_OPERATION_TIMEOUT, 0U, now_ms);
            continue;
        }
        CommunicationOperationMailbox_Record(mailbox, request, COMMUNICATION_OPERATION_REQUEST_DISPATCHED, 0U, now_ms);
        PlatformCritical_Exit(critical);
        return 1U;
    }
    PlatformCritical_Exit(critical);
    return 0U;
}

void CommunicationOperationMailbox_Complete(const communication_operation_request_t *request,
                                            communication_operation_stage_t          stage,
                                            uint32_t                                 detail_mask,
                                            uint32_t                                 now_ms)
{
    platform_critical_state_t critical;

    if (request == 0 || CommunicationOperationMailbox_LinkValid(request->link) == 0U
        || stage < COMMUNICATION_OPERATION_BUSINESS_APPLIED || stage > COMMUNICATION_OPERATION_TIMEOUT)
    {
        return;
    }
    critical = PlatformCritical_Enter();
    if ((uint32_t)(now_ms - request->received_at_ms) > COMMUNICATION_OPERATION_TIMEOUT_MS)
    {
        stage = COMMUNICATION_OPERATION_TIMEOUT;
    }
    CommunicationOperationMailbox_Record(&operation_mailboxes[request->link], request, stage, detail_mask, now_ms);
    PlatformCritical_Exit(critical);
}

void CommunicationOperationMailbox_GetLastResult(communication_link_t link, communication_operation_result_t *result)
{
    platform_critical_state_t critical;

    if (result == 0 || CommunicationOperationMailbox_LinkValid(link) == 0U)
    {
        return;
    }
    critical = PlatformCritical_Enter();
    *result  = operation_mailboxes[link].last_result;
    PlatformCritical_Exit(critical);
}
