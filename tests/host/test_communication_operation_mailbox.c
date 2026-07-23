#include "communication_operation_mailbox.h"

#include <stdio.h>
#include <stdlib.h>

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static void test_sticky_estop_preempts_full_queue(void)
{
    communication_operation_request_t request;

    CommunicationOperationMailbox_ResetLink(COMMUNICATION_LINK_UPPER);
    for (uint8_t i = 0U; i < COMMUNICATION_OPERATION_QUEUE_CAPACITY; ++i)
    {
        require_int(CommunicationOperationMailbox_Enqueue(COMMUNICATION_LINK_UPPER,
                                                          COMMUNICATION_OPERATION_LINE_CTRL,
                                                          i & 1U,
                                                          i)
                        != 0U,
                    "ordinary request fills fixed queue");
    }
    require_int(CommunicationOperationMailbox_Enqueue(COMMUNICATION_LINK_UPPER, COMMUNICATION_OPERATION_ESTOP, 1U, 10U)
                    != 0U,
                "ESTOP remains enqueueable when ordinary queue is full");
    require_int(CommunicationOperationMailbox_Take(COMMUNICATION_LINK_UPPER, 11U, &request) != 0U,
                "sticky ESTOP is available");
    require_int(request.kind == COMMUNICATION_OPERATION_ESTOP, "sticky ESTOP dispatches first");
}

static void test_overflow_and_timeout_are_formal_results(void)
{
    communication_operation_request_t request;
    communication_operation_result_t  result;

    CommunicationOperationMailbox_ResetLink(COMMUNICATION_LINK_ESP12F);
    for (uint8_t i = 0U; i < COMMUNICATION_OPERATION_QUEUE_CAPACITY; ++i)
    {
        (void)CommunicationOperationMailbox_Enqueue(COMMUNICATION_LINK_ESP12F,
                                                    COMMUNICATION_OPERATION_CLEAR_FAULT,
                                                    1U,
                                                    0U);
    }
    require_int(
        CommunicationOperationMailbox_Enqueue(COMMUNICATION_LINK_ESP12F, COMMUNICATION_OPERATION_CLEAR_FAULT, 1U, 0U)
            == 0U,
        "ordinary overflow is rejected");
    CommunicationOperationMailbox_GetLastResult(COMMUNICATION_LINK_ESP12F, &result);
    require_int(result.stage == COMMUNICATION_OPERATION_BUSINESS_REJECTED, "overflow records business rejected stage");
    require_int(CommunicationOperationMailbox_Take(COMMUNICATION_LINK_ESP12F, 101U, &request) == 0U,
                "all requests older than 100 ms expire instead of dispatching");
    CommunicationOperationMailbox_GetLastResult(COMMUNICATION_LINK_ESP12F, &result);
    require_int(result.stage == COMMUNICATION_OPERATION_TIMEOUT, "expired request records timeout stage");
}

static void test_completion_is_link_local(void)
{
    communication_operation_request_t request;
    communication_operation_result_t  upper_result;
    communication_operation_result_t  esp_result;

    CommunicationOperationMailbox_ResetLink(COMMUNICATION_LINK_UPPER);
    CommunicationOperationMailbox_ResetLink(COMMUNICATION_LINK_ESP12F);
    (void)CommunicationOperationMailbox_Enqueue(COMMUNICATION_LINK_UPPER, COMMUNICATION_OPERATION_LINE_CTRL, 1U, 5U);
    require_int(CommunicationOperationMailbox_Take(COMMUNICATION_LINK_UPPER, 6U, &request) != 0U,
                "upper request dispatches");
    CommunicationOperationMailbox_Complete(&request, COMMUNICATION_OPERATION_BUSINESS_APPLIED, 0U, 7U);
    CommunicationOperationMailbox_GetLastResult(COMMUNICATION_LINK_UPPER, &upper_result);
    CommunicationOperationMailbox_GetLastResult(COMMUNICATION_LINK_ESP12F, &esp_result);
    require_int(upper_result.stage == COMMUNICATION_OPERATION_BUSINESS_APPLIED,
                "completion is retained on originating link");
    require_int(esp_result.generation == 0U, "other link result is untouched");

    CommunicationOperationMailbox_ResetLink(COMMUNICATION_LINK_UPPER);
    (void)CommunicationOperationMailbox_Enqueue(COMMUNICATION_LINK_UPPER, COMMUNICATION_OPERATION_CLEAR_FAULT, 1U, 0U);
    require_int(CommunicationOperationMailbox_Take(COMMUNICATION_LINK_UPPER, 1U, &request) != 0U,
                "request starts before timeout");
    CommunicationOperationMailbox_Complete(&request, COMMUNICATION_OPERATION_BUSINESS_APPLIED, 0U, 101U);
    CommunicationOperationMailbox_GetLastResult(COMMUNICATION_LINK_UPPER, &upper_result);
    require_int(upper_result.stage == COMMUNICATION_OPERATION_TIMEOUT,
                "completion after 100 ms is recorded as timeout");
}

int main(void)
{
    test_sticky_estop_preempts_full_queue();
    test_overflow_and_timeout_are_formal_results();
    test_completion_is_link_local();
    puts("PASS: communication operation mailbox");
    return 0;
}
