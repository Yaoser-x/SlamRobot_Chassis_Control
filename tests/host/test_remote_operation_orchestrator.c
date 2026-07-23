#include "remote_operation_orchestrator.h"

#include "line_following_service.h"
#include "safety_management_service.h"

#include <assert.h>
#include <stdio.h>

static uint8_t                 fake_estop;
static line_following_result_t fake_line_result;
static safety_clear_result_t   fake_clear_result;

void SafetyManagement_SetEmergencyStop(uint8_t enabled)
{
    fake_estop = enabled;
}

uint8_t SafetyManagement_IsEmergencyStop(void)
{
    return fake_estop;
}

safety_clear_result_t SafetyManagement_ClearLatchedFaults(uint32_t mask)
{
    fake_clear_result.requested_mask = mask;
    return fake_clear_result;
}

line_following_result_t LineFollowing_Enable(uint8_t enabled)
{
    (void)enabled;
    return fake_line_result;
}

int main(void)
{
    communication_operation_request_t request = {.kind = COMMUNICATION_OPERATION_ESTOP};
    uint32_t                          detail;

    assert(RemoteOperationOrchestrator_Dispatch(&request, &detail) == COMMUNICATION_OPERATION_BUSINESS_APPLIED);
    assert(fake_estop == 1U);

    request.kind      = COMMUNICATION_OPERATION_CLEAR_FAULT;
    fake_clear_result = (safety_clear_result_t){
        .code           = SAFETY_CLEAR_RESULT_CONDITION_NOT_CLEARED,
        .remaining_mask = 0x42U,
    };
    assert(RemoteOperationOrchestrator_Dispatch(&request, &detail) == COMMUNICATION_OPERATION_CONDITION_NOT_CLEARED);
    assert(detail == 0x42U && fake_clear_result.requested_mask == 0xFFFFFFFFUL);

    request.kind     = COMMUNICATION_OPERATION_LINE_CTRL;
    fake_line_result = LINE_FOLLOWING_RESULT_REJECTED;
    assert(RemoteOperationOrchestrator_Dispatch(&request, &detail) == COMMUNICATION_OPERATION_BUSINESS_REJECTED);
    fake_line_result = LINE_FOLLOWING_RESULT_APPLIED;
    assert(RemoteOperationOrchestrator_Dispatch(&request, &detail) == COMMUNICATION_OPERATION_BUSINESS_APPLIED);

    puts("PASS: App remote operation orchestrator");
    return 0;
}
