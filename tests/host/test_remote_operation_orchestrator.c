#include "remote_operation_orchestrator.h"

#include "line_following_service.h"
#include "safety_management_service.h"
#include "control_mode_coordinator.h"
#include "safety_workflow_coordinator.h"

#include <assert.h>
#include <stdio.h>

static uint8_t                 fake_estop;
static line_following_result_t fake_line_result[2];
static uint8_t                 fake_line_calls[4];
static uint8_t                 fake_line_call_count;
static safety_clear_result_t   fake_clear_result;
static control_mode_t          fake_requested_mode;
static uint8_t                 fake_mode_result = 1U;

void AppSafetyWorkflow_SetEmergencyStop(uint8_t enabled)
{
    fake_estop = enabled;
}

uint8_t SafetyManagement_IsEmergencyStop(void)
{
    return fake_estop;
}

safety_clear_result_t AppSafetyWorkflow_ClearLatchedFaults(uint32_t mask)
{
    fake_clear_result.requested_mask = mask;
    return fake_clear_result;
}

line_following_result_t LineFollowing_Enable(uint8_t enabled)
{
    if (fake_line_call_count < sizeof(fake_line_calls))
    {
        fake_line_calls[fake_line_call_count] = (enabled != 0U) ? 1U : 0U;
    }
    fake_line_call_count++;
    return fake_line_result[(enabled != 0U) ? 1U : 0U];
}

uint8_t ControlModeCoordinator_Request(control_mode_t mode)
{
    fake_requested_mode = mode;
    return fake_mode_result;
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

    request.kind        = COMMUNICATION_OPERATION_LINE_CTRL;
    request.value       = 1U;
    fake_line_result[0] = LINE_FOLLOWING_RESULT_REJECTED;
    fake_line_result[1] = LINE_FOLLOWING_RESULT_APPLIED;
    assert(RemoteOperationOrchestrator_Dispatch(&request, &detail) == COMMUNICATION_OPERATION_BUSINESS_REJECTED);
    assert(fake_line_call_count == 1U && fake_line_calls[0] == 0U);

    fake_line_call_count = 0U;
    fake_line_result[0]  = LINE_FOLLOWING_RESULT_APPLIED;
    fake_line_result[1]  = LINE_FOLLOWING_RESULT_REJECTED;
    assert(RemoteOperationOrchestrator_Dispatch(&request, &detail) == COMMUNICATION_OPERATION_BUSINESS_REJECTED);
    assert(fake_line_call_count == 2U && fake_line_calls[0] == 0U && fake_line_calls[1] == 1U);

    fake_line_call_count = 0U;
    fake_line_result[1]  = LINE_FOLLOWING_RESULT_APPLIED;
    assert(RemoteOperationOrchestrator_Dispatch(&request, &detail) == COMMUNICATION_OPERATION_BUSINESS_APPLIED);
    assert(fake_line_call_count == 2U && fake_line_calls[0] == 0U && fake_line_calls[1] == 1U);
    assert(fake_requested_mode == CONTROL_MODE_LINE);
    request.value        = 0U;
    fake_line_call_count = 0U;
    assert(RemoteOperationOrchestrator_Dispatch(&request, &detail) == COMMUNICATION_OPERATION_BUSINESS_APPLIED);
    assert(fake_line_call_count == 1U && fake_line_calls[0] == 0U);
    assert(fake_requested_mode == CONTROL_MODE_AUTO);

    puts("PASS: App remote operation orchestrator");
    return 0;
}
