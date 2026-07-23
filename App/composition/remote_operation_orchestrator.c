#include "remote_operation_orchestrator.h"

#include "line_following_service.h"
#include "safety_management_service.h"

communication_operation_stage_t RemoteOperationOrchestrator_Dispatch(const communication_operation_request_t *request,
                                                                     uint32_t *detail_mask)
{
    if (detail_mask != 0)
    {
        *detail_mask = 0U;
    }
    if (request == 0)
    {
        return COMMUNICATION_OPERATION_BUSINESS_REJECTED;
    }
    switch (request->kind)
    {
        case COMMUNICATION_OPERATION_ESTOP:
            SafetyManagement_SetEmergencyStop(1U);
            return (SafetyManagement_IsEmergencyStop() != 0U) ? COMMUNICATION_OPERATION_BUSINESS_APPLIED
                                                              : COMMUNICATION_OPERATION_BUSINESS_REJECTED;
        case COMMUNICATION_OPERATION_CLEAR_FAULT:
        {
            safety_clear_result_t result = SafetyManagement_ClearLatchedFaults(0xFFFFFFFFUL);

            if (detail_mask != 0)
            {
                *detail_mask = result.remaining_mask;
            }
            if (result.code == SAFETY_CLEAR_RESULT_APPLIED)
            {
                return COMMUNICATION_OPERATION_BUSINESS_APPLIED;
            }
            if (result.code == SAFETY_CLEAR_RESULT_CONDITION_NOT_CLEARED)
            {
                return COMMUNICATION_OPERATION_CONDITION_NOT_CLEARED;
            }
            return COMMUNICATION_OPERATION_BUSINESS_REJECTED;
        }
        case COMMUNICATION_OPERATION_LINE_CTRL:
            return (LineFollowing_Enable(request->value) == LINE_FOLLOWING_RESULT_APPLIED)
                       ? COMMUNICATION_OPERATION_BUSINESS_APPLIED
                       : COMMUNICATION_OPERATION_BUSINESS_REJECTED;
        default:
            return COMMUNICATION_OPERATION_BUSINESS_REJECTED;
    }
}
