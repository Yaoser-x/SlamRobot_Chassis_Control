#ifndef REMOTE_OPERATION_ORCHESTRATOR_H
#define REMOTE_OPERATION_ORCHESTRATOR_H

#include "communication_types.h"

#include <stdint.h>

communication_operation_stage_t RemoteOperationOrchestrator_Dispatch(const communication_operation_request_t *request,
                                                                     uint32_t *detail_mask);

#endif
