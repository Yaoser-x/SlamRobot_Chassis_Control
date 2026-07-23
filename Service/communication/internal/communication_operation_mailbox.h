#ifndef COMMUNICATION_OPERATION_MAILBOX_H
#define COMMUNICATION_OPERATION_MAILBOX_H

#include "communication_types.h"

#include <stdint.h>

#define COMMUNICATION_OPERATION_QUEUE_CAPACITY 8U
#define COMMUNICATION_OPERATION_TIMEOUT_MS     100U

void    CommunicationOperationMailbox_ResetLink(communication_link_t link);
uint8_t CommunicationOperationMailbox_Enqueue(communication_link_t           link,
                                              communication_operation_kind_t kind,
                                              uint8_t                        value,
                                              uint32_t                       now_ms);
uint8_t CommunicationOperationMailbox_Take(communication_link_t               link,
                                           uint32_t                           now_ms,
                                           communication_operation_request_t *request);
void    CommunicationOperationMailbox_Complete(const communication_operation_request_t *request,
                                               communication_operation_stage_t          stage,
                                               uint32_t                                 detail_mask,
                                               uint32_t                                 now_ms);
void CommunicationOperationMailbox_GetLastResult(communication_link_t link, communication_operation_result_t *result);

#endif
