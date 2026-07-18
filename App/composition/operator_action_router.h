#ifndef OPERATOR_ACTION_ROUTER_H
#define OPERATOR_ACTION_ROUTER_H

#include "teleoperation_action_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Handle one Teleoperation action by dispatching to the appropriate Service or Adapter. */
    void OperatorActionRouter_Handle(const teleoperation_action_t *action);

#ifdef __cplusplus
}
#endif

#endif /* OPERATOR_ACTION_ROUTER_H */