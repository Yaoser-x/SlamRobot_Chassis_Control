#ifndef MOTION_CONTROL_SERVICE_H
#define MOTION_CONTROL_SERVICE_H

#include <stdint.h>

#include "motion_control_config.h"
#include "motion_control_status.h"
#include "motion_control_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** Validate product configuration without side effects. */
    uint8_t MotionControl_ValidateConfig(const motion_control_config_t *config);
    /** Initialize the sole runtime motor-output owner with injected product configuration. */
    uint8_t MotionControl_Init(const motion_control_config_t *config);
    /** Execute one cycle from an App-composed fact set and return cross-Service events. */
    void MotionControl_StepWithInput(const motion_control_input_t *input, motion_control_event_t *event);
    /** Return whether a motion-control cycle is currently applying outputs. */
    uint8_t MotionControl_IsStepActive(void);
    /** Copy one complete published motion status and return its generation. */
    uint32_t MotionControl_GetStatus(motion_control_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROL_SERVICE_H */
