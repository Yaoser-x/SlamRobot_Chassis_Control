#ifndef MOTION_CONTROL_MAINTENANCE_H
#define MOTION_CONTROL_MAINTENANCE_H

#include <stdint.h>

#include "motion_control_config.h"
#include "parameter_management_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** Revoke commands, reset controllers, and stop all motion outputs. */
    void MotionControl_EmergencyStop(void);
    /** Cancel every active debug motor-test lease. */
    void MotionControl_CancelTestMode(void);
    /** Apply or refresh the gated side open-loop test command. */
    void MotionControl_OpenLoopTest(int16_t left_permille, int16_t right_permille);
    /** Apply or refresh the gated per-side raw motor-input test command. */
    void MotionControl_RawInputTest(int16_t left_forward_permille,
                                    int16_t left_reverse_permille,
                                    int16_t right_forward_permille,
                                    int16_t right_reverse_permille);
    /** Apply or refresh the gated raw input test for motor identifier. */
    void MotionControl_RawMotorInputTest(uint8_t motor, int16_t forward_permille, int16_t reverse_permille);
    /** Resolve differential side targets using the active runtime parameter model. */
    void MotionControl_ResolveSideTargetsWithParameters(float                linear_x,
                                                        float                angular_z,
                                                        const param_model_t *params,
                                                        float               *left_mps,
                                                        float               *right_mps);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROL_MAINTENANCE_H */
