#ifndef MOTION_CONTROL_SERVICE_H
#define MOTION_CONTROL_SERVICE_H

#include <stdint.h>

#include "motion_control_config.h"
#include "motion_control_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MOTION_CONTROL_MAINTENANCE_OK = 0,
        MOTION_CONTROL_MAINTENANCE_BUSY,
        MOTION_CONTROL_MAINTENANCE_NOT_STATIONARY
    } motion_control_maintenance_result_t;

    /** Initialize the sole runtime motor-output owner with injected product configuration. */
    uint8_t MotionControl_Init(const motion_control_config_t *config);
    /** Execute one motion-control cycle at the supplied monotonic timestamp. */
    void MotionControl_Step(uint32_t now_ms);
    /** Return whether a motion-control cycle is currently applying outputs. */
    uint8_t MotionControl_IsStepActive(void);
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
    /** Apply or refresh the gated raw input test for one motor identifier. */
    void MotionControl_RawMotorInputTest(uint8_t motor, int16_t forward_permille, int16_t reverse_permille);
    /** Resolve differential side targets using the active runtime parameter model. */
    void MotionControl_ResolveSideTargets(float linear_x, float angular_z, float *left_mps, float *right_mps);
    /** Copy one complete published motion status and return its generation. */
    uint32_t MotionControl_GetStatus(motion_control_status_t *status);
    /** Acquire maintenance ownership after stopping and verifying the drivetrain. */
    motion_control_maintenance_result_t MotionControl_BeginMaintenance(void);
    /** Release maintenance ownership acquired through Motion Control. */
    void MotionControl_EndMaintenance(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_CONTROL_SERVICE_H */
