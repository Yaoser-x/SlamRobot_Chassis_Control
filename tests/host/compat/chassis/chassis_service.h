#ifndef CHASSIS_SERVICE_H
#define CHASSIS_SERVICE_H

#include <stdint.h>

#include "motor_driver.h"
#include "motion_control_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef motion_control_status_t chassis_service_snapshot_t;

    void    ChassisService_Init(void);
    void    ChassisService_Step(uint32_t now_ms);
    uint8_t ChassisService_IsStepActive(void);
    void    ChassisService_EmergencyStop(void);
    void    ChassisService_CancelTestMode(void);
    void    ChassisService_OpenLoopTest(int16_t left_permille, int16_t right_permille);
    void    ChassisService_RawInputTest(int16_t left_forward_permille,
                                        int16_t left_reverse_permille,
                                        int16_t right_forward_permille,
                                        int16_t right_reverse_permille);
    void    ChassisService_RawMotorInputTest(motor_id_t motor, int16_t forward_permille, int16_t reverse_permille);
    void    ChassisService_ResolveSideTargets(float linear_x, float angular_z, float *left_mps, float *right_mps);
    void    ChassisService_GetState(chassis_service_snapshot_t *state);

#ifdef __cplusplus
}
#endif

#endif
