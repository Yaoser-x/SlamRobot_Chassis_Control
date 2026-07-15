#include "chassis_output_service.h"

#include "bsp_config.h"
#include "chassis_layout.h"
#include "control_config.h"
#include "current_guard.h"
#include "motor_driver.h"

int16_t ChassisOutputService_Clamp(int32_t permille)
{
    if (permille > CHASSIS_PWM_MAX_PERMILLE)
    {
        return CHASSIS_PWM_MAX_PERMILLE;
    }
    if (permille < -CHASSIS_PWM_MAX_PERMILLE)
    {
        return -CHASSIS_PWM_MAX_PERMILLE;
    }
    return (int16_t)permille;
}

int16_t ChassisOutputService_MpsToPermille(float target_mps)
{
    int32_t permille;

    if (CHASSIS_OPENLOOP_FULL_MPS <= 0.0f)
    {
        return 0;
    }
    if (target_mps > CHASSIS_OPENLOOP_FULL_MPS)
    {
        target_mps = CHASSIS_OPENLOOP_FULL_MPS;
    }
    else if (target_mps < -CHASSIS_OPENLOOP_FULL_MPS)
    {
        target_mps = -CHASSIS_OPENLOOP_FULL_MPS;
    }
    permille = (int32_t)((target_mps / CHASSIS_OPENLOOP_FULL_MPS) * (float)CHASSIS_PWM_MAX_PERMILLE);
    return ChassisOutputService_Clamp(permille);
}

int16_t ChassisOutputService_ApplyCurrentLimit(motor_id_t                 motor,
                                               int16_t                    permille,
                                               const adc_monitor_state_t *adc,
                                               uint8_t                   *limited)
{
    return ChassisOutputService_Clamp(CurrentGuard_ApplyMotorLimit(motor, permille, adc, 0U, limited));
}

void ChassisOutputService_SetMotorWithAdc(chassis_service_snapshot_t *snapshot,
                                          motor_id_t                  motor,
                                          int16_t                     permille,
                                          const adc_monitor_state_t  *adc)
{
    int16_t applied;

    if (snapshot == 0 || adc == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return;
    }
    if (ChassisLayout_MotorEnabled(motor) == 0U)
    {
        snapshot->motor_current_limited[motor] = 0U;
        snapshot->motor_output_permille[motor] = 0;
        MotorDriver_SetPermille(motor, 0);
        return;
    }
    snapshot->motor_current_limited[motor] = 0U;
    applied                                = ChassisOutputService_ApplyCurrentLimit(motor,
                                                     ChassisOutputService_Clamp(permille),
                                                     adc,
                                                     &snapshot->motor_current_limited[motor]);
    snapshot->motor_output_permille[motor] = applied;
    MotorDriver_SetPermille(motor, applied);
}

void ChassisOutputService_SetMotor(chassis_service_snapshot_t *snapshot, motor_id_t motor, int16_t permille)
{
    adc_monitor_state_t adc;

    AdcMonitor_GetState(&adc);
    ChassisOutputService_SetMotorWithAdc(snapshot, motor, permille, &adc);
}

uint8_t ChassisOutputService_AnyActive(const chassis_service_snapshot_t *snapshot)
{
    if (snapshot == 0)
    {
        return 0U;
    }
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)index) != 0U && snapshot->motor_output_permille[index] != 0)
        {
            return 1U;
        }
    }
    return 0U;
}
