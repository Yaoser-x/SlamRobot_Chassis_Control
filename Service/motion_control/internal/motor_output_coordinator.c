#include "motor_output_coordinator.h"

#include "motor_hardware_layout.h"
#include "motor_current_limiter.h"
#include "motor_driver.h"

static float output_full_speed_mps;

void MotorOutputCoordinator_Init(const motion_control_config_t *config)
{
    output_full_speed_mps = (config != 0) ? config->open_loop_full_mps : 0.0f;
}

int16_t MotorOutputCoordinator_Clamp(int32_t permille)
{
    if (permille > MOTOR_DRIVER_MAX_PERMILLE)
    {
        return MOTOR_DRIVER_MAX_PERMILLE;
    }
    if (permille < -MOTOR_DRIVER_MAX_PERMILLE)
    {
        return -MOTOR_DRIVER_MAX_PERMILLE;
    }
    return (int16_t)permille;
}

int16_t MotorOutputCoordinator_MpsToPermille(float target_mps)
{
    int32_t permille;

    if (output_full_speed_mps <= 0.0f)
    {
        return 0;
    }
    if (target_mps > output_full_speed_mps)
    {
        target_mps = output_full_speed_mps;
    }
    else if (target_mps < -output_full_speed_mps)
    {
        target_mps = -output_full_speed_mps;
    }
    permille = (int32_t)((target_mps / output_full_speed_mps) * (float)MOTOR_DRIVER_MAX_PERMILLE);
    return MotorOutputCoordinator_Clamp(permille);
}

int16_t MotorOutputCoordinator_ApplyCurrentLimit(motor_id_t                       motor,
                                                 int16_t                          permille,
                                                 const power_management_status_t *power_status,
                                                 const param_model_t             *params,
                                                 uint8_t                         *limited)
{
    return MotorOutputCoordinator_Clamp(
        MotorCurrentLimiter_ApplyMotorLimit(motor, permille, power_status, params, 0U, limited));
}

void MotorOutputCoordinator_SetMotorWithPower(motion_control_status_t         *snapshot,
                                              motor_id_t                       motor,
                                              int16_t                          permille,
                                              const power_management_status_t *power_status,
                                              const param_model_t             *params)
{
    int16_t applied;

    if (snapshot == 0 || power_status == 0 || params == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return;
    }
    if (MotorHardwareLayout_MotorEnabled(motor) == 0U)
    {
        snapshot->motor_current_limited[motor] = 0U;
        snapshot->motor_output_permille[motor] = 0;
        MotorDriver_SetPermille(motor, 0);
        return;
    }
    snapshot->motor_current_limited[motor] = 0U;
    applied                                = MotorOutputCoordinator_ApplyCurrentLimit(motor,
                                                       MotorOutputCoordinator_Clamp(permille),
                                                       power_status,
                                                       params,
                                                       &snapshot->motor_current_limited[motor]);
    snapshot->motor_output_permille[motor] = applied;
    MotorDriver_SetPermille(motor, applied);
}

void MotorOutputCoordinator_StopMotor(motion_control_status_t *snapshot, motor_id_t motor)
{
    if (snapshot == 0 || (uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return;
    }
    snapshot->motor_current_limited[motor] = 0U;
    snapshot->motor_output_permille[motor] = 0;
    MotorDriver_SetPermille(motor, 0);
}

uint8_t MotorOutputCoordinator_AnyActive(const motion_control_status_t *snapshot)
{
    if (snapshot == 0)
    {
        return 0U;
    }
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        if (MotorHardwareLayout_MotorEnabled((motor_id_t)index) != 0U && snapshot->motor_output_permille[index] != 0)
        {
            return 1U;
        }
    }
    return 0U;
}
