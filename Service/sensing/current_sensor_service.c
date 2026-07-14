#include "current_sensor_service.h"

#include "chassis_layout.h"
#include "encoder_service.h"
#include "motor_driver.h"

#define CURRENT_ZERO_MAX_SPEED_MPS 0.02f

void CurrentSensorService_Init(void)
{
    AdcMonitor_Init();
}

void CurrentSensorService_UpdateStationary(void)
{
    encoder_service_snapshot_t encoder;
    motor_driver_state_t       motor;
    uint8_t                    stationary = 1U;

    EncoderService_GetSnapshot(&encoder);
    MotorDriver_GetState(&motor);
    for (uint8_t index = 0U; index < MOTOR_ID_COUNT; ++index)
    {
        if (motor.effective_pwm[index] != 0
            || (ChassisLayout_MotorEnabled((motor_id_t)index) != 0U
                && (encoder.speed_valid[index] == 0U || encoder.speed_mps[index] < -CURRENT_ZERO_MAX_SPEED_MPS
                    || encoder.speed_mps[index] > CURRENT_ZERO_MAX_SPEED_MPS)))
        {
            stationary = 0U;
            break;
        }
    }
    AdcMonitor_SetCurrentZeroStationary(stationary);
}

void CurrentSensorService_ApplyZeroCalibration(const uint16_t zero_raw[MOTOR_ID_COUNT])
{
    AdcMonitor_ApplyCurrentZeroCalibration(zero_raw);
}

void CurrentSensorService_GetSnapshot(current_sensor_snapshot_t *snapshot)
{
    AdcMonitor_GetState(snapshot);
}
