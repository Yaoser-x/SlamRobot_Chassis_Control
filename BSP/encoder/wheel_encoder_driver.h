#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>

#include "motor_types.h"
#include "wheel_encoder_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float  wheel_radius_m;
        int8_t encoder_dir[MOTOR_ID_COUNT];
    } wheel_encoder_driver_config_t;

    void WheelEncoderDriver_Init(void);
    void WheelEncoderDriver_Update(uint32_t now_ms, const wheel_encoder_driver_config_t *config);
    void WheelEncoderDriver_GetState(wheel_encoder_state_t *state);
    /** Read the raw hardware timer count for each logical motor. */
    void  WheelEncoderDriver_GetHardwareCounts(uint32_t counts[MOTOR_ID_COUNT]);
    float WheelEncoderDriver_GetCountsPerRev(void);
    float
    WheelEncoderDriver_CountDeltaSpeedMps(int32_t delta, uint32_t dt_ms, float counts_per_rev, float wheel_radius_m);
    float   WheelEncoderDriver_GetMotorSpeedMps(motor_id_t motor);
    int32_t WheelEncoderDriver_DiffCount(uint32_t now, uint32_t last, uint32_t period);

#ifdef __cplusplus
}
#endif

#endif
