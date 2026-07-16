#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>

#include "motor_types.h"
#include "wheel_estimation_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef wheel_estimation_t encoder_state_t;

    typedef struct
    {
        float  wheel_radius_m;
        int8_t encoder_dir[MOTOR_ID_COUNT];
    } encoder_driver_config_t;

    void EncoderDriver_Init(void);
    void EncoderDriver_Update(uint32_t now_ms, const encoder_driver_config_t *config);
    void EncoderDriver_GetState(encoder_state_t *state);
    /** Read the raw hardware timer count for each logical motor. */
    void    EncoderDriver_GetHardwareCounts(uint32_t counts[MOTOR_ID_COUNT]);
    float   EncoderDriver_GetCountsPerRev(void);
    float   EncoderDriver_GetMotorSpeedMps(motor_id_t motor);
    int32_t EncoderDriver_DiffCount(uint32_t now, uint32_t last, uint32_t period);

#ifdef __cplusplus
}
#endif

#endif
