#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#include "motor_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MOTOR_STOP_LOW_SIDE_BRAKE = 0
    } motor_stop_mode_t;

    typedef enum
    {
        MOTOR_DRIVER_PHASE_IDLE_BRAKE = 0,
        MOTOR_DRIVER_PHASE_RUN,
        MOTOR_DRIVER_PHASE_RAMP_DOWN,
        MOTOR_DRIVER_PHASE_REVERSE_BRAKE,
        MOTOR_DRIVER_PHASE_PH_SETTLE,
        MOTOR_DRIVER_PHASE_RAMP_UP
    } motor_driver_phase_t;

    typedef enum
    {
        MOTOR_BREAK_ORIGIN_NONE = 0,
        MOTOR_BREAK_ORIGIN_STARTUP_TIMEOUT,
        MOTOR_BREAK_ORIGIN_TIM1_RUNTIME
    } motor_break_origin_t;

    typedef struct
    {
        uint8_t              fault_active[MOTOR_ID_COUNT];
        int16_t              output_permille[MOTOR_ID_COUNT];
        int16_t              requested_pwm[MOTOR_ID_COUNT];
        int16_t              applied_pwm[MOTOR_ID_COUNT];
        int16_t              effective_pwm[MOTOR_ID_COUNT];
        int8_t               current_ph_dir[MOTOR_ID_COUNT];
        int8_t               pending_dir[MOTOR_ID_COUNT];
        motor_driver_phase_t phase[MOTOR_ID_COUNT];
        uint8_t              sleep_enabled;
        uint8_t              tim1_moe_active;
        uint8_t              tim1_break_flag;
        uint8_t              tim1_break_latched;
        uint8_t              tim8_moe_active;
        uint8_t              tim8_break_flag;
        uint32_t             tim1_break_count;
        uint32_t             tim8_break_count;
        uint32_t             tim1_break_last_ms;
        uint32_t             tim8_break_last_ms;
        uint8_t              startup_qualified;
        uint8_t              startup_pre_wake_bif;
        uint8_t              startup_bkin_high;
        uint8_t              startup_nfault_high_mask;
        motor_break_origin_t break_origin;
        uint32_t             fault_edge_count[MOTOR_ID_COUNT];
        uint32_t             fault_last_change_ms[MOTOR_ID_COUNT];
        uint32_t             fault_low_since_ms[MOTOR_ID_COUNT];
    } motor_driver_state_t;

    typedef float (*motor_speed_getter_t)(motor_id_t motor);

    void    MotorDriver_Init(void);
    void    MotorDriver_SetSpeedGetter(motor_speed_getter_t getter);
    void    MotorDriver_SetDirectionConfig(const int8_t direction[MOTOR_ID_COUNT]);
    void    MotorDriver_SetPermille(motor_id_t motor, int16_t permille);
    void    MotorDriver_SetSidePermille(motor_side_t side, int16_t permille);
    void    MotorDriver_Stop(motor_id_t motor, motor_stop_mode_t mode);
    void    MotorDriver_StopSide(motor_side_t side, motor_stop_mode_t mode);
    void    MotorDriver_StopAll(motor_stop_mode_t mode);
    void    MotorDriver_UpdateFaults(void);
    uint8_t MotorDriver_HasFault(void);
    void    MotorDriver_OnTim1BreakFromIsr(void);
    uint8_t MotorDriver_ClearBreakLatch(void);
    void    MotorDriver_GetState(motor_driver_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
