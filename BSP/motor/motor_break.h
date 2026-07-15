#ifndef MOTOR_BREAK_H
#define MOTOR_BREAK_H

#include <stdint.h>

typedef enum
{
    MOTOR_BREAK_ORIGIN_NONE = 0,
    MOTOR_BREAK_ORIGIN_STARTUP_TIMEOUT,
    MOTOR_BREAK_ORIGIN_TIM1_RUNTIME
} motor_break_origin_t;

typedef struct
{
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
} motor_break_snapshot_t;

/** Reset break diagnostics, disable TIM1 outputs and capture pre-wake BIF. */
void MotorBreak_Init(void);

/** Complete startup qualification and enable or latch TIM1 outputs. */
uint8_t MotorBreak_CompleteStartup(uint8_t qualified, uint8_t nfault_high_mask, uint32_t now_ms);

/** Latch a runtime TIM1 break and immediately clear all PWM compares. */
void MotorBreak_OnTim1Runtime(uint32_t now_ms);

/** Sample TIM1/TIM8 break status; return non-zero when TIM1 became latched. */
uint8_t MotorBreak_Update(uint32_t now_ms);

/** Clear a latched TIM1 break when external and output safety conditions allow. */
uint8_t MotorBreak_ClearLatch(uint8_t external_safe);

/** Return non-zero when the TIM1 BKIN input is currently high. */
uint8_t MotorBreak_IsBkinHigh(void);

/** Copy the current break and startup diagnostics. */
void MotorBreak_GetSnapshot(motor_break_snapshot_t *snapshot);

#endif
