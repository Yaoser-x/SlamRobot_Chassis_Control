/*
 * Copyright 2026 Your Name. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef OLED_UI_H
#define OLED_UI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        OLED_PHASE_WELCOME = 0,
        OLED_PHASE_SELFCHECK,
        OLED_PHASE_NORMAL
    } oled_phase_t;

/* Self-check error bitmask (bits 9~16, to avoid conflict with motor/battery bits 0~8) */
#define OLED_SC_ERROR_I2C        (1UL << 9)
#define OLED_SC_ERROR_IMU        (1UL << 10)
#define OLED_SC_ERROR_ADC        (1UL << 11)
#define OLED_SC_ERROR_MOTOR      (1UL << 12)
#define OLED_SC_ERROR_ENCODER    (1UL << 13)
#define OLED_SC_ERROR_UART3_RPI  (1UL << 14)
#define OLED_SC_ERROR_UART4_LINE (1UL << 15)
#define OLED_SC_ERROR_ESP12F     (1UL << 16)

    void         OLED_UI_Init(void);
    void         OLED_UI_Update(void);
    oled_phase_t OLED_UI_GetPhase(void);
    uint8_t      OLED_UI_SelfCheckDone(void);
    uint32_t     OLED_UI_GetSelfCheckErrors(void);

#ifdef __cplusplus
}
#endif

#endif /* OLED_UI_H */
