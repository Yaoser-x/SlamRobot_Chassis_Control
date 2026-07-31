#ifndef MOTOR_DRIVER_SNAPSHOT_ADAPTER_H
#define MOTOR_DRIVER_SNAPSHOT_ADAPTER_H

#include <stdint.h>

#define APP_MOTOR_DRIVER_COUNT 4U

typedef struct
{
    int16_t  requested_pwm[APP_MOTOR_DRIVER_COUNT];
    int16_t  applied_pwm[APP_MOTOR_DRIVER_COUNT];
    int16_t  effective_pwm[APP_MOTOR_DRIVER_COUNT];
    uint8_t  fault_active[APP_MOTOR_DRIVER_COUNT];
    uint8_t  tim1_break_latched;
    uint8_t  enabled_mask;
    uint32_t generation;
} app_motor_driver_snapshot_t;

/** Refresh hardware fault state and advance the adapter-owned fact generation. */
void AppMotorDriverAdapter_UpdateFaults(void);

/** Copy a hardware-neutral motor fact for App/composition workflows. */
uint32_t AppMotorDriverAdapter_GetSnapshot(app_motor_driver_snapshot_t *snapshot);

/** Attempt to clear the hardware break latch and advance generation only on success. */
uint8_t AppMotorDriverAdapter_ClearBreakLatch(void);

#endif /* MOTOR_DRIVER_SNAPSHOT_ADAPTER_H */
