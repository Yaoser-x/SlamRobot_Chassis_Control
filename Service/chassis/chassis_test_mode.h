#ifndef CHASSIS_TEST_MODE_H
#define CHASSIS_TEST_MODE_H

#include <stdint.h>

#include "chassis_service.h"
#include "chassis_speed_loop.h"
#include "motor_types.h"

typedef struct
{
    uint8_t  open_loop_enabled;
    uint8_t  raw_input_enabled;
    int16_t  open_loop_side[2];
    int16_t  raw_forward[MOTOR_ID_COUNT];
    int16_t  raw_reverse[MOTOR_ID_COUNT];
    uint32_t last_refresh_ms;
    uint8_t  lease_active;
} chassis_test_mode_t;

typedef struct
{
    uint8_t open_loop_active;
    uint8_t raw_input_active;
    uint8_t expired;
    int16_t open_loop_side[2];
    int16_t raw_forward[MOTOR_ID_COUNT];
    int16_t raw_reverse[MOTOR_ID_COUNT];
} chassis_test_mode_snapshot_t;

/** Initialize chassis maintenance test-mode state. */
void ChassisTestMode_Init(chassis_test_mode_t *mode);

/** Cancel every active chassis test mode. */
void ChassisTestMode_Cancel(chassis_test_mode_t *mode);

/** Snapshot test commands and expire a stale lease. */
void ChassisTestMode_GetSnapshot(chassis_test_mode_t *mode, uint32_t now_ms, chassis_test_mode_snapshot_t *snapshot);

/** Set or refresh side open-loop test output. */
void ChassisTestMode_SetOpenLoop(chassis_test_mode_t *mode, int16_t left_permille, int16_t right_permille);

/** Set or refresh per-side raw forward/reverse test input. */
void ChassisTestMode_SetRawSides(chassis_test_mode_t *mode,
                                 int16_t              left_forward,
                                 int16_t              left_reverse,
                                 int16_t              right_forward,
                                 int16_t              right_reverse);

/** Set or refresh one raw motor test input. */
void ChassisTestMode_SetRawMotor(chassis_test_mode_t *mode,
                                 motor_id_t           motor,
                                 int16_t              forward_permille,
                                 int16_t              reverse_permille);

/** Apply one captured side open-loop test command. */
void ChassisTestMode_ApplyOpenLoop(const chassis_test_mode_snapshot_t *test,
                                   chassis_service_snapshot_t         *chassis,
                                   chassis_speed_loop_t               *speed_loop);

/** Apply one captured per-motor raw input test command. */
void ChassisTestMode_ApplyRaw(const chassis_test_mode_snapshot_t *test,
                              chassis_service_snapshot_t         *chassis,
                              chassis_speed_loop_t               *speed_loop);

#endif
