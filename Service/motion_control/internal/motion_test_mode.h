#ifndef CHASSIS_TEST_MODE_H
#define CHASSIS_TEST_MODE_H

#include <stdint.h>

#include "wheel_speed_control_loop.h"
#include "motor_types.h"
#include "motion_control_config.h"
#include "motion_control_status.h"

typedef struct
{
    uint8_t  open_loop_enabled;
    uint8_t  raw_input_enabled;
    int16_t  open_loop_side[2];
    int16_t  raw_forward[MOTOR_ID_COUNT];
    int16_t  raw_reverse[MOTOR_ID_COUNT];
    uint32_t last_refresh_ms;
    uint8_t  lease_active;
    uint32_t lease_ms;
} motion_test_mode_t;

typedef struct
{
    uint8_t open_loop_active;
    uint8_t raw_input_active;
    uint8_t expired;
    int16_t open_loop_side[2];
    int16_t raw_forward[MOTOR_ID_COUNT];
    int16_t raw_reverse[MOTOR_ID_COUNT];
} motion_test_mode_snapshot_t;

/** Initialize chassis maintenance test-mode state. */
void MotionTestMode_Init(motion_test_mode_t *mode, const motion_control_config_t *config);

/** Cancel every active chassis test mode. */
void MotionTestMode_Cancel(motion_test_mode_t *mode);

/** Snapshot test commands and expire a stale lease. */
void MotionTestMode_GetSnapshot(motion_test_mode_t *mode, uint32_t now_ms, motion_test_mode_snapshot_t *snapshot);

/** Set or refresh side open-loop test output. */
void MotionTestMode_SetOpenLoop(motion_test_mode_t *mode, int16_t left_permille, int16_t right_permille);

/** Set or refresh per-side raw forward/reverse test input. */
void MotionTestMode_SetRawSides(motion_test_mode_t *mode,
                                int16_t             left_forward,
                                int16_t             left_reverse,
                                int16_t             right_forward,
                                int16_t             right_reverse);

/** Set or refresh one raw motor test input. */
void MotionTestMode_SetRawMotor(motion_test_mode_t *mode,
                                motor_id_t          motor,
                                int16_t             forward_permille,
                                int16_t             reverse_permille);

/** Apply one captured side open-loop test command. */
void MotionTestMode_ApplyOpenLoop(const motion_test_mode_snapshot_t *test,
                                  motion_control_status_t           *chassis,
                                  wheel_speed_control_loop_t        *speed_loop);

/** Apply one captured per-motor raw input test command. */
void MotionTestMode_ApplyRaw(const motion_test_mode_snapshot_t *test,
                             motion_control_status_t           *chassis,
                             wheel_speed_control_loop_t        *speed_loop);

#endif
