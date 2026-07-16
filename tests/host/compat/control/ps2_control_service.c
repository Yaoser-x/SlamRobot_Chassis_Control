#include "ps2_control_service.h"

#include "control_config.h"
#include "teleoperation_service.h"

void Ps2ControlService_Init(void)
{
    const teleoperation_config_t config = {
        .linear_max_mps             = PS2_LINEAR_MAX_MPS,
        .angular_max_rps            = PS2_ANGULAR_MAX_RPS,
        .dpad_linear_mps            = PS2_DPAD_LINEAR_MPS,
        .dpad_angular_rps           = PS2_DPAD_ANGULAR_RPS,
        .manual_cancel_threshold    = PS2_MANUAL_CANCEL_THRESHOLD,
        .heading_quarter_timeout_ms = PS2_HEADING_QUARTER_TIMEOUT_MS,
        .heading_full_timeout_ms    = PS2_HEADING_FULL_TIMEOUT_MS,
        .heading_imu_fresh_ms       = PS2_HEADING_IMU_FRESH_MS,
        .idle_release_ms            = PS2_IDLE_RELEASE_MS,
        .axis_center                = PS2_AXIS_CENTER,
        .axis_deadzone              = PS2_AXIS_DEADZONE,
        .offline_fail_limit         = PS2_OFFLINE_FAIL_LIMIT,
        .macro_l1_mask              = PS2_MACRO_L1_MASK,
        .macro_r1_mask              = PS2_MACRO_R1_MASK,
        .macro_l2_mask              = PS2_MACRO_L2_MASK,
        .macro_r2_mask              = PS2_MACRO_R2_MASK,
        .line_toggle_mask           = PS2_LINE_TOGGLE_MASK,
        .linecal_floor_mask         = PS2_LINECAL_FLOOR_MASK,
        .linecal_line_mask          = PS2_LINECAL_LINE_MASK,
    };

    (void)Teleoperation_Init(&config);
}

void Ps2ControlService_Update(void)
{
    Teleoperation_Update();
}

void Ps2ControlService_GetState(ps2_control_service_state_t *state)
{
    (void)Teleoperation_GetStatus(state);
}
