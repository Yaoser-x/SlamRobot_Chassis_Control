#include "motion_trace.h"
#include "boot_recovery_snapshot.h"
#include "debug_cmd_diagnostics.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static char output[8192];
void        DebugConsoleWriter_Write(const char *text)
{
    assert(strlen(output) + strlen(text) < sizeof(output));
    strcat(output, text);
}

static void TestReverseFreezeAndExport(void)
{
    app_motion_trace_frame_t  frame = {.enabled_mask = 6U};
    app_motion_trace_frame_t  saved;
    app_motion_trace_status_t status;
    AppMotionTrace_Init();
    assert(AppMotionTrace_Arm() == 0U);
    for (unsigned i = 0U; i < 110U; ++i)
    {
        frame.time_ms          = i * 10U;
        frame.count[1]         = (int32_t)i;
        frame.raw_count[1]     = i;
        frame.delta[1]         = 1;
        frame.target_mmps[1]   = 50;
        frame.effective_pwm[1] = -100; /* Board PH polarity is not wheel direction. */
        AppMotionTrace_Record(&frame);
    }
    frame.time_ms = 1100U;
    frame.count[1] -= 5;
    frame.delta[1] = -5;
    AppMotionTrace_Record(&frame);
    for (unsigned i = 0U; i < APP_MOTION_TRACE_POST_SAMPLES; ++i)
    {
        frame.time_ms += 10U;
        frame.delta[1] = 0;
        AppMotionTrace_Record(&frame);
    }
    AppMotionTrace_GetStatus(&status);
    assert(status.frozen != 0U && status.reason == APP_MOTION_TRACE_REVERSE);
    assert(status.count == APP_MOTION_TRACE_CAPACITY && status.trigger_index == 63U);
    assert(AppMotionTrace_Read(status.trigger_index, &saved) == 0U); /* Still commanded to move. */
    frame.target_mmps[1]      = 0;
    frame.effective_pwm[1]    = 0;
    frame.nonzero_target_mask = 2U; /* Sub-mm/s targets must not disappear through integer export. */
    for (unsigned i = 0; i < 3U; ++i)
        AppMotionTrace_Record(&frame);
    assert(AppMotionTrace_Read(63U, &saved) == 0U);
    frame.nonzero_target_mask = 0U;
    frame.moving_speed_mask   = 2U; /* 5.9mm/s rounds to 5, but still fails the exact stop bound. */
    for (unsigned i = 0; i < 3U; ++i)
        AppMotionTrace_Record(&frame);
    assert(AppMotionTrace_Read(63U, &saved) == 0U);
    frame.moving_speed_mask = 0U;
    for (unsigned i = 0; i < 3U; ++i)
        AppMotionTrace_Record(&frame);
    assert(AppMotionTrace_Read(63U, &saved) != 0U && saved.delta[1] == -5 && saved.time_ms == 1100U);
    assert(saved.effective_pwm[1] == -100 && saved.target_mmps[1] == 50);
    assert(AppMotionTrace_Read(96U, &saved) == 0U);
    output[0] = 0;
    assert(DebugCmdDiagnostics_TryHandle("trace read 63") != 0U);
    assert(strstr(output, "delta=-5") != 0 && strstr(output, "effective=-100") != 0);
    assert(AppMotionTrace_Arm() != 0U);
    AppMotionTrace_GetStatus(&status);
    assert(status.count == 0U && status.reason == 0U && status.frozen == 0U);
}

static void TestDirectionChangeAndManualCapture(void)
{
    app_motion_trace_frame_t  frame = {.enabled_mask = 6U};
    app_motion_trace_status_t status;
    AppMotionTrace_Init();
    frame.target_mmps[1] = -50;
    AppMotionTrace_Record(&frame);
    frame.target_mmps[1] = 50;
    frame.delta[1]       = -1;
    AppMotionTrace_Record(&frame);
    AppMotionTrace_GetStatus(&status);
    assert(status.reason == 0U); /* Transition itself is not evidence of an unexpected reversal. */
    frame.target_mmps[1] = 0;
    frame.delta[1]       = 0;
    for (unsigned i = 0; i < 3U; ++i)
        AppMotionTrace_Record(&frame);
    AppMotionTrace_Freeze();
    AppMotionTrace_GetStatus(&status);
    assert(status.reason == APP_MOTION_TRACE_MANUAL && status.frozen != 0U);
    assert(status.trigger_index == status.count - 1U);
}

static void TestImmutableFullBootSnapshot(void)
{
    parameter_management_status_t parameters = {0};
    imu_calibration_t             applied;
    app_boot_recovery_snapshot_t  boot;
    memset(&parameters.imu_calibration, 0xA5, sizeof(parameters.imu_calibration));
    memcpy(&applied, &parameters.imu_calibration, sizeof(applied));
    parameters.loaded_sequence           = 37U;
    parameters.loaded_sequence_valid     = 1U;
    parameters.flash_loaded              = 1U;
    parameters.persisted_parameter_crc32 = 0x0D663E2AUL;
    AppBootRecoverySnapshot_Capture(&parameters, &applied, 1U, 1U);
    memset(&parameters.imu_calibration, 0, sizeof(parameters.imu_calibration));
    parameters.loaded_sequence++;
    AppBootRecoverySnapshot_Capture(&parameters, &applied, 0U, 0U);
    AppBootRecoverySnapshot_Get(&boot);
    assert(boot.sequence == 37U && boot.calibration_applied == 1U);
    assert(memcmp(&boot.loaded_calibration, &applied, sizeof(applied)) == 0);
    assert(memcmp(&boot.applied_calibration, &applied, sizeof(applied)) == 0);
    output[0] = 0;
    assert(DebugCmdDiagnostics_TryHandle("bootcal") != 0U);
    assert(strstr(output, "seq=37") != 0 && strstr(output, "imu_bytes=96") != 0);
    output[0] = 0;
    assert(DebugCmdDiagnostics_TryHandle("bootcal loaded 64") != 0U);
    assert(strstr(output, "length=32 hex=A5A5A5") != 0);
    assert(DebugCmdDiagnostics_TryHandle("bootcal loaded 96") == 0U);
    assert(DebugCmdDiagnostics_TryHandle("bootcal loaded -1") == 0U);
    assert(DebugCmdDiagnostics_TryHandle("bootcal loaded 1") == 0U);
}

int main(void)
{
    TestReverseFreezeAndExport();
    TestDirectionChangeAndManualCapture();
    TestImmutableFullBootSnapshot();
    puts("PASS: frozen motion evidence and immutable boot recovery snapshot");
    return 0;
}
