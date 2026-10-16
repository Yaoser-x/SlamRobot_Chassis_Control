#include "debug_cmd_diagnostics.h"
#include "debug_console_parser.h"
#include "debug_console_writer.h"
#include "boot_recovery_snapshot.h"
#include "motion_trace.h"
#include <stdio.h>
#include <string.h>

static void PrintTraceStatus(void)
{
    app_motion_trace_status_t status;
    char                      tx[256];
    AppMotionTrace_GetStatus(&status);
    (void)snprintf(tx,
                   sizeof(tx),
                   "TRACE schema=1 capacity=%u count=%u frozen=%u reason=%u trigger_ms=%lu trigger_index=%u post=%u "
                   "stationary=%u\r\n",
                   APP_MOTION_TRACE_CAPACITY,
                   status.count,
                   status.frozen,
                   status.reason,
                   (unsigned long)status.trigger_time_ms,
                   status.trigger_index,
                   status.post_remaining,
                   status.stationary_samples);
    DebugConsoleWriter_Write(tx);
}

static void PrintTraceFrame(uint16_t index)
{
    app_motion_trace_frame_t frame;
    char                     tx[512];
    if (AppMotionTrace_Read(index, &frame) == 0U)
    {
        DebugConsoleWriter_Write("TRACE_READ rejected: freeze and stop first, or index out of range\r\n");
        return;
    }
    (void)snprintf(tx,
                   sizeof(tx),
                   "TRACE_FRAME schema=1 index=%u ms=%lu errors=%lu permit=%lu revoke=%lu source=%u mask=%u safety=%u "
                   "allowed=%u target_nonzero=%u moving=%u\r\n",
                   index,
                   (unsigned long)frame.time_ms,
                   (unsigned long)frame.error_flags,
                   (unsigned long)frame.permit_generation,
                   (unsigned long)frame.revoke_generation,
                   frame.source,
                   frame.enabled_mask,
                   frame.safety_state,
                   frame.motion_allowed,
                   frame.nonzero_target_mask,
                   frame.moving_speed_mask);
    DebugConsoleWriter_Write(tx);
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        (void)snprintf(tx,
                       sizeof(tx),
                       "TRACE_MOTOR index=%u motor=%u raw=%lu count=%ld delta=%ld target_mmps=%d speed_mmps=%d "
                       "requested=%d applied=%d effective=%d phase=%u ph=%d pending=%d\r\n",
                       index,
                       i + 1U,
                       (unsigned long)frame.raw_count[i],
                       (long)frame.count[i],
                       (long)frame.delta[i],
                       frame.target_mmps[i],
                       frame.speed_mmps[i],
                       frame.requested_pwm[i],
                       frame.applied_pwm[i],
                       frame.effective_pwm[i],
                       frame.phase[i],
                       frame.ph_dir[i],
                       frame.pending_dir[i]);
        DebugConsoleWriter_Write(tx);
    }
}

static uint8_t HandleBoot(const debug_console_args_t *args)
{
    app_boot_recovery_snapshot_t boot;
    char                         tx[320];
    const uint8_t               *bytes;
    size_t                       size;
    uint32_t                     offset;
    AppBootRecoverySnapshot_Get(&boot);
    if (args->argc == 1)
    {
        (void)snprintf(tx,
                       sizeof(tx),
                       "BOOTCAL schema=1 captured=%u loaded=%u seq=%lu seq_valid=%u persisted_crc=%08lX "
                       "effective_crc=%08lX applied=%u read_ok=%u param_bytes=%u imu_bytes=%u\r\n",
                       boot.captured,
                       boot.flash_loaded,
                       (unsigned long)boot.sequence,
                       boot.sequence_valid,
                       (unsigned long)boot.persisted_crc32,
                       (unsigned long)boot.effective_crc32,
                       boot.calibration_applied,
                       boot.estimator_read_ok,
                       (unsigned)sizeof(boot.parameters),
                       (unsigned)sizeof(boot.loaded_calibration));
        DebugConsoleWriter_Write(tx);
        return 1U;
    }
    if (args->argc != 3 || DebugConsoleParser_ParseUint32(args->argv[2], &offset) == 0U)
        return 0U;
    if (strcmp(args->argv[1], "loaded") == 0)
    {
        bytes = (const uint8_t *)&boot.loaded_calibration;
        size  = sizeof(boot.loaded_calibration);
    }
    else if (strcmp(args->argv[1], "applied") == 0)
    {
        bytes = (const uint8_t *)&boot.applied_calibration;
        size  = sizeof(boot.applied_calibration);
    }
    else if (strcmp(args->argv[1], "params") == 0)
    {
        bytes = (const uint8_t *)&boot.parameters;
        size  = sizeof(boot.parameters);
    }
    else
        return 0U;
    if (boot.captured == 0U || offset >= size || offset % 32U != 0U)
        return 0U;
    size_t length = size - offset;
    if (length > 32U)
        length = 32U;
    int used = snprintf(tx,
                        sizeof(tx),
                        "BOOTCAL_DATA section=%s offset=%lu length=%u hex=",
                        args->argv[1],
                        (unsigned long)offset,
                        (unsigned)length);
    for (size_t i = 0U; i < length; ++i)
        used += snprintf(tx + used, sizeof(tx) - (size_t)used, "%02X", bytes[offset + i]);
    (void)snprintf(tx + used, sizeof(tx) - (size_t)used, "\r\n");
    DebugConsoleWriter_Write(tx);
    return 1U;
}

uint8_t DebugCmdDiagnostics_TryHandle(const char *line)
{
    char                 storage[96];
    debug_console_args_t args;
    uint32_t             index;
    if (line == 0 || DebugConsoleParser_Parse(line, storage, sizeof(storage), &args) == 0U)
        return 0U;
    if (strcmp(args.argv[0], "bootcal") == 0)
        return HandleBoot(&args);
    if (strcmp(args.argv[0], "trace") != 0)
        return 0U;
    if (args.argc == 1 || (args.argc == 2 && strcmp(args.argv[1], "status") == 0))
        PrintTraceStatus();
    else if (args.argc == 2 && strcmp(args.argv[1], "freeze") == 0)
    {
        AppMotionTrace_Freeze();
        PrintTraceStatus();
    }
    else if (args.argc == 2 && strcmp(args.argv[1], "arm") == 0)
        DebugConsoleWriter_Write(AppMotionTrace_Arm() ? "TRACE armed\r\n" : "TRACE_ARM rejected: stop first\r\n");
    else if (args.argc == 3 && strcmp(args.argv[1], "read") == 0
             && DebugConsoleParser_ParseUint32(args.argv[2], &index) != 0U && index < APP_MOTION_TRACE_CAPACITY)
        PrintTraceFrame((uint16_t)index);
    else
        return 0U;
    return 1U;
}
