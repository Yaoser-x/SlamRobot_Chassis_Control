#include "debug_telemetry.h"
#include "debug_telemetry_csv.h"
#include "debug_telemetry_json.h"
#include "debug_telemetry_model.h"

#include "adc_monitor.h"
#include "motion_control_service.h"
#include "debug_console_writer.h"
#include "debug_log_policy.h"
#include "debug_straight_telemetry.h"
#include "encoder_driver.h"
#include "wheel_speed_estimator.h"
#include "esp12f_service.h"
#include "imu_bmi270.h"
#include "line_uart.h"
#include "motor_driver.h"
#include "parameter_management_service.h"
#include "teleoperation_service.h"
#include "safety_management_service.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG_TELEMETRY_TX_SIZE 1536U

static uint8_t            stream_mode;
static debug_log_policy_t log_policy;
static uint8_t            log_filter_count;
static uint8_t            log_filter_order[10];
static int32_t            motor_log_last_count[MOTOR_ID_COUNT];
static uint32_t           motor_log_last_ms;
static uint8_t            motor_log_baseline_valid;
static uint32_t           last_log_ms;

static int32_t DebugTelemetry_Milli(float value)
{
    return (int32_t)(value * 1000.0f);
}

/* ────────── 日志字段分组 ────────── */

typedef enum
{
    LOG_FLD_MOTOR = 0,
    LOG_FLD_ADC,
    LOG_FLD_ADC_RAW,
    LOG_FLD_IMU,
    LOG_FLD_ERRORS,
    LOG_FLD_SOURCE,
    LOG_FLD_PS2,
    LOG_FLD_LINE,
    LOG_FLD_ESP,
    LOG_FLD_COUNT
} log_field_id_t;

static const char *const log_field_names[LOG_FLD_COUNT] =
    {"motor", "adc", "adcraw", "imu", "errors", "source", "ps2", "line", "esp"};

static const char *const log_field_headers[LOG_FLD_COUNT] = {
    "m1_mms,m2_mms,m3_mms,m4_mms,m1_pwm,m2_pwm,m3_pwm,m4_pwm",
    "vbat_mv,m1_ma,m2_ma,m3_ma,m4_ma",
    "m1_mean_ma,m1_rms_ma,m1_pk_ma,m1_n,m2_mean_ma,m2_rms_ma,m2_pk_ma,m2_n,m3_mean_ma,m3_rms_ma,m3_pk_ma,m3_n,m4_mean_"
    "ma,m4_rms_ma,m4_pk_ma,m4_n",
    "imu_online,imu_chip,imu_acc_x_g,imu_acc_y_g,imu_acc_z_g,imu_gyro_corr_x_dps,imu_gyro_corr_y_dps,imu_gyro_corr_z_"
    "dps,imu_gyro_filt_x_dps,imu_gyro_filt_y_dps,imu_gyro_filt_z_dps,imu_roll_deg,imu_pitch_deg,imu_yaw_deg,imu_stime,"
    "imu_qw,imu_qx,imu_qy,imu_qz,imu_quality",
    "errors",
    "source",
    "ps2_ok,ps2_fail",
    "line_bytes,line_frames",
    "esp_rx,esp_tx"};

static void DebugTelemetry_ResetMotorBaseline(void)
{
    motor_log_baseline_valid = 0U;
    motor_log_last_ms        = 0U;
}

static void DebugTelemetry_GetMotorSpeed(uint32_t now_ms, const encoder_state_t *state, float speed_mps[MOTOR_ID_COUNT])
{
    uint32_t      dt_ms          = now_ms - motor_log_last_ms;
    float         counts_per_rev = EncoderDriver_GetCountsPerRev();
    param_model_t params;

    (void)ParameterManagement_GetSnapshot(&params);

    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (motor_log_baseline_valid != 0U)
        {
            speed_mps[i] = WheelSpeedEstimator_CountDeltaSpeedMps(state->count[i] - motor_log_last_count[i],
                                                                  dt_ms,
                                                                  counts_per_rev,
                                                                  params.wheel_radius_m);
        }
        else
        {
            speed_mps[i] = 0.0f;
        }
        motor_log_last_count[i] = state->count[i];
    }

    motor_log_last_ms        = now_ms;
    motor_log_baseline_valid = 1U;
}

static void DebugTelemetry_PrintFilteredHeader(void)
{
    char    tx[DEBUG_TELEMETRY_TX_SIZE];
    size_t  pos = 0U;
    uint8_t i;

    pos += (size_t)snprintf(tx + pos, sizeof(tx) - pos, "t_ms");
    for (i = 0U; i < log_filter_count; ++i)
    {
        pos += (size_t)snprintf(tx + pos, sizeof(tx) - pos, ",%s", log_field_headers[log_filter_order[i]]);
    }
    pos += (size_t)snprintf(tx + pos, sizeof(tx) - pos, "\r\n");
    DebugConsoleWriter_Write(tx);
}

static size_t DebugTelemetry_WriteFieldData(char *tx, size_t pos, log_field_id_t field, uint32_t now_ms)
{
    adc_monitor_state_t        adc;
    encoder_state_t            enc;
    motion_control_status_t    cs;
    motor_driver_state_t       motor_state;
    safety_management_status_t mon;
    imu_bmi270_state_t         imu;
    teleoperation_status_t     ps2;
    line_uart_state_t          line;
    esp12f_service_state_t     esp;
    float                      motor_log_speed_mps[MOTOR_ID_COUNT];

    /* 惰性获取：仅需要的字段才获取状态快照 */
    switch (field)
    {
        case LOG_FLD_MOTOR:
            EncoderDriver_GetState(&enc);
            (void)MotionControl_GetStatus(&cs);
            MotorDriver_GetState(&motor_state);
            DebugTelemetry_GetMotorSpeed(now_ms, &enc, motor_log_speed_mps);
            pos += (size_t)snprintf(tx + pos,
                                    DEBUG_TELEMETRY_TX_SIZE - pos,
                                    "%ld,%ld,%ld,%ld,%d,%d,%d,%d",
                                    (long)DebugTelemetry_Milli(motor_log_speed_mps[MOTOR_ID_M1]),
                                    (long)DebugTelemetry_Milli(motor_log_speed_mps[MOTOR_ID_M2]),
                                    (long)DebugTelemetry_Milli(motor_log_speed_mps[MOTOR_ID_M3]),
                                    (long)DebugTelemetry_Milli(motor_log_speed_mps[MOTOR_ID_M4]),
                                    motor_state.effective_pwm[MOTOR_ID_M1],
                                    motor_state.effective_pwm[MOTOR_ID_M2],
                                    motor_state.effective_pwm[MOTOR_ID_M3],
                                    motor_state.effective_pwm[MOTOR_ID_M4]);
            break;

        case LOG_FLD_ADC:
            AdcMonitor_GetState(&adc);
            pos += (size_t)snprintf(tx + pos,
                                    DEBUG_TELEMETRY_TX_SIZE - pos,
                                    "%ld,%ld,%ld,%ld,%ld",
                                    (long)DebugTelemetry_Milli(adc.battery_voltage),
                                    (long)DebugTelemetry_Milli(adc.current_a[MOTOR_ID_M1]),
                                    (long)DebugTelemetry_Milli(adc.current_a[MOTOR_ID_M2]),
                                    (long)DebugTelemetry_Milli(adc.current_a[MOTOR_ID_M3]),
                                    (long)DebugTelemetry_Milli(adc.current_a[MOTOR_ID_M4]));
            break;

        case LOG_FLD_ADC_RAW:
            AdcMonitor_GetState(&adc);
            pos += (size_t)snprintf(tx + pos,
                                    DEBUG_TELEMETRY_TX_SIZE - pos,
                                    "%ld,%ld,%ld,%u,%ld,%ld,%ld,%u,%ld,%ld,%ld,%u,%ld,%ld,%ld,%u",
                                    (long)DebugTelemetry_Milli(adc.current_mean_a[MOTOR_ID_M1]),
                                    (long)DebugTelemetry_Milli(adc.current_rms_a[MOTOR_ID_M1]),
                                    (long)DebugTelemetry_Milli(adc.current_peak_a[MOTOR_ID_M1]),
                                    adc.current_sample_count[MOTOR_ID_M1],
                                    (long)DebugTelemetry_Milli(adc.current_mean_a[MOTOR_ID_M2]),
                                    (long)DebugTelemetry_Milli(adc.current_rms_a[MOTOR_ID_M2]),
                                    (long)DebugTelemetry_Milli(adc.current_peak_a[MOTOR_ID_M2]),
                                    adc.current_sample_count[MOTOR_ID_M2],
                                    (long)DebugTelemetry_Milli(adc.current_mean_a[MOTOR_ID_M3]),
                                    (long)DebugTelemetry_Milli(adc.current_rms_a[MOTOR_ID_M3]),
                                    (long)DebugTelemetry_Milli(adc.current_peak_a[MOTOR_ID_M3]),
                                    adc.current_sample_count[MOTOR_ID_M3],
                                    (long)DebugTelemetry_Milli(adc.current_mean_a[MOTOR_ID_M4]),
                                    (long)DebugTelemetry_Milli(adc.current_rms_a[MOTOR_ID_M4]),
                                    (long)DebugTelemetry_Milli(adc.current_peak_a[MOTOR_ID_M4]),
                                    adc.current_sample_count[MOTOR_ID_M4]);
            break;

        case LOG_FLD_IMU:
            ImuBmi270_GetState(&imu);
            pos += (size_t)snprintf(
                tx + pos,
                DEBUG_TELEMETRY_TX_SIZE - pos,
                "%u,%u,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu,%.4f,%.4f,%.4f,%.4f,%lu",
                imu.online,
                imu.chip_id,
                imu.body_accel_g[0],
                imu.body_accel_g[1],
                imu.body_accel_g[2],
                imu.body_gyro_dps[0],
                imu.body_gyro_dps[1],
                imu.body_gyro_dps[2],
                imu.gyro_filtered_dps[0],
                imu.gyro_filtered_dps[1],
                imu.gyro_filtered_dps[2],
                imu.roll_deg,
                imu.pitch_deg,
                imu.yaw_deg,
                (unsigned long)imu.sensor_time,
                imu.quaternion[0],
                imu.quaternion[1],
                imu.quaternion[2],
                imu.quaternion[3],
                (unsigned long)imu.quality_flags);
            break;

        case LOG_FLD_ERRORS:
            (void)SafetyManagement_GetStatus(&mon);
            pos += (size_t)snprintf(tx + pos, DEBUG_TELEMETRY_TX_SIZE - pos, "%lu", (unsigned long)mon.error_flags);
            break;

        case LOG_FLD_SOURCE:
            (void)SafetyManagement_GetStatus(&mon);
            pos += (size_t)snprintf(tx + pos, DEBUG_TELEMETRY_TX_SIZE - pos, "%u", mon.control_mode);
            break;

        case LOG_FLD_PS2:
            (void)Teleoperation_GetStatus(&ps2);
            pos += (size_t)snprintf(tx + pos,
                                    DEBUG_TELEMETRY_TX_SIZE - pos,
                                    "%lu,%lu",
                                    (unsigned long)ps2.rx_ok_count,
                                    (unsigned long)ps2.rx_fail_count);
            break;

        case LOG_FLD_LINE:
            LineUart_GetState(&line);
            pos += (size_t)snprintf(tx + pos,
                                    DEBUG_TELEMETRY_TX_SIZE - pos,
                                    "%lu,%lu",
                                    (unsigned long)line.rx_bytes,
                                    (unsigned long)line.rx_frames);
            break;

        case LOG_FLD_ESP:
            Esp12fService_GetState(&esp);
            pos += (size_t)snprintf(tx + pos,
                                    DEBUG_TELEMETRY_TX_SIZE - pos,
                                    "%lu,%lu",
                                    (unsigned long)esp.rx_frames,
                                    (unsigned long)esp.tx_frames);
            break;

        default:
            break;
    }
    return pos;
}

static void DebugTelemetry_PrintFilteredFrame(uint32_t now_ms)
{
    char    tx[DEBUG_TELEMETRY_TX_SIZE];
    size_t  pos = 0U;
    uint8_t i;

    pos += (size_t)snprintf(tx + pos, sizeof(tx) - pos, "%lu", (unsigned long)now_ms);
    for (i = 0U; i < log_filter_count; ++i)
    {
        tx[pos++] = ',';
        pos       = DebugTelemetry_WriteFieldData(tx, pos, (log_field_id_t)log_filter_order[i], now_ms);
    }
    pos += (size_t)snprintf(tx + pos, sizeof(tx) - pos, "\r\n");
    DebugConsoleWriter_Write(tx);
}

void DebugTelemetry_PrintHeader(void)
{
    char   tx[DEBUG_TELEMETRY_TX_SIZE];
    size_t pos;

    pos = (size_t)snprintf(
        tx,
        sizeof(tx),
        "t_ms,m1_mms,m2_mms,m3_mms,m4_mms,m1_pwm,m2_pwm,m3_pwm,m4_pwm,vbat_mv,m1_ma,m2_ma,m3_ma,m4_ma,imu_online,imu_"
        "chip,errors,source,ps2_ok,ps2_fail,line_bytes,line_frames,esp_rx,esp_tx,imu_acc_x_g,imu_acc_y_g,imu_acc_z_g,"
        "imu_gyro_corr_x_dps,imu_gyro_corr_y_dps,imu_gyro_corr_z_dps,imu_gyro_filt_x_dps,imu_gyro_filt_y_dps,imu_gyro_"
        "filt_z_dps,imu_roll_deg,imu_pitch_deg,imu_yaw_deg,imu_stime,imu_qw,imu_qx,imu_qy,imu_qz,imu_quality,m1_pid_"
        "err,m2_pid_err,m3_pid_err,m4_pid_err,m1_pid_out,m2_pid_out,m3_pid_out,m4_pid_out,imu_temp_c,imu_cal_state,");
    pos += DebugStraightTelemetry_FormatCsvHeader(tx + pos, sizeof(tx) - pos);
    (void)snprintf(tx + pos, sizeof(tx) - pos, "\r\n");
    DebugConsoleWriter_Write(tx);
}

static void DebugTelemetry_CaptureFullSnapshot(uint32_t now_ms, debug_full_log_snapshot_t *snapshot)
{
    AdcMonitor_GetState(&snapshot->adc);
    EncoderDriver_GetState(&snapshot->encoder);
    DebugTelemetry_GetMotorSpeed(now_ms, &snapshot->encoder, snapshot->motor_log_speed_mps);
    (void)MotionControl_GetStatus(&snapshot->chassis);
    MotorDriver_GetState(&snapshot->motor);
    (void)SafetyManagement_GetStatus(&snapshot->monitor);
    ImuBmi270_GetState(&snapshot->imu);
    (void)Teleoperation_GetStatus(&snapshot->ps2);
    LineUart_GetState(&snapshot->line);
    Esp12fService_GetState(&snapshot->esp);
}

static void DebugTelemetry_PrintFullFrame(uint32_t now_ms)
{
    debug_full_log_snapshot_t snapshot;

    DebugTelemetry_CaptureFullSnapshot(now_ms, &snapshot);
    if (log_policy.format == DEBUG_LOG_FORMAT_JSON)
    {
        DebugTelemetryJson_Print(now_ms, &snapshot);
    }
    else
    {
        DebugTelemetryCsv_Print(now_ms, &snapshot);
    }
}

void DebugTelemetry_Init(void)
{
    stream_mode      = 0U;
    log_filter_count = 0U;
    last_log_ms      = 0U;
    DebugLogPolicy_Init(&log_policy);
    DebugTelemetry_ResetMotorBaseline();
}

void DebugTelemetry_Stop(void)
{
    stream_mode      = 0U;
    log_filter_count = 0U;
    DebugTelemetry_ResetMotorBaseline();
}

uint8_t DebugTelemetry_TryHandle(char *line)
{
    if (line == 0)
    {
        return 0U;
    }
    if (strcmp(line, "header") == 0)
    {
        DebugTelemetry_PrintHeader();
    }
    else if (strncmp(line, "log rate ", 9) == 0)
    {
        unsigned long period;
        char          tx[96];

        if (sscanf(line, "log rate %lu", &period) == 1 && period >= 50UL && period <= 5000UL)
        {
            (void)DebugLogPolicy_SetPeriod(&log_policy, (uint32_t)period);
            (void)snprintf(tx, sizeof(tx), "[INFO] log rate=%lums\r\n", period);
        }
        else
        {
            (void)snprintf(tx, sizeof(tx), "[ERR] log rate range is 50..5000ms\r\n");
        }
        DebugConsoleWriter_Write(tx);
    }
    else if (strcmp(line, "log csv") == 0 || strcmp(line, "log json") == 0)
    {
        char tx[96];

        DebugLogPolicy_SetFormat(&log_policy,
                                 (strcmp(line, "log json") == 0) ? DEBUG_LOG_FORMAT_JSON : DEBUG_LOG_FORMAT_CSV);
        (void)snprintf(tx,
                       sizeof(tx),
                       "[INFO] log format=%s\r\n",
                       log_policy.format == DEBUG_LOG_FORMAT_JSON ? "json" : "csv");
        DebugConsoleWriter_Write(tx);
    }
    else if (strncmp(line, "log ", 4) == 0)
    {
        char *token = strtok(line + 4, " \t");

        if (token == 0)
        {
            DebugConsoleWriter_Write("usage: log 0 | log 1 [field...]\r\n");
        }
        else if (atoi(token) == 0)
        {
            DebugTelemetry_Stop();
            DebugConsoleWriter_Write("[INFO] log off\r\n");
        }
        else
        {
            DebugTelemetry_ResetMotorBaseline();
            token = strtok(0, " \t");
            if (token == 0)
            {
                stream_mode      = 1U;
                log_filter_count = 0U;
                if (log_policy.format == DEBUG_LOG_FORMAT_CSV)
                {
                    DebugTelemetry_PrintHeader();
                }
            }
            else if (log_policy.format == DEBUG_LOG_FORMAT_JSON)
            {
                stream_mode      = 1U;
                log_filter_count = 0U;
                DebugConsoleWriter_Write(
                    "[WARN] JSON uses the stable full schema; field filters apply to CSV only\r\n");
            }
            else
            {
                uint8_t count = 0U;
                uint8_t order[10];
                uint8_t ok = 1U;

                while (token != 0 && count < 10U)
                {
                    uint8_t j;
                    int8_t  found = -1;

                    for (j = 0U; j < LOG_FLD_COUNT; ++j)
                    {
                        if (strcmp(token, log_field_names[j]) == 0)
                        {
                            found = (int8_t)j;
                            break;
                        }
                    }
                    if (found < 0)
                    {
                        ok = 0U;
                        break;
                    }
                    order[count++] = (uint8_t)found;
                    token          = strtok(0, " \t");
                }
                if (ok == 0U || count == 0U)
                {
                    DebugConsoleWriter_Write(
                        "[ERR] unknown field, valid: motor adc adcraw imu errors source ps2 line esp\r\n");
                }
                else
                {
                    stream_mode      = 2U;
                    log_filter_count = count;
                    for (uint8_t i = 0U; i < count; ++i)
                    {
                        log_filter_order[i] = order[i];
                    }
                    DebugTelemetry_PrintFilteredHeader();
                }
            }
        }
    }
    else
    {
        return 0U;
    }
    return 1U;
}

void DebugTelemetry_Step(uint32_t now_ms)
{
    if (stream_mode == 0U || (now_ms - last_log_ms) < log_policy.period_ms)
    {
        return;
    }
    last_log_ms = now_ms;
    if (stream_mode == 2U)
    {
        DebugTelemetry_PrintFilteredFrame(now_ms);
    }
    else
    {
        DebugTelemetry_PrintFullFrame(now_ms);
    }
}
