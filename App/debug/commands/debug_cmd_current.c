#include "debug_cmd_current.h"

#include "adc_monitor.h"
#include "adc_monitor_config.h"
#include "chassis_layout.h"
#include "debug_console_writer.h"
#include "motor_driver.h"
#include "safety_management_service.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG_CMD_CURRENT_TX_SIZE 1536U

#define CURRENT_LOG(level, fmt, ...)                                                                                   \
    do                                                                                                                 \
    {                                                                                                                  \
        char tx[DEBUG_CMD_CURRENT_TX_SIZE];                                                                            \
        (void)snprintf(tx, sizeof(tx), "[" level "] " fmt "\r\n", ##__VA_ARGS__);                                      \
        DebugConsoleWriter_Write(tx);                                                                                  \
    } while (0)

static int32_t DebugCmdCurrent_Milli(float value)
{
    return (int32_t)(value * 1000.0f);
}

static float DebugCmdCurrent_ScaleForMotor(motor_id_t motor)
{
    static const float scales[MOTOR_ID_COUNT] = {
        MOTOR_CURRENT_VOLTS_PER_AMP_M1,
        MOTOR_CURRENT_VOLTS_PER_AMP_M2,
        MOTOR_CURRENT_VOLTS_PER_AMP_M3,
        MOTOR_CURRENT_VOLTS_PER_AMP_M4,
    };

    if ((uint32_t)motor >= MOTOR_ID_COUNT)
    {
        return MOTOR_CURRENT_VOLTS_PER_AMP;
    }
    return scales[(uint32_t)motor];
}

static uint8_t DebugCmdCurrent_ParseMotor(const char *token, motor_id_t *motor)
{
    if (token == 0 || motor == 0 || token[0] != 'm' || token[1] < '1' || token[1] > '4' || token[2] != '\0')
    {
        return 0U;
    }
    *motor = (motor_id_t)(token[1] - '1');
    return 1U;
}

static uint8_t DebugCmdCurrent_AllEnabledMotorsStopped(void)
{
    motor_driver_state_t motor_state;

    MotorDriver_GetState(&motor_state);
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U && motor_state.effective_pwm[i] != 0)
        {
            return 0U;
        }
    }
    return 1U;
}

void DebugCmdCurrent_PrintCalibrationStatus(void)
{
    char                       tx[DEBUG_CMD_CURRENT_TX_SIZE];
    adc_monitor_state_t        adc_state;
    safety_management_status_t monitor_state;

    AdcMonitor_GetState(&adc_state);
    (void)SafetyManagement_GetStatus(&monitor_state);
    (void)snprintf(tx,
                   sizeof(tx),
                   "ADCCAL cal=%u/%u valid=%u cvalid=%u cmask=0x%02X invalid=0x%08lX raw_n=%u rate_mHz=%lu "
                   "observe=%lu,%lu,%lu,%lu would=%lu,%lu,%lu,%lu\r\n",
                   adc_state.current_zero_sample_count,
                   (uint16_t)ADC_MONITOR_CURRENT_ZERO_SAMPLES,
                   adc_state.current_valid,
                   adc_state.current_control_valid,
                   adc_state.current_control_valid_mask,
                   (unsigned long)adc_state.invalid_reason_flags,
                   adc_state.raw_sample_count,
                   (unsigned long)adc_state.sample_rate_hz_milli,
                   (unsigned long)monitor_state.current_observe_over_limit_count[MOTOR_ID_M1],
                   (unsigned long)monitor_state.current_observe_over_limit_count[MOTOR_ID_M2],
                   (unsigned long)monitor_state.current_observe_over_limit_count[MOTOR_ID_M3],
                   (unsigned long)monitor_state.current_observe_over_limit_count[MOTOR_ID_M4],
                   (unsigned long)monitor_state.current_fault_would_latch_count[MOTOR_ID_M1],
                   (unsigned long)monitor_state.current_fault_would_latch_count[MOTOR_ID_M2],
                   (unsigned long)monitor_state.current_fault_would_latch_count[MOTOR_ID_M3],
                   (unsigned long)monitor_state.current_fault_would_latch_count[MOTOR_ID_M4]);
    DebugConsoleWriter_Write(tx);
    (void)snprintf(tx,
                   sizeof(tx),
                   "ADCQ m1 signed=%ld noise=%ld span=%u q=0x%08lX m2 signed=%ld noise=%ld span=%u q=0x%08lX m3 "
                   "signed=%ld noise=%ld span=%u q=0x%08lX m4 signed=%ld noise=%ld span=%u q=0x%08lX\r\n",
                   (long)DebugCmdCurrent_Milli(adc_state.current_signed_mean_a[MOTOR_ID_M1]),
                   (long)DebugCmdCurrent_Milli(adc_state.current_noise_a[MOTOR_ID_M1]),
                   adc_state.current_zero_span_raw[MOTOR_ID_M1],
                   (unsigned long)adc_state.current_quality_flags[MOTOR_ID_M1],
                   (long)DebugCmdCurrent_Milli(adc_state.current_signed_mean_a[MOTOR_ID_M2]),
                   (long)DebugCmdCurrent_Milli(adc_state.current_noise_a[MOTOR_ID_M2]),
                   adc_state.current_zero_span_raw[MOTOR_ID_M2],
                   (unsigned long)adc_state.current_quality_flags[MOTOR_ID_M2],
                   (long)DebugCmdCurrent_Milli(adc_state.current_signed_mean_a[MOTOR_ID_M3]),
                   (long)DebugCmdCurrent_Milli(adc_state.current_noise_a[MOTOR_ID_M3]),
                   adc_state.current_zero_span_raw[MOTOR_ID_M3],
                   (unsigned long)adc_state.current_quality_flags[MOTOR_ID_M3],
                   (long)DebugCmdCurrent_Milli(adc_state.current_signed_mean_a[MOTOR_ID_M4]),
                   (long)DebugCmdCurrent_Milli(adc_state.current_noise_a[MOTOR_ID_M4]),
                   adc_state.current_zero_span_raw[MOTOR_ID_M4],
                   (unsigned long)adc_state.current_quality_flags[MOTOR_ID_M4]);
    DebugConsoleWriter_Write(tx);
}

static void DebugCmdCurrent_PrintPlan(const char *motor_token, const char *known_token)
{
    motor_id_t          motor = MOTOR_ID_M1;
    int                 known_ma;
    adc_monitor_state_t adc_state;
    float               measured_a;
    float               known_a;
    float               current_scale;
    float               suggested_scale;

    if (DebugCmdCurrent_ParseMotor(motor_token, &motor) == 0U || known_token == 0)
    {
        DebugConsoleWriter_Write("usage: adccal plan m1|m2|m3|m4 known_mA\r\n");
        return;
    }
    known_ma = atoi(known_token);
    if (known_ma <= 0)
    {
        CURRENT_LOG("WARN", "adccal plan rejected: known_mA must be positive");
        return;
    }
    AdcMonitor_GetState(&adc_state);
    measured_a = adc_state.current_signed_mean_a[motor];
    if (measured_a < 0.0f)
    {
        measured_a = -measured_a;
    }
    if (measured_a < 0.001f)
    {
        measured_a = adc_state.current_mean_a[motor];
    }
    if (measured_a < 0.001f)
    {
        CURRENT_LOG("WARN", "adccal plan rejected: measured current too small");
        return;
    }
    known_a         = (float)known_ma / 1000.0f;
    current_scale   = DebugCmdCurrent_ScaleForMotor(motor);
    suggested_scale = current_scale * (measured_a / known_a);
    CURRENT_LOG("INFO",
                "adccal m%u measured=%ldmA known=%dmA current_scale_mV_per_A=%ld suggested_mV_per_A=%ld",
                (unsigned int)motor + 1U,
                (long)DebugCmdCurrent_Milli(measured_a),
                known_ma,
                (long)DebugCmdCurrent_Milli(current_scale),
                (long)DebugCmdCurrent_Milli(suggested_scale));
}

uint8_t DebugCmdCurrent_TryHandle(char *line)
{
    char *args;
    char *command;

    if (line == 0 || strncmp(line, "adccal", 6U) != 0)
    {
        return 0U;
    }
    args = line + 6;
    while (*args == ' ' || *args == '\t')
    {
        ++args;
    }
    command = strtok(args, " \t");
    if (command == 0 || strcmp(command, "show") == 0)
    {
        DebugCmdCurrent_PrintCalibrationStatus();
    }
    else if (strcmp(command, "zero") == 0)
    {
        if (DebugCmdCurrent_AllEnabledMotorsStopped() == 0U)
        {
            CURRENT_LOG("WARN", "adccal zero rejected: stop enabled motors first");
        }
        else
        {
            AdcMonitor_RequestCurrentZeroCalibration();
            CURRENT_LOG("INFO", "adc current zero calibration restarted");
        }
    }
    else if (strcmp(command, "plan") == 0)
    {
        char *motor_token = strtok(0, " \t");
        char *known_token = strtok(0, " \t");

        DebugCmdCurrent_PrintPlan(motor_token, known_token);
    }
    else
    {
        DebugConsoleWriter_Write("usage: adccal show | adccal zero | adccal plan mN known_mA\r\n");
    }
    return 1U;
}
