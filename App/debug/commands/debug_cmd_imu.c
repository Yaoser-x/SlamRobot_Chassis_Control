#include "debug_cmd_imu.h"

#include "debug_console_writer.h"
#include "bmi270_driver.h"

#include <stdio.h>
#include <string.h>

#define DEBUG_CMD_IMU_TX_SIZE 1536U

#define IMU_LOG(level, fmt, ...)                                                                                       \
    do                                                                                                                 \
    {                                                                                                                  \
        char tx[DEBUG_CMD_IMU_TX_SIZE];                                                                                \
        (void)snprintf(tx, sizeof(tx), "[" level "] " fmt "\r\n", ##__VA_ARGS__);                                      \
        DebugConsoleWriter_Write(tx);                                                                                  \
    } while (0)

static const char *DebugCmdImu_FailReason(uint8_t reason)
{
    switch (reason)
    {
        case IMU_BMI270_GYRO_CAL_FAIL_NONE:
            return "none";
        case IMU_BMI270_GYRO_CAL_FAIL_CONFIG:
            return "config";
        case IMU_BMI270_GYRO_CAL_FAIL_READ:
            return "read";
        case IMU_BMI270_GYRO_CAL_FAIL_ABS:
            return "abs";
        case IMU_BMI270_GYRO_CAL_FAIL_SPAN:
            return "span";
        case IMU_BMI270_GYRO_CAL_FAIL_MOTION:
            return "motion";
        default:
            return "unknown";
    }
}

static void DebugCmdImu_PrintDiag(void)
{
    char              tx[DEBUG_CMD_IMU_TX_SIZE];
    imu_bmi270_diag_t diag;

    if (Bmi270Driver_Diagnose(&diag) == 0U)
    {
        IMU_LOG("WARN", "bmi270 diag failed");
        return;
    }
    (void)snprintf(tx,
                   sizeof(tx),
                   "BMI270 diag hal1 st=%u rx=%02X,%02X,%02X hal2 st=%u rx=%02X,%02X,%02X\r\n",
                   diag.hal_status[0],
                   diag.hal_rx[0][0],
                   diag.hal_rx[0][1],
                   diag.hal_rx[0][2],
                   diag.hal_status[1],
                   diag.hal_rx[1][0],
                   diag.hal_rx[1][1],
                   diag.hal_rx[1][2]);
    DebugConsoleWriter_Write(tx);
    (void)snprintf(tx,
                   sizeof(tx),
                   "BMI270 diag bitbang rx=%02X,%02X,%02X miso nopull=%u pullup=%u pulldown=%u\r\n",
                   diag.bitbang_rx[0],
                   diag.bitbang_rx[1],
                   diag.bitbang_rx[2],
                   diag.miso_nopull,
                   diag.miso_pullup,
                   diag.miso_pulldown);
    DebugConsoleWriter_Write(tx);
}

static void DebugCmdImu_Calibrate(int value)
{
    uint16_t              samples = (value > 0) ? (uint16_t)value : 0U;
    bmi270_driver_state_t imu_state;

    IMU_LOG("INFO", "bmi270 gyro calibration request: keep still");
    if (Bmi270Driver_CalibrateGyro(samples, 10U) == 0U)
    {
        Bmi270Driver_GetState(&imu_state);
        IMU_LOG("WARN",
                "bmi270 gyro calibration failed reason=%s axis=%u samples=%u mean_dps=%.2f,%.2f,%.2f "
                "span_dps=%.2f,%.2f,%.2f min_dps=%.2f,%.2f,%.2f max_dps=%.2f,%.2f,%.2f init=%u err=%u "
                "chip=0x%02X online=%u",
                DebugCmdImu_FailReason(imu_state.gyro_cal_fail_reason),
                imu_state.gyro_cal_fail_axis,
                imu_state.gyro_cal_sample_count,
                imu_state.gyro_cal_mean_dps[0],
                imu_state.gyro_cal_mean_dps[1],
                imu_state.gyro_cal_mean_dps[2],
                imu_state.gyro_cal_span_dps[0],
                imu_state.gyro_cal_span_dps[1],
                imu_state.gyro_cal_span_dps[2],
                imu_state.gyro_cal_min_dps[0],
                imu_state.gyro_cal_min_dps[1],
                imu_state.gyro_cal_min_dps[2],
                imu_state.gyro_cal_max_dps[0],
                imu_state.gyro_cal_max_dps[1],
                imu_state.gyro_cal_max_dps[2],
                imu_state.init_state,
                imu_state.last_error,
                imu_state.chip_id,
                imu_state.online);
        return;
    }
    Bmi270Driver_GetState(&imu_state);
    IMU_LOG("INFO",
            "bmi270 gyro calibration accepted state=%u samples=%u; use status/acal for progress",
            imu_state.gyro_auto_cal_state,
            imu_state.gyro_cal_sample_count);
}

uint8_t DebugCmdImu_TryHandle(const char *line)
{
    int value = 0;

    if (line == 0)
    {
        return 0U;
    }
    if (strcmp(line, "imutest") == 0)
    {
        uint8_t probe_ok = Bmi270Driver_ProbeNow();
        if (probe_ok != 0U)
        {
            IMU_LOG("INFO", "bmi270 probe ok");
        }
        else
        {
            IMU_LOG("WARN", "bmi270 probe failed");
        }
        return 1U;
    }
    if (strcmp(line, "imudiag") == 0)
    {
        DebugCmdImu_PrintDiag();
        return 1U;
    }
    if (strcmp(line, "imuinit") == 0)
    {
        bmi270_driver_state_t state;
        if (Bmi270Driver_ConfigNow() != 0U)
        {
            IMU_LOG("INFO", "bmi270 init ok");
        }
        else
        {
            Bmi270Driver_GetState(&state);
            IMU_LOG("WARN",
                    "bmi270 init failed init=%u err=%u chip=0x%02X online=%u",
                    state.init_state,
                    state.last_error,
                    state.chip_id,
                    state.online);
        }
        return 1U;
    }
    if (strcmp(line, "imucal") == 0 || sscanf(line, "imucal %d", &value) == 1)
    {
        DebugCmdImu_Calibrate(value);
        return 1U;
    }
    if (strcmp(line, "imucalclear") == 0)
    {
        Bmi270Driver_ClearCalibration();
        IMU_LOG("INFO", "bmi270 gyro calibration cleared");
        return 1U;
    }
    if (sscanf(line, "imu %d", &value) == 1)
    {
        (void)Bmi270Driver_SetEnabled((value != 0) ? 1U : 0U);
        IMU_LOG("INFO", "imu %s", (value != 0) ? "enabled" : "disabled");
        return 1U;
    }
    return 0U;
}
