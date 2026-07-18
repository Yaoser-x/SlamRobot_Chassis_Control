#include "debug_cmd_line.h"

#include "motion_control_service.h"
#include "motion_control_maintenance.h"
#include "debug_console_writer.h"
#include "line_following_service.h"
#include "line_following_maintenance.h"
#include "line_sensor_driver.h"

#include <stdio.h>
#include <string.h>

#define DEBUG_CMD_LINE_TX_SIZE 1536U

#define LINE_LOG(level, fmt, ...)                                                                                      \
    do                                                                                                                 \
    {                                                                                                                  \
        char tx[DEBUG_CMD_LINE_TX_SIZE];                                                                               \
        (void)snprintf(tx, sizeof(tx), "[" level "] " fmt "\r\n", ##__VA_ARGS__);                                      \
        DebugConsoleWriter_Write(tx);                                                                                  \
    } while (0)

static void DebugCmdLine_PrintStatus(void)
{
    static char                tx[DEBUG_CMD_LINE_TX_SIZE];
    line_sensor_data_t         sensor;
    line_following_status_t    control_state;
    line_sensor_driver_state_t line_state;

    LineSensorDriver_GetSensorData(&sensor);
    (void)LineFollowing_GetStatus(&control_state);
    LineSensorDriver_GetState(&line_state);

    (void)snprintf(
        tx,
        sizeof(tx),
        "LINE enabled=%u active=%u pos=%.2f err=%.2f derr=%.2f lx=%.3f az=%.3f det=%u sat=%u lost=%u polarity=%u\r\n"
        "LINE st=%u%u%u%u%u%u%u%u an=%u,%u,%u,%u,%u,%u,%u,%u\r\n"
        "LINE rx_bytes=%lu frames=%lu proto_err=%lu ovf=%lu\r\n",
        control_state.globally_enabled,
        control_state.tracking_active,
        (double)control_state.line_position,
        (double)control_state.error,
        (double)control_state.error_derivative,
        (double)control_state.linear_x,
        (double)control_state.angular_z,
        control_state.detected_count,
        control_state.output_saturated,
        control_state.lost_reason,
        control_state.active_low,
        sensor.state[0],
        sensor.state[1],
        sensor.state[2],
        sensor.state[3],
        sensor.state[4],
        sensor.state[5],
        sensor.state[6],
        sensor.state[7],
        sensor.analog[0],
        sensor.analog[1],
        sensor.analog[2],
        sensor.analog[3],
        sensor.analog[4],
        sensor.analog[5],
        sensor.analog[6],
        sensor.analog[7],
        (unsigned long)line_state.rx_bytes,
        (unsigned long)line_state.rx_frames,
        (unsigned long)line_state.rx_protocol_errors,
        (unsigned long)line_state.overflow_count);
    DebugConsoleWriter_Write(tx);
}

static void DebugCmdLine_PrintCalibration(void)
{
    line_sensor_calibration_t calibration;

    LineFollowing_CalibrationGet(&calibration);
    LINE_LOG("INFO",
             "linecal ready=0x%02X collecting=%u surface=%u n=%u/%u fail=0x%02X",
             calibration.ready_mask,
             calibration.collecting,
             calibration.surface,
             calibration.count[calibration.surface],
             calibration.target_samples,
             calibration.fail_mask);
    for (uint8_t channel = 0U; channel < LINE_CALIBRATION_CHANNELS; ++channel)
    {
        uint16_t floor_mean =
            (calibration.count[0] != 0U) ? (uint16_t)(calibration.sum[0][channel] / calibration.count[0]) : 0U;
        uint16_t line_mean =
            (calibration.count[1] != 0U) ? (uint16_t)(calibration.sum[1][channel] / calibration.count[1]) : 0U;
        uint16_t floor_range =
            (calibration.count[0] != 0U) ? (uint16_t)(calibration.max[0][channel] - calibration.min[0][channel]) : 0U;
        uint16_t line_range =
            (calibration.count[1] != 0U) ? (uint16_t)(calibration.max[1][channel] - calibration.min[1][channel]) : 0U;
        LINE_LOG("INFO",
                 "linecal ch%u floor mean/range/n=%u/%u/%u line=%u/%u/%u",
                 channel,
                 floor_mean,
                 floor_range,
                 calibration.count[0],
                 line_mean,
                 line_range,
                 calibration.count[1]);
    }
}

static void DebugCmdLine_ApplyCalibration(void)
{
    if (MotionControl_BeginMaintenance() != MOTION_CONTROL_MAINTENANCE_OK)
    {
        LINE_LOG("WARN", "linecal apply rejected: chassis not stationary");
        return;
    }
    if (LineFollowing_ApplyCalibration() != 0U)
    {
        LINE_LOG("INFO", "linecal applied to RAM; run set save to persist");
    }
    else
    {
        LINE_LOG("WARN", "linecal apply rejected: incomplete or low separation");
    }
    MotionControl_EndMaintenance();
}

static void DebugCmdLine_HandleCalibration(const char *line)
{
    char     action[12] = {0};
    unsigned samples    = 0U;

    if (sscanf(line + 8, "%11s %u", action, &samples) < 1)
    {
        return;
    }
    if ((strcmp(action, "floor") == 0 || strcmp(action, "line") == 0) && samples >= 4U && samples <= 2000U)
    {
        line_sensor_calibration_surface_t surface =
            (strcmp(action, "floor") == 0) ? LINE_CALIBRATION_SURFACE_FLOOR : LINE_CALIBRATION_SURFACE_LINE;
        if (LineFollowing_RequestCalibration(surface, (uint16_t)samples) != 0U)
        {
            LINE_LOG("INFO", "linecal %s collecting %u samples", action, samples);
        }
    }
    else if (strcmp(action, "show") == 0)
    {
        DebugCmdLine_PrintCalibration();
    }
    else if (strcmp(action, "apply") == 0)
    {
        DebugCmdLine_ApplyCalibration();
    }
    else if (strcmp(action, "cancel") == 0)
    {
        LineFollowing_CalibrationCancel();
        LINE_LOG("INFO", "linecal cancelled");
    }
    else
    {
        LINE_LOG("WARN", "usage: linecal floor|line <4..2000> | show|apply|cancel");
    }
}

uint8_t DebugCmdLine_TryHandle(const char *line)
{
    if (line == 0)
    {
        return 0U;
    }
    if (strcmp(line, "line") == 0)
    {
        DebugCmdLine_PrintStatus();
    }
    else if (strcmp(line, "line on") == 0)
    {
        LineFollowing_Enable(1U);
        LINE_LOG("INFO", "line tracking enabled");
    }
    else if (strcmp(line, "line off") == 0)
    {
        LineFollowing_Enable(0U);
        LINE_LOG("INFO", "line tracking disabled");
    }
    else if (strncmp(line, "linecal ", 8U) == 0)
    {
        DebugCmdLine_HandleCalibration(line);
    }
    else
    {
        return 0U;
    }
    return 1U;
}
