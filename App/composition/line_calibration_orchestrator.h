#ifndef LINE_CALIBRATION_ORCHESTRATOR_H
#define LINE_CALIBRATION_ORCHESTRATOR_H

#include <stdint.h>

#include "line_following_calibration_types.h"

typedef enum
{
    APP_LINE_CALIBRATION_MODE_MANUAL = 1,
    APP_LINE_CALIBRATION_MODE_AUTOMATIC
} app_line_calibration_mode_t;

typedef enum
{
    APP_LINE_CALIBRATION_RESULT_OK = 0,
    APP_LINE_CALIBRATION_RESULT_BUSY,
    APP_LINE_CALIBRATION_RESULT_MAINTENANCE_DENIED,
    APP_LINE_CALIBRATION_RESULT_REQUEST_REJECTED,
    APP_LINE_CALIBRATION_RESULT_INCOMPLETE,
    APP_LINE_CALIBRATION_RESULT_LOW_SEPARATION,
    APP_LINE_CALIBRATION_RESULT_PARAMETER_REJECTED,
    APP_LINE_CALIBRATION_RESULT_SAVE_FAILED,
    APP_LINE_CALIBRATION_RESULT_TIMEOUT,
    APP_LINE_CALIBRATION_RESULT_CANCELLED
} app_line_calibration_result_t;

typedef enum
{
    APP_LINE_CALIBRATION_LED_NONE = 0,
    APP_LINE_CALIBRATION_LED_RUNNING,
    APP_LINE_CALIBRATION_LED_SURFACE_READY,
    APP_LINE_CALIBRATION_LED_APPLIED,
    APP_LINE_CALIBRATION_LED_FAILED
} app_line_calibration_led_event_t;

/** Initialize the App-owned line calibration session state. */
void LineCalibrationOrchestrator_Init(uint32_t no_progress_timeout_ms);
/** Start or extend one owned calibration session. */
app_line_calibration_result_t LineCalibrationOrchestrator_Request(app_line_calibration_mode_t       mode,
                                                                  line_sensor_calibration_surface_t surface,
                                                                  uint16_t                          samples);
/** Apply a Debug-owned session to RAM and release its maintenance lease. */
app_line_calibration_result_t LineCalibrationOrchestrator_ApplyManual(void);
/** Explicitly cancel the active session and clear all collected data. */
void LineCalibrationOrchestrator_Cancel(void);
/** Advance completion, automatic commit, and no-progress timeout handling. */
void LineCalibrationOrchestrator_Update(uint32_t now_ms);
/** Consume at most one semantic LED event. */
uint8_t LineCalibrationOrchestrator_TakeLedEvent(app_line_calibration_led_event_t *event);
/** Read the terminal or most recent request result for diagnostics. */
app_line_calibration_result_t LineCalibrationOrchestrator_GetLastResult(void);

#endif /* LINE_CALIBRATION_ORCHESTRATOR_H */
