#ifndef DEBUG_CMD_CURRENT_H
#define DEBUG_CMD_CURRENT_H

#include <stdint.h>

/** Print the current-sensor calibration and quality status. */
void DebugCmdCurrent_PrintCalibrationStatus(void);

/** Handle current-sensor calibration debug commands. */
uint8_t DebugCmdCurrent_TryHandle(char *line);

#endif
