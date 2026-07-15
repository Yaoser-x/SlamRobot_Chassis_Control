#ifndef DEBUG_CMD_LINE_H
#define DEBUG_CMD_LINE_H

#include <stdint.h>

/** Handle line-sensor and line-calibration debug commands. */
uint8_t DebugCmdLine_TryHandle(const char *line);

#endif
