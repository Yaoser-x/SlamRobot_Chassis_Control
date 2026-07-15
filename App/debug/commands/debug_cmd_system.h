#ifndef DEBUG_CMD_SYSTEM_H
#define DEBUG_CMD_SYSTEM_H

#include <stdint.h>

/** Handle registered system, status, identity, RTOS, and I2C debug commands. */
uint8_t DebugCmdSystem_TryHandle(const char *line);

#endif
