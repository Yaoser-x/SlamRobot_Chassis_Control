#ifndef DEBUG_CMD_ESP12F_H
#define DEBUG_CMD_ESP12F_H

#include <stdint.h>

/** Handle ESP12F maintenance and bridge debug commands. */
uint8_t DebugCmdEsp12f_TryHandle(const char *line, uint8_t *debug_velocity_enabled);

#endif
