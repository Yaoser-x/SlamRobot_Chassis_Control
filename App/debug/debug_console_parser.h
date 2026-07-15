#ifndef DEBUG_CONSOLE_PARSER_H
#define DEBUG_CONSOLE_PARSER_H

#include <stddef.h>
#include <stdint.h>

#include "motor_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define DEBUG_CONSOLE_PARSER_MAX_ARGS 12

    typedef struct
    {
        int         argc;
        const char *argv[DEBUG_CONSOLE_PARSER_MAX_ARGS];
    } debug_console_args_t;

    uint8_t DebugConsoleParser_Parse(const char *line, char *storage, size_t storage_size, debug_console_args_t *args);
    uint8_t DebugConsoleParser_ParseInt32(const char *text, int32_t *value);
    uint8_t DebugConsoleParser_ParseUint32(const char *text, uint32_t *value);
    uint8_t DebugConsoleParser_ParseFloat(const char *text, float *value);
    uint8_t DebugConsoleParser_ParseBool(const char *text, uint8_t *value);
    uint8_t DebugConsoleParser_ParseMotor(const char *text, motor_id_t *motor);

#ifdef __cplusplus
}
#endif

#endif
