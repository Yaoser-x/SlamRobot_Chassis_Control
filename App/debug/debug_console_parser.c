#include "debug_console_parser.h"

#include <errno.h>
#include <stdlib.h>
#include <string.h>

uint8_t DebugConsoleParser_Parse(const char *line, char *storage, size_t storage_size, debug_console_args_t *args)
{
    char *cursor;

    if (line == 0 || storage == 0 || storage_size == 0U || args == 0)
    {
        return 0U;
    }
    if (strlen(line) >= storage_size)
    {
        return 0U;
    }
    (void)strcpy(storage, line);
    args->argc = 0;
    cursor     = storage;
    while (*cursor != '\0')
    {
        while (*cursor == ' ' || *cursor == '\t')
        {
            cursor++;
        }
        if (*cursor == '\0')
        {
            break;
        }
        if (args->argc >= DEBUG_CONSOLE_PARSER_MAX_ARGS)
        {
            args->argc = 0;
            return 0U;
        }
        args->argv[args->argc++] = cursor;
        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t')
        {
            cursor++;
        }
        if (*cursor != '\0')
        {
            *cursor++ = '\0';
        }
    }
    return (args->argc > 0) ? 1U : 0U;
}

uint8_t DebugConsoleParser_ParseInt32(const char *text, int32_t *value)
{
    char *end;
    long  parsed;

    if (text == 0 || value == 0)
    {
        return 0U;
    }
    errno  = 0;
    parsed = strtol(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed < INT32_MIN || parsed > INT32_MAX)
    {
        return 0U;
    }
    *value = (int32_t)parsed;
    return 1U;
}

uint8_t DebugConsoleParser_ParseUint32(const char *text, uint32_t *value)
{
    char         *end;
    unsigned long parsed;

    if (text == 0 || value == 0 || text[0] == '-')
    {
        return 0U;
    }
    errno  = 0;
    parsed = strtoul(text, &end, 0);
    if (errno != 0 || end == text || *end != '\0' || parsed > UINT32_MAX)
    {
        return 0U;
    }
    *value = (uint32_t)parsed;
    return 1U;
}

uint8_t DebugConsoleParser_ParseFloat(const char *text, float *value)
{
    char *end;
    float parsed;

    if (text == 0 || value == 0)
    {
        return 0U;
    }
    errno  = 0;
    parsed = strtof(text, &end);
    if (errno != 0 || end == text || *end != '\0')
    {
        return 0U;
    }
    *value = parsed;
    return 1U;
}

uint8_t DebugConsoleParser_ParseBool(const char *text, uint8_t *value)
{
    if (text == 0 || value == 0)
    {
        return 0U;
    }
    if (strcmp(text, "1") == 0 || strcmp(text, "on") == 0 || strcmp(text, "true") == 0)
    {
        *value = 1U;
        return 1U;
    }
    if (strcmp(text, "0") == 0 || strcmp(text, "off") == 0 || strcmp(text, "false") == 0)
    {
        *value = 0U;
        return 1U;
    }
    return 0U;
}

uint8_t DebugConsoleParser_ParseMotor(const char *text, motor_id_t *motor)
{
    if (text == 0 || motor == 0 || text[0] != 'm' || text[2] != '\0')
    {
        return 0U;
    }
    if (text[1] < '1' || text[1] > '4')
    {
        return 0U;
    }
    *motor = (motor_id_t)(text[1] - '1');
    return 1U;
}
