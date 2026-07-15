#ifndef DEBUG_CONSOLE_REGISTRY_H
#define DEBUG_CONSOLE_REGISTRY_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        DEBUG_CMD_OK = 0,
        DEBUG_CMD_INVALID_ARGUMENT,
        DEBUG_CMD_NOT_ALLOWED,
        DEBUG_CMD_BUSY,
        DEBUG_CMD_FAILED,
        DEBUG_CMD_NOT_SUPPORTED
    } debug_cmd_result_t;

    typedef struct
    {
        void *user_data;
    } debug_cmd_context_t;

    typedef debug_cmd_result_t (*debug_cmd_handler_t)(const debug_cmd_context_t *context,
                                                      int                        argc,
                                                      const char *const          argv[]);

    typedef struct
    {
        const char         *name;
        const char         *usage;
        const char         *summary;
        uint8_t             maintenance_required;
        debug_cmd_handler_t handler;
    } debug_command_t;

    const debug_command_t *
    DebugConsoleRegistry_Find(const debug_command_t *commands, size_t command_count, const char *name);

#ifdef __cplusplus
}
#endif

#endif
