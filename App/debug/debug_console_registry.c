#include "debug_console_registry.h"

#include <string.h>

const debug_command_t *
DebugConsoleRegistry_Find(const debug_command_t *commands, size_t command_count, const char *name)
{
    if (commands == 0 || name == 0)
    {
        return 0;
    }
    for (size_t index = 0U; index < command_count; ++index)
    {
        if (commands[index].name != 0 && strcmp(commands[index].name, name) == 0)
        {
            return &commands[index];
        }
    }
    return 0;
}
