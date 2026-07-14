#include "app_tasks.h"

#include "usart1_debug_console.h"

void Task_Debug(void *argument)
{
    DebugConsole_RunTask(argument);
}
