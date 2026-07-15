#include "debug_console_writer.h"

#include "debug_uart_transport.h"

#include <stdint.h>
#include <string.h>

#define DEBUG_CONSOLE_WRITER_TIMEOUT_MS 100U

void DebugConsoleWriter_Write(const char *text)
{
    if (text != 0)
    {
        DebugUartTransport_Write((const uint8_t *)text, (uint16_t)strlen(text), DEBUG_CONSOLE_WRITER_TIMEOUT_MS);
    }
}
