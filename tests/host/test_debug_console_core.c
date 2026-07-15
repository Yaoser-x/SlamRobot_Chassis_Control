#include "debug_console_parser.h"
#include "debug_console_registry.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t handler_calls;

static debug_cmd_result_t TestHandler(const debug_cmd_context_t *context, int argc, const char *const argv[])
{
    assert(context != 0);
    assert(context->user_data == &handler_calls);
    assert(argc == 2);
    assert(strcmp(argv[1], "arg") == 0);
    handler_calls++;
    return DEBUG_CMD_OK;
}

static void TestLineParsing(void)
{
    char                 storage[32];
    debug_console_args_t args;

    assert(DebugConsoleParser_Parse("  cmd\targ  ", storage, sizeof(storage), &args) == 1U);
    assert(args.argc == 2);
    assert(strcmp(args.argv[0], "cmd") == 0);
    assert(strcmp(args.argv[1], "arg") == 0);
    assert(DebugConsoleParser_Parse("", storage, sizeof(storage), &args) == 0U);
    assert(DebugConsoleParser_Parse("012345678901234567890123456789012", storage, sizeof(storage), &args) == 0U);
}

static void TestValueParsing(void)
{
    int32_t    signed_value;
    uint32_t   unsigned_value;
    float      float_value;
    uint8_t    bool_value;
    motor_id_t motor;

    assert(DebugConsoleParser_ParseInt32("-123", &signed_value) == 1U && signed_value == -123);
    assert(DebugConsoleParser_ParseInt32("12x", &signed_value) == 0U);
    assert(DebugConsoleParser_ParseUint32("0x20", &unsigned_value) == 1U && unsigned_value == 32U);
    assert(DebugConsoleParser_ParseUint32("-1", &unsigned_value) == 0U);
    assert(DebugConsoleParser_ParseFloat("1.25", &float_value) == 1U && fabsf(float_value - 1.25f) < 0.0001f);
    assert(DebugConsoleParser_ParseFloat("1.0x", &float_value) == 0U);
    assert(DebugConsoleParser_ParseBool("on", &bool_value) == 1U && bool_value == 1U);
    assert(DebugConsoleParser_ParseBool("false", &bool_value) == 1U && bool_value == 0U);
    assert(DebugConsoleParser_ParseMotor("m4", &motor) == 1U && motor == MOTOR_ID_M4);
    assert(DebugConsoleParser_ParseMotor("m5", &motor) == 0U);
}

static void TestRegistry(void)
{
    static const debug_command_t commands[] = {
        {"cmd", "cmd arg", "test command", 0U, TestHandler},
    };
    const debug_command_t *command;
    debug_cmd_context_t    context = {&handler_calls};
    const char            *argv[]  = {"cmd", "arg"};

    command = DebugConsoleRegistry_Find(commands, 1U, "cmd");
    assert(command != 0);
    assert(command->handler(&context, 2, argv) == DEBUG_CMD_OK);
    assert(handler_calls == 1U);
    assert(DebugConsoleRegistry_Find(commands, 1U, "missing") == 0);
    assert(DebugConsoleRegistry_Find(0, 1U, "cmd") == 0);
}

int main(void)
{
    TestLineParsing();
    TestValueParsing();
    TestRegistry();
    puts("debug console core tests passed");
    return 0;
}
