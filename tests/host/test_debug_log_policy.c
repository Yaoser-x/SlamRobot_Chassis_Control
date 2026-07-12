#include "debug_log_policy.h"
#include <stdio.h>
#include <stdlib.h>

static void check(int ok, const char *msg)
{
    if (!ok)
    {
        fprintf(stderr, "FAIL: %s\n", msg);
        exit(1);
    }
}

int main(void)
{
    debug_log_policy_t policy;
    DebugLogPolicy_Init(&policy);
    check(policy.period_ms == 500U && policy.format == DEBUG_LOG_FORMAT_CSV, "defaults");
    check(!DebugLogPolicy_SetPeriod(&policy, 49U), "reject below minimum");
    check(DebugLogPolicy_SetPeriod(&policy, 50U), "accept minimum");
    check(DebugLogPolicy_SetPeriod(&policy, 5000U), "accept maximum");
    check(!DebugLogPolicy_SetPeriod(&policy, 5001U), "reject above maximum");
    DebugLogPolicy_SetFormat(&policy, DEBUG_LOG_FORMAT_JSON);
    check(policy.format == DEBUG_LOG_FORMAT_JSON, "json format");
    puts("PASS: debug log policy");
    return 0;
}
