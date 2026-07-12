#include "esp_link_policy.h"
#include <stdio.h>
#include <stdlib.h>

static void check(int ok, const char *msg) { if (!ok) { fprintf(stderr, "FAIL: %s\n", msg); exit(1); } }

int main(void)
{
  esp_link_policy_t policy = {0};
  EspLinkPolicy_OnStatus(&policy, 1000U);
  check(EspLinkPolicy_Update(&policy, 1499U) == 0U && policy.online, "499ms online");
  check(EspLinkPolicy_Update(&policy, 1500U) == 0U && policy.online, "500ms online");
  check(EspLinkPolicy_Update(&policy, 1501U) == 1U && !policy.online, "501ms neutral and offline");
  check(EspLinkPolicy_Update(&policy, 1600U) == 0U, "neutral emitted once");
  EspLinkPolicy_OnStatus(&policy, UINT32_MAX - 10U);
  check(EspLinkPolicy_Update(&policy, 20U) == 0U && policy.online, "millis wrap online");
  EspLinkPolicy_OnStatus(&policy, 2000U);
  check(policy.online && !policy.neutral_sent, "valid status recovers telemetry only");
  puts("PASS: ESP link policy");
  return 0;
}
