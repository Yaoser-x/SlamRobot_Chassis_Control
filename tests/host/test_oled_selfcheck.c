#include "oled_selfcheck.h"

#include <stdio.h>

static void require_int(int condition, const char *message)
{
  if (!condition)
  {
    (void)printf("FAIL: %s\n", message);
    __builtin_exit(1);
  }
}

static void test_rpi_line_and_esp_link_statuses(void)
{
  require_int(OLED_SelfCheckRpi(1200U, 1000U, 500U) == OLED_SELFCHECK_OK,
              "recent RPI frame passes");
  require_int(OLED_SelfCheckRpi(2000U, 1000U, 500U) == OLED_SELFCHECK_FAIL,
              "stale RPI frame fails");
  require_int(OLED_SelfCheckLine(1200U, 1U, 1180U, 50U) == OLED_SELFCHECK_OK,
              "valid recent line frame passes");
  require_int(OLED_SelfCheckLine(1200U, 0U, 1180U, 50U) == OLED_SELFCHECK_FAIL,
              "invalid line frame fails");
  require_int(OLED_SelfCheckEsp12f(1200U, 1000U, 500U, 0U) == OLED_SELFCHECK_OK,
              "recent ESP frame passes");
  require_int(OLED_SelfCheckEsp12f(2000U, 1000U, 500U, 0U) == OLED_SELFCHECK_FAIL,
              "stale ESP frame fails");
  require_int(OLED_SelfCheckEsp12f(1200U, 0U, 500U, 1U) == OLED_SELFCHECK_SKIP,
              "ESP download mode skips");
  require_int(OLED_SelfCheckEsp12f(1200U, 0U, 500U, 0U) == OLED_SELFCHECK_FAIL,
              "ESP no RX fails");
}

int main(void)
{
  test_rpi_line_and_esp_link_statuses();

  (void)printf("PASS: oled selfcheck host tests\n");
  return 0;
}
