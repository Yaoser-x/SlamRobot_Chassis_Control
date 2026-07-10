#ifndef OLED_SELFCHECK_H
#define OLED_SELFCHECK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  OLED_SELFCHECK_PENDING = 0,
  OLED_SELFCHECK_OK = 1,
  OLED_SELFCHECK_FAIL = 2,
  OLED_SELFCHECK_SKIP = 3
} oled_selfcheck_status_t;

oled_selfcheck_status_t OLED_SelfCheckRpi(uint32_t now_ms,
                                          uint32_t last_rx_ms,
                                          uint32_t timeout_ms);
oled_selfcheck_status_t OLED_SelfCheckLine(uint32_t now_ms,
                                           uint8_t data_valid,
                                           uint32_t timestamp_ms,
                                           uint32_t timeout_ms);
oled_selfcheck_status_t OLED_SelfCheckEsp12f(uint32_t now_ms,
                                             uint32_t last_rx_ms,
                                             uint32_t timeout_ms,
                                             uint8_t download_mode);

#ifdef __cplusplus
}
#endif

#endif /* OLED_SELFCHECK_H */
