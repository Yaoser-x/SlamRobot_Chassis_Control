#include "oled_selfcheck.h"

oled_selfcheck_status_t OLED_SelfCheckRpi(uint32_t now_ms, uint32_t last_rx_ms, uint32_t timeout_ms)
{
    if (last_rx_ms == 0U)
    {
        return OLED_SELFCHECK_FAIL;
    }
    return ((now_ms - last_rx_ms) <= timeout_ms) ? OLED_SELFCHECK_OK : OLED_SELFCHECK_FAIL;
}

oled_selfcheck_status_t
OLED_SelfCheckLine(uint32_t now_ms, uint8_t data_valid, uint32_t timestamp_ms, uint32_t timeout_ms)
{
    if (data_valid == 0U || timestamp_ms == 0U)
    {
        return OLED_SELFCHECK_FAIL;
    }
    return ((now_ms - timestamp_ms) <= timeout_ms) ? OLED_SELFCHECK_OK : OLED_SELFCHECK_FAIL;
}

oled_selfcheck_status_t
OLED_SelfCheckEsp12f(uint32_t now_ms, uint32_t last_rx_ms, uint32_t timeout_ms, uint8_t download_mode)
{
    if (download_mode != 0U)
    {
        return OLED_SELFCHECK_SKIP;
    }
    if (last_rx_ms == 0U)
    {
        return OLED_SELFCHECK_FAIL;
    }
    return ((now_ms - last_rx_ms) <= timeout_ms) ? OLED_SELFCHECK_OK : OLED_SELFCHECK_FAIL;
}
