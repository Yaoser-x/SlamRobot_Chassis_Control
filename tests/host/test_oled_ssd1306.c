#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "ssd1306.h"
#include "control_config.h"
#include "i2c.h"

static I2C_TypeDef i2c1_instance = {.CR1 = I2C_CR1_PE};
I2C_HandleTypeDef  hi2c1         = {.Instance = &i2c1_instance};

static uint32_t          mem_write_count;
static uint32_t          data_write_count;
static uint32_t          deinit_count;
static uint32_t          init_count;
static uint16_t          ready_address;
static uint32_t          ready_trials;
static uint32_t          ready_timeout;
static HAL_StatusTypeDef next_i2c_status = HAL_OK;

void HAL_Delay(uint32_t delay_ms)
{
    (void)delay_ms;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
                                    uint16_t           DevAddress,
                                    uint16_t           MemAddress,
                                    uint16_t           MemAddSize,
                                    uint8_t           *pData,
                                    uint16_t           Size,
                                    uint32_t           Timeout)
{
    (void)hi2c;
    (void)DevAddress;
    (void)MemAddSize;
    (void)pData;
    (void)Timeout;
    mem_write_count++;
    if (MemAddress == 0x40U && Size == SSD1306_WIDTH)
    {
        data_write_count++;
    }
    return next_i2c_status;
}

HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    deinit_count++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    init_count++;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)
{
    (void)hi2c;
    ready_address = DevAddress;
    ready_trials  = Trials;
    ready_timeout = Timeout;
    return next_i2c_status;
}

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)printf("FAIL: %s\n", message);
        exit(1);
    }
}

static void test_dirty_pages_only_refresh_changed_pages(void)
{
    SSD1306_Clear();
    SSD1306_Refresh();
    require_int(data_write_count == SSD1306_PAGES, "clear marks all pages dirty");

    data_write_count = 0U;
    SSD1306_Refresh();
    require_int(data_write_count == 0U, "clean framebuffer does not rewrite pages");

    SSD1306_SetPixel(3U, 9U, 1U);
    SSD1306_Refresh();
    require_int(data_write_count == 1U, "single pixel dirties one page");
}

static void test_i2c_failures_trigger_recovery(void)
{
    next_i2c_status = HAL_TIMEOUT;
    for (uint8_t i = 0U; i < 3U; ++i)
    {
        SSD1306_SetPixel((uint8_t)(10U + i), 0U, 1U);
        SSD1306_Refresh();
    }
    require_int(deinit_count != 0U, "i2c recovery deinitializes bus");
    require_int(init_count != 0U, "i2c recovery initializes bus");
}

static void test_ready_probe_uses_oled_address(void)
{
    next_i2c_status = HAL_OK;
    require_int(SSD1306_IsReady() != 0U, "ready probe reports HAL success");
    require_int(ready_address == (uint16_t)(OLED_I2C_ADDR << 1), "ready probe uses shifted address");
    require_int(ready_trials == 2U, "ready probe uses two trials");
    require_int(ready_timeout == 10U, "ready probe uses bounded timeout");

    next_i2c_status = HAL_TIMEOUT;
    require_int(SSD1306_IsReady() == 0U, "ready probe reports HAL failure");
}

int main(void)
{
    test_dirty_pages_only_refresh_changed_pages();
    test_i2c_failures_trigger_recovery();
    test_ready_probe_uses_oled_address();
    (void)printf("PASS: oled ssd1306 host tests\n");
    return 0;
}
