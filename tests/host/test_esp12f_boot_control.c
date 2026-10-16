#include "esp12f_boot_control.h"

#include "main.h"

#include <assert.h>
#include <stdio.h>

GPIO_TypeDef GPIOB_Instance;
GPIO_TypeDef GPIOD_Instance;

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    GPIO_PinState state;
} gpio_write_t;

static gpio_write_t writes[6];
static uint8_t      write_count;
static uint32_t     fake_now_ms;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    writes[write_count++] = (gpio_write_t){.port = port, .pin = pin, .state = state};
}

void HAL_Delay(uint32_t delay_ms)
{
    (void)delay_ms;
    assert(0 && "Boot sequencing must not block a task");
}

uint32_t PlatformTime_NowMs(void)
{
    return fake_now_ms;
}

static void AssertCommonSequence(GPIO_PinState io0_state, uint32_t start)
{
    assert(write_count == 3U);
    assert(writes[0].port == ESP_EN_GPIO_Port && writes[0].pin == ESP_EN_Pin && writes[0].state == GPIO_PIN_RESET);
    assert(writes[1].port == ESP_IO0_GPIO_Port && writes[1].pin == ESP_IO0_Pin && writes[1].state == io0_state);
    assert(writes[2].port == ESP_RST_GPIO_Port && writes[2].pin == ESP_RST_Pin && writes[2].state == GPIO_PIN_RESET);
    assert(Esp12fBootControl_Update(start + 49U) == 0U && write_count == 3U);
    assert(Esp12fBootControl_Update(start + 50U) == 0U && write_count == 4U);
    assert(writes[3].port == ESP_EN_GPIO_Port && writes[3].pin == ESP_EN_Pin && writes[3].state == GPIO_PIN_SET);
    assert(Esp12fBootControl_Update(start + 59U) == 0U && write_count == 4U);
    assert(Esp12fBootControl_Update(start + 60U) == 0U && write_count == 5U);
    assert(writes[4].port == ESP_RST_GPIO_Port && writes[4].pin == ESP_RST_Pin && writes[4].state == GPIO_PIN_SET);
    assert(Esp12fBootControl_Update(start + 159U) == 0U);
    assert(Esp12fBootControl_Update(start + 160U) != 0U);
    assert(Esp12fBootControl_Update(start + 500U) != 0U && write_count == 5U);
}

int main(void)
{
    Esp12fBootControl_EnterDownload();
    AssertCommonSequence(GPIO_PIN_RESET, 0U);

    write_count = 0U;
    fake_now_ms = 0xFFFFFFF0UL;
    Esp12fBootControl_EnterNormal();
    AssertCommonSequence(GPIO_PIN_SET, fake_now_ms);

    /* A late poll must still give the actual EN edge its full settling interval. */
    write_count = 0U;
    fake_now_ms = 1000U;
    Esp12fBootControl_EnterDownload();
    assert(Esp12fBootControl_Update(2000U) == 0U && write_count == 4U);
    assert(Esp12fBootControl_Update(2009U) == 0U && write_count == 4U);
    assert(Esp12fBootControl_Update(2010U) == 0U && write_count == 5U);
    assert(Esp12fBootControl_Update(2109U) == 0U);
    assert(Esp12fBootControl_Update(2110U) != 0U);

    puts("PASS: ESP12F boot control");
    return 0;
}
