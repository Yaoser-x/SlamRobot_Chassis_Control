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
static uint32_t     delays[3];
static uint8_t      write_count;
static uint8_t      delay_count;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    writes[write_count++] = (gpio_write_t){.port = port, .pin = pin, .state = state};
}

void HAL_Delay(uint32_t delay_ms)
{
    delays[delay_count++] = delay_ms;
}

static void AssertCommonSequence(GPIO_PinState io0_state)
{
    assert(write_count == 5U);
    assert(delay_count == 3U);
    assert(writes[0].port == ESP_EN_GPIO_Port && writes[0].pin == ESP_EN_Pin && writes[0].state == GPIO_PIN_RESET);
    assert(writes[1].port == ESP_IO0_GPIO_Port && writes[1].pin == ESP_IO0_Pin && writes[1].state == io0_state);
    assert(writes[2].port == ESP_RST_GPIO_Port && writes[2].pin == ESP_RST_Pin && writes[2].state == GPIO_PIN_RESET);
    assert(writes[3].port == ESP_EN_GPIO_Port && writes[3].pin == ESP_EN_Pin && writes[3].state == GPIO_PIN_SET);
    assert(writes[4].port == ESP_RST_GPIO_Port && writes[4].pin == ESP_RST_Pin && writes[4].state == GPIO_PIN_SET);
    assert(delays[0] == 50U && delays[1] == 10U && delays[2] == 100U);
}

int main(void)
{
    Esp12fBootControl_EnterDownload();
    AssertCommonSequence(GPIO_PIN_RESET);

    write_count = 0U;
    delay_count = 0U;
    Esp12fBootControl_EnterNormal();
    AssertCommonSequence(GPIO_PIN_SET);

    puts("PASS: ESP12F boot control");
    return 0;
}
