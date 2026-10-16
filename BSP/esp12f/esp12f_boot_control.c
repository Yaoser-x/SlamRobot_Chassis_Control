#include "esp12f_boot_control.h"

#include "main.h"
#include "platform_time.h"

#define ESP12F_BOOT_EN_LOW_MS    50U
#define ESP12F_BOOT_EN_TO_RST_MS 10U
#define ESP12F_BOOT_WAIT_MS      100U

typedef enum
{
    ESP12F_BOOT_READY = 0,
    ESP12F_BOOT_EN_LOW,
    ESP12F_BOOT_EN_TO_RST,
    ESP12F_BOOT_WAIT
} esp12f_boot_phase_t;

static esp12f_boot_phase_t boot_phase;
static uint32_t            boot_phase_start_ms;

static void Esp12fBootControl_Enter(uint8_t download_mode)
{
    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ESP_IO0_GPIO_Port, ESP_IO0_Pin, (download_mode != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
    boot_phase_start_ms = PlatformTime_NowMs();
    boot_phase          = ESP12F_BOOT_EN_LOW;
}

void Esp12fBootControl_EnterDownload(void)
{
    Esp12fBootControl_Enter(1U);
}

void Esp12fBootControl_EnterNormal(void)
{
    Esp12fBootControl_Enter(0U);
}

uint8_t Esp12fBootControl_Update(uint32_t now_ms)
{
    uint32_t elapsed_ms = now_ms - boot_phase_start_ms;
    switch (boot_phase)
    {
        case ESP12F_BOOT_EN_LOW:
            if (elapsed_ms >= ESP12F_BOOT_EN_LOW_MS)
            {
                HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
                boot_phase_start_ms = now_ms;
                boot_phase          = ESP12F_BOOT_EN_TO_RST;
            }
            break;
        case ESP12F_BOOT_EN_TO_RST:
            if (elapsed_ms >= ESP12F_BOOT_EN_TO_RST_MS)
            {
                HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
                boot_phase_start_ms = now_ms;
                boot_phase          = ESP12F_BOOT_WAIT;
            }
            break;
        case ESP12F_BOOT_WAIT:
            if (elapsed_ms >= ESP12F_BOOT_WAIT_MS)
            {
                boot_phase = ESP12F_BOOT_READY;
            }
            break;
        default:
            break;
    }
    return (boot_phase == ESP12F_BOOT_READY) ? 1U : 0U;
}
