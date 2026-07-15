#include "esp12f_boot_control.h"

#include "main.h"

#define ESP12F_BOOT_EN_LOW_MS    50U
#define ESP12F_BOOT_EN_TO_RST_MS 10U
#define ESP12F_BOOT_WAIT_MS      100U

static void Esp12fBootControl_Enter(uint8_t download_mode)
{
    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ESP_IO0_GPIO_Port, ESP_IO0_Pin, (download_mode != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(ESP12F_BOOT_EN_LOW_MS);

    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(ESP12F_BOOT_EN_TO_RST_MS);

    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(ESP12F_BOOT_WAIT_MS);
}

void Esp12fBootControl_EnterDownload(void)
{
    Esp12fBootControl_Enter(1U);
}

void Esp12fBootControl_EnterNormal(void)
{
    Esp12fBootControl_Enter(0U);
}
