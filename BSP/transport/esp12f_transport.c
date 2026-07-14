#include "esp12f_transport.h"

#include "main.h"
#include "usart.h"

uint8_t Esp12fTransport_IsTxReady(void)
{
    return (huart2.gState == HAL_UART_STATE_READY) ? 1U : 0U;
}

transport_status_t Esp12fTransport_TransmitAsync(uint8_t *data, uint16_t size)
{
    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&huart2, data, size);
    if (status == HAL_OK)
    {
        return TRANSPORT_STATUS_OK;
    }
    return (status == HAL_BUSY) ? TRANSPORT_STATUS_BUSY : TRANSPORT_STATUS_ERROR;
}

void Esp12fTransport_StartRx(uint8_t *byte)
{
    (void)HAL_UART_Receive_IT(&huart2, byte, 1U);
}

void Esp12fTransport_ResetModule(void)
{
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(5U);
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
}

void Esp12fTransport_Isolate(void)
{
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    CLEAR_BIT(huart2.Instance->CR3, USART_CR3_DMAR);
    huart2.hdmarx = 0;
    (void)HAL_UART_Abort(&huart2);
    HAL_NVIC_ClearPendingIRQ(USART2_IRQn);
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_RESET);
}

void Esp12fTransport_SetDownloadMode(uint8_t enabled)
{
    HAL_GPIO_WritePin(ESP_EN_GPIO_Port, ESP_EN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ESP_IO0_GPIO_Port, ESP_IO0_Pin, (enabled != 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(ESP_RST_GPIO_Port, ESP_RST_Pin, GPIO_PIN_SET);
}
