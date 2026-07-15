#include "bmi270_bus.h"

#include "bmi270_registers.h"
#include "main.h"
#include "spi.h"

#include <string.h>

#define BMI270_BUS_SPI_TIMEOUT_MS  10U
#define BMI270_BUS_SELECT_DELAY_MS 1U
#define BMI270_BUS_READ_MAX_BYTES  128U
#define BMI270_BUS_WRITE_MAX_BYTES 32U

static void Bmi270Bus_CsLow(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static void Bmi270Bus_CsHigh(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

void Bmi270Bus_Deselect(void)
{
    Bmi270Bus_CsHigh();
}

static HAL_StatusTypeDef Bmi270Bus_ReadRegRaw(uint8_t reg, uint8_t rx[3])
{
    uint8_t           tx[3] = {(uint8_t)(reg | BMI270_READ_BIT), 0U, 0U};
    HAL_StatusTypeDef status;

    Bmi270Bus_CsLow();
    status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, sizeof(tx), BMI270_BUS_SPI_TIMEOUT_MS);
    Bmi270Bus_CsHigh();
    return status;
}

uint8_t Bmi270Bus_ReadReg(uint8_t reg, uint8_t *value)
{
    uint8_t rx[3] = {0U};

    if (value == 0 || Bmi270Bus_ReadRegRaw(reg, rx) != HAL_OK)
    {
        return 0U;
    }
    *value = rx[2];
    return 1U;
}

uint8_t Bmi270Bus_ReadBytes(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t rx[BMI270_BUS_READ_MAX_BYTES + 2U] = {0U};
    uint8_t tx[BMI270_BUS_READ_MAX_BYTES + 2U] = {0U};

    if (data == 0 || len == 0U || len > BMI270_BUS_READ_MAX_BYTES)
    {
        return 0U;
    }
    tx[0] = (uint8_t)(reg | BMI270_READ_BIT);
    Bmi270Bus_CsLow();
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)(len + 2U), BMI270_BUS_SPI_TIMEOUT_MS);
    Bmi270Bus_CsHigh();
    if (status != HAL_OK)
    {
        return 0U;
    }
    for (uint8_t index = 0U; index < len; ++index)
    {
        data[index] = rx[index + 2U];
    }
    return 1U;
}

uint8_t Bmi270Bus_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t           tx[2] = {(uint8_t)(reg & (uint8_t)~BMI270_READ_BIT), value};
    HAL_StatusTypeDef status;

    Bmi270Bus_CsLow();
    status = HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), BMI270_BUS_SPI_TIMEOUT_MS);
    Bmi270Bus_CsHigh();
    return (status == HAL_OK) ? 1U : 0U;
}

uint8_t Bmi270Bus_WriteBytes(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t tx[BMI270_BUS_WRITE_MAX_BYTES + 1U];

    if (data == 0 || len == 0U || len > BMI270_BUS_WRITE_MAX_BYTES)
    {
        return 0U;
    }
    tx[0] = (uint8_t)(reg & (uint8_t)~BMI270_READ_BIT);
    for (uint8_t index = 0U; index < len; ++index)
    {
        tx[index + 1U] = data[index];
    }
    Bmi270Bus_CsLow();
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi2, tx, (uint16_t)(len + 1U), BMI270_BUS_SPI_TIMEOUT_MS);
    Bmi270Bus_CsHigh();
    return (status == HAL_OK) ? 1U : 0U;
}

static void Bmi270Bus_ConfigGpioOutput(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_GPIO_WritePin(port, pin, state);
    gpio.Pin   = pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &gpio);
}

static void Bmi270Bus_ConfigMisoInput(uint32_t pull)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin   = IMU_MISO_Pin;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Pull  = pull;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IMU_MISO_GPIO_Port, &gpio);
}

static uint8_t Bmi270Bus_BitBangByte(uint8_t tx)
{
    uint8_t rx = 0U;

    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
        HAL_GPIO_WritePin(IMU_MOSI_GPIO_Port, IMU_MOSI_Pin, ((tx & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        for (volatile uint32_t delay = 0U; delay < 80U; ++delay)
        {
            __NOP();
        }
        HAL_GPIO_WritePin(IMU_SCK_GPIO_Port, IMU_SCK_Pin, GPIO_PIN_SET);
        for (volatile uint32_t delay = 0U; delay < 80U; ++delay)
        {
            __NOP();
        }
        rx <<= 1U;
        if (HAL_GPIO_ReadPin(IMU_MISO_GPIO_Port, IMU_MISO_Pin) == GPIO_PIN_SET)
        {
            rx |= 1U;
        }
        HAL_GPIO_WritePin(IMU_SCK_GPIO_Port, IMU_SCK_Pin, GPIO_PIN_RESET);
        tx <<= 1U;
    }
    return rx;
}

static void Bmi270Bus_BitBangReadRegRaw(uint8_t reg, uint8_t rx[3])
{
    Bmi270Bus_ConfigGpioOutput(IMU_SCK_GPIO_Port, IMU_SCK_Pin, GPIO_PIN_RESET);
    Bmi270Bus_ConfigGpioOutput(IMU_MOSI_GPIO_Port, IMU_MOSI_Pin, GPIO_PIN_RESET);
    Bmi270Bus_ConfigGpioOutput(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
    Bmi270Bus_ConfigMisoInput(GPIO_NOPULL);
    Bmi270Bus_CsLow();
    rx[0] = Bmi270Bus_BitBangByte((uint8_t)(reg | BMI270_READ_BIT));
    rx[1] = Bmi270Bus_BitBangByte(0U);
    rx[2] = Bmi270Bus_BitBangByte(0U);
    Bmi270Bus_CsHigh();
}

static uint8_t Bmi270Bus_ReadMisoWithPull(uint32_t pull)
{
    Bmi270Bus_ConfigMisoInput(pull);
    HAL_Delay(1U);
    return (HAL_GPIO_ReadPin(IMU_MISO_GPIO_Port, IMU_MISO_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

uint8_t Bmi270Bus_RunRecoveryProbe(uint8_t chip_id_reg, imu_bmi270_diag_t *diag)
{
    if (diag == 0)
    {
        return 0U;
    }
    memset(diag, 0, sizeof(*diag));
    Bmi270Bus_CsHigh();
    HAL_Delay(BMI270_BUS_SELECT_DELAY_MS);
    for (uint8_t index = 0U; index < 2U; ++index)
    {
        HAL_StatusTypeDef status = Bmi270Bus_ReadRegRaw(chip_id_reg, diag->hal_rx[index]);
        diag->hal_status[index]  = (uint8_t)status;
        HAL_Delay(BMI270_BUS_SELECT_DELAY_MS);
    }
    (void)HAL_SPI_DeInit(&hspi2);
    Bmi270Bus_BitBangReadRegRaw(chip_id_reg, diag->bitbang_rx);
    diag->miso_nopull   = Bmi270Bus_ReadMisoWithPull(GPIO_NOPULL);
    diag->miso_pullup   = Bmi270Bus_ReadMisoWithPull(GPIO_PULLUP);
    diag->miso_pulldown = Bmi270Bus_ReadMisoWithPull(GPIO_PULLDOWN);
    MX_SPI2_Init();
    Bmi270Bus_CsHigh();
    return 1U;
}
