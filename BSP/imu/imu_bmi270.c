#include "imu_bmi270.h"

#include "main.h"
#include "spi.h"

#define BMI270_REG_CHIP_ID       0x00U
#define BMI270_REG_ERR_REG       0x02U
#define BMI270_REG_DATA_8        0x0CU
#define BMI270_REG_STATUS        0x1BU
#define BMI270_REG_PWR_CONF      0x7CU
#define BMI270_REG_PWR_CTRL      0x7DU
#define BMI270_REG_CMD           0x7EU
#define BMI270_READ_BIT          0x80U
#define BMI270_CHIP_ID           0x24U
#define BMI270_CMD_SOFT_RESET    0xB6U
#define BMI270_SPI_TIMEOUT_MS    10U
#define BMI270_ACCEL_LSB_PER_G   16384.0f
#define BMI270_GYRO_LSB_PER_DPS  16.4f

static imu_bmi270_state_t imu_state;

static void ImuBmi270_CsLow(void)
{
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static void ImuBmi270_CsHigh(void)
{
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
}

static int16_t ImuBmi270_ReadI16(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static void ImuBmi270_SetError(uint8_t error)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  imu_state.last_error = error;
  if (error != IMU_BMI270_ERROR_NONE)
  {
    imu_state.error_count++;
  }
  __set_PRIMASK(primask);
}

uint8_t ImuBmi270_ReadReg(uint8_t reg, uint8_t *value)
{
  uint8_t tx[3] = { (uint8_t)(reg | BMI270_READ_BIT), 0U, 0U };
  uint8_t rx[3] = { 0U, 0U, 0U };
  HAL_StatusTypeDef status;

  if (value == 0)
  {
    return 0U;
  }

  ImuBmi270_CsLow();
  status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, sizeof(tx), BMI270_SPI_TIMEOUT_MS);
  ImuBmi270_CsHigh();
  if (status != HAL_OK)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_SPI);
    return 0U;
  }
  *value = rx[2];
  return 1U;
}

static uint8_t ImuBmi270_ReadBytes(uint8_t reg, uint8_t *data, uint8_t len)
{
  HAL_StatusTypeDef status;
  uint8_t addr = (uint8_t)(reg | BMI270_READ_BIT);
  uint8_t dummy = 0U;

  if (data == 0 || len == 0U)
  {
    return 0U;
  }

  ImuBmi270_CsLow();
  status = HAL_SPI_Transmit(&hspi2, &addr, 1U, BMI270_SPI_TIMEOUT_MS);
  if (status == HAL_OK)
  {
    status = HAL_SPI_TransmitReceive(&hspi2, &dummy, data, 1U, BMI270_SPI_TIMEOUT_MS);
  }
  if (status == HAL_OK)
  {
    status = HAL_SPI_Receive(&hspi2, data, len, BMI270_SPI_TIMEOUT_MS);
  }
  ImuBmi270_CsHigh();
  if (status != HAL_OK)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_SPI);
    return 0U;
  }
  return 1U;
}

uint8_t ImuBmi270_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { (uint8_t)(reg & (uint8_t)~BMI270_READ_BIT), value };
  HAL_StatusTypeDef status;

  ImuBmi270_CsLow();
  status = HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), BMI270_SPI_TIMEOUT_MS);
  ImuBmi270_CsHigh();
  if (status != HAL_OK)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_SPI);
    return 0U;
  }
  return 1U;
}

void ImuBmi270_Init(void)
{
  imu_state = (imu_bmi270_state_t){0};
  ImuBmi270_CsHigh();
}

uint8_t ImuBmi270_SetEnabled(uint8_t enabled)
{
  imu_state.enabled = (enabled != 0U) ? 1U : 0U;
  return 1U;
}

uint8_t ImuBmi270_ProbeNow(void)
{
  uint8_t chip_id = 0U;

  (void)ImuBmi270_ReadReg(BMI270_REG_CHIP_ID, &chip_id);
  if (ImuBmi270_ReadReg(BMI270_REG_CHIP_ID, &chip_id) == 0U)
  {
    imu_state.online = 0U;
    return 0U;
  }

  imu_state.chip_id = chip_id;
  if (chip_id != BMI270_CHIP_ID)
  {
    imu_state.online = 0U;
    ImuBmi270_SetError(IMU_BMI270_ERROR_CHIP_ID);
    return 0U;
  }

  imu_state.online = 1U;
  ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
  return 1U;
}

uint8_t ImuBmi270_ConfigNow(void)
{
  if (ImuBmi270_WriteReg(BMI270_REG_CMD, BMI270_CMD_SOFT_RESET) == 0U)
  {
    return 0U;
  }
  HAL_Delay(5U);
  if (ImuBmi270_ProbeNow() == 0U)
  {
    return 0U;
  }

  (void)ImuBmi270_WriteReg(BMI270_REG_PWR_CONF, 0x00U);
  (void)ImuBmi270_WriteReg(BMI270_REG_PWR_CTRL, 0x0EU);
  imu_state.enabled = 1U;
  ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
  return 1U;
}

uint8_t ImuBmi270_Update(void)
{
  uint8_t data[12];
  uint32_t primask;

  if (imu_state.enabled == 0U)
  {
    return 1U;
  }
  if (imu_state.online == 0U && ImuBmi270_ProbeNow() == 0U)
  {
    return 0U;
  }

  if (ImuBmi270_ReadBytes(BMI270_REG_DATA_8, data, sizeof(data)) == 0U)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_READ);
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  imu_state.accel_raw[0] = ImuBmi270_ReadI16(&data[0]);
  imu_state.accel_raw[1] = ImuBmi270_ReadI16(&data[2]);
  imu_state.accel_raw[2] = ImuBmi270_ReadI16(&data[4]);
  imu_state.gyro_raw[0] = ImuBmi270_ReadI16(&data[6]);
  imu_state.gyro_raw[1] = ImuBmi270_ReadI16(&data[8]);
  imu_state.gyro_raw[2] = ImuBmi270_ReadI16(&data[10]);
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    imu_state.accel_g[i] = (float)imu_state.accel_raw[i] / BMI270_ACCEL_LSB_PER_G;
    imu_state.gyro_dps[i] = (float)imu_state.gyro_raw[i] / BMI270_GYRO_LSB_PER_DPS;
  }
  imu_state.last_update_ms = HAL_GetTick();
  imu_state.online = 1U;
  __set_PRIMASK(primask);

  ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
  return 1U;
}

void ImuBmi270_GetState(imu_bmi270_state_t *state)
{
  uint32_t primask;

  if (state == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *state = imu_state;
  __set_PRIMASK(primask);
}
