#include "imu_bmi270.h"

#include "imu_bmi270_config.h"
#include "main.h"
#include "spi.h"

#include <math.h>
#include <string.h>

#define BMI270_REG_CHIP_ID       0x00U
#define BMI270_REG_ERR_REG       0x02U
#define BMI270_REG_DATA_8        0x0CU
#define BMI270_REG_INTERNAL_STATUS 0x21U
#define BMI270_REG_ACC_CONF      0x40U
#define BMI270_REG_ACC_RANGE     0x41U
#define BMI270_REG_GYR_CONF      0x42U
#define BMI270_REG_GYR_RANGE     0x43U
#define BMI270_REG_INIT_CTRL     0x59U
#define BMI270_REG_INIT_ADDR_0   0x5BU
#define BMI270_REG_INIT_ADDR_1   0x5CU
#define BMI270_REG_INIT_DATA     0x5EU
#define BMI270_REG_PWR_CONF      0x7CU
#define BMI270_REG_PWR_CTRL      0x7DU
#define BMI270_REG_CMD           0x7EU
#define BMI270_READ_BIT          0x80U
#define BMI270_CHIP_ID           0x24U
#define BMI270_CMD_SOFT_RESET    0xB6U
#define BMI270_SPI_TIMEOUT_MS    10U
#define BMI270_SPI_SELECT_DELAY_MS 1U
#define BMI270_INIT_POLL_DELAY_MS 1U
#define BMI270_INIT_TIMEOUT_MS   25U
#define BMI270_INIT_RETRY_MS     1000U
#define BMI270_CONFIG_CHUNK_SIZE 32U
#define BMI270_INTERNAL_STATUS_MSG_MASK 0x0FU
#define BMI270_INTERNAL_STATUS_INIT_OK 0x01U
#define BMI270_INIT_CTRL_PREPARE 0x00U
#define BMI270_INIT_CTRL_COMPLETE 0x01U
#define BMI270_PWR_CONF_APS_OFF  0x00U
#define BMI270_PWR_CTRL_ACC_GYR_TEMP_ON 0x0EU
#define BMI270_ACC_CONF_100HZ_PERF 0xA8U
#define BMI270_ACC_RANGE_2G      0x00U
#define BMI270_GYR_CONF_100HZ_NOISE_PERF 0xE8U
#define BMI270_GYR_RANGE_500DPS  0x02U
#define BMI270_ACCEL_LSB_PER_G   16384.0f
#define BMI270_GYRO_LSB_PER_DPS  65.6f
#define BMI270_ACCEL_FILTER_ALPHA 0.20f
#define BMI270_GYRO_FILTER_ALPHA 0.20f
#define BMI270_ATTITUDE_FILTER_ALPHA 0.02f
#define BMI270_ATTITUDE_MAX_DT_S 0.100f
#define BMI270_RAD_TO_DEG        57.2957795f
#define BMI270_GYRO_CAL_DEFAULT_SAMPLES 500U
#define BMI270_GYRO_CAL_MIN_SAMPLES 50U
#define BMI270_GYRO_CAL_MAX_SAMPLES 2000U
#define BMI270_GYRO_CAL_DEFAULT_DELAY_MS 10U
#define BMI270_GYRO_CAL_STILL_SPAN_DPS 5.0f
#define BMI270_GYRO_CAL_MAX_ABS_DPS 20.0f

static imu_bmi270_state_t imu_state;
static uint32_t imu_next_init_retry_ms;

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

static float ImuBmi270_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float ImuBmi270_Filter(float previous, float input, float alpha)
{
  return previous + (alpha * (input - previous));
}

static float ImuBmi270_WrapAngleDeg(float angle_deg)
{
  while (angle_deg > 180.0f)
  {
    angle_deg -= 360.0f;
  }
  while (angle_deg <= -180.0f)
  {
    angle_deg += 360.0f;
  }
  return angle_deg;
}

static void ImuBmi270_AccelEulerDeg(const float accel_g[3], float *roll_deg, float *pitch_deg)
{
  *roll_deg = atan2f(accel_g[1], accel_g[2]) * BMI270_RAD_TO_DEG;
  *pitch_deg = atan2f(-accel_g[0],
                      sqrtf((accel_g[1] * accel_g[1]) + (accel_g[2] * accel_g[2]))) *
               BMI270_RAD_TO_DEG;
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

static HAL_StatusTypeDef ImuBmi270_ReadRegRaw(uint8_t reg, uint8_t rx[3])
{
  uint8_t tx[3] = { (uint8_t)(reg | BMI270_READ_BIT), 0U, 0U };
  HAL_StatusTypeDef status;

  ImuBmi270_CsLow();
  status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, sizeof(tx), BMI270_SPI_TIMEOUT_MS);
  ImuBmi270_CsHigh();
  return status;
}

uint8_t ImuBmi270_ReadReg(uint8_t reg, uint8_t *value)
{
  uint8_t rx[3] = { 0U, 0U, 0U };
  HAL_StatusTypeDef status;

  if (value == 0)
  {
    return 0U;
  }

  status = ImuBmi270_ReadRegRaw(reg, rx);
  if (status != HAL_OK)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_SPI);
    return 0U;
  }
  *value = rx[2];
  return 1U;
}

static void ImuBmi270_ConfigGpioOutput(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
  GPIO_InitTypeDef gpio = {0};

  HAL_GPIO_WritePin(port, pin, state);
  gpio.Pin = pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(port, &gpio);
}

static void ImuBmi270_ConfigMisoInput(uint32_t pull)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = IMU_MISO_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = pull;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_MISO_GPIO_Port, &gpio);
}

static uint8_t ImuBmi270_BitBangByte(uint8_t tx)
{
  uint8_t rx = 0U;

  for (uint8_t bit = 0U; bit < 8U; ++bit)
  {
    HAL_GPIO_WritePin(IMU_MOSI_GPIO_Port, IMU_MOSI_Pin,
                      ((tx & 0x80U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
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

static void ImuBmi270_BitBangReadRegRaw(uint8_t reg, uint8_t rx[3])
{
  ImuBmi270_ConfigGpioOutput(IMU_SCK_GPIO_Port, IMU_SCK_Pin, GPIO_PIN_RESET);
  ImuBmi270_ConfigGpioOutput(IMU_MOSI_GPIO_Port, IMU_MOSI_Pin, GPIO_PIN_RESET);
  ImuBmi270_ConfigGpioOutput(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  ImuBmi270_ConfigMisoInput(GPIO_NOPULL);

  ImuBmi270_CsLow();
  rx[0] = ImuBmi270_BitBangByte((uint8_t)(reg | BMI270_READ_BIT));
  rx[1] = ImuBmi270_BitBangByte(0U);
  rx[2] = ImuBmi270_BitBangByte(0U);
  ImuBmi270_CsHigh();
}

static uint8_t ImuBmi270_ReadMisoWithPull(uint32_t pull)
{
  ImuBmi270_ConfigMisoInput(pull);
  HAL_Delay(1U);
  return (HAL_GPIO_ReadPin(IMU_MISO_GPIO_Port, IMU_MISO_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

uint8_t ImuBmi270_Diagnose(imu_bmi270_diag_t *diag)
{
  if (diag == 0)
  {
    return 0U;
  }

  memset(diag, 0, sizeof(*diag));
  ImuBmi270_CsHigh();
  HAL_Delay(BMI270_SPI_SELECT_DELAY_MS);

  for (uint8_t i = 0U; i < 2U; ++i)
  {
    HAL_StatusTypeDef status = ImuBmi270_ReadRegRaw(BMI270_REG_CHIP_ID, diag->hal_rx[i]);
    diag->hal_status[i] = (uint8_t)status;
    HAL_Delay(BMI270_SPI_SELECT_DELAY_MS);
  }

  (void)HAL_SPI_DeInit(&hspi2);
  ImuBmi270_BitBangReadRegRaw(BMI270_REG_CHIP_ID, diag->bitbang_rx);
  diag->miso_nopull = ImuBmi270_ReadMisoWithPull(GPIO_NOPULL);
  diag->miso_pullup = ImuBmi270_ReadMisoWithPull(GPIO_PULLUP);
  diag->miso_pulldown = ImuBmi270_ReadMisoWithPull(GPIO_PULLDOWN);

  MX_SPI2_Init();
  ImuBmi270_CsHigh();
  return 1U;
}

static uint8_t ImuBmi270_ReadBytes(uint8_t reg, uint8_t *data, uint8_t len)
{
  HAL_StatusTypeDef status;
  uint8_t addr = (uint8_t)(reg | BMI270_READ_BIT);
  uint8_t rx[BMI270_CONFIG_CHUNK_SIZE + 2U] = {0U};
  uint8_t tx[BMI270_CONFIG_CHUNK_SIZE + 2U] = {0U};

  if (data == 0 || len == 0U || len > BMI270_CONFIG_CHUNK_SIZE)
  {
    return 0U;
  }

  tx[0] = addr;
  ImuBmi270_CsLow();
  status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, (uint16_t)(len + 2U), BMI270_SPI_TIMEOUT_MS);
  ImuBmi270_CsHigh();
  if (status != HAL_OK)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_SPI);
    return 0U;
  }
  for (uint8_t i = 0U; i < len; ++i)
  {
    data[i] = rx[i + 2U];
  }
  return 1U;
}

static uint8_t ImuBmi270_ReadRawFrame(int16_t accel_raw[3], int16_t gyro_raw[3])
{
  uint8_t data[12];

  if (accel_raw == 0 || gyro_raw == 0)
  {
    return 0U;
  }
  if (ImuBmi270_ReadBytes(BMI270_REG_DATA_8, data, sizeof(data)) == 0U)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_READ);
    return 0U;
  }

  accel_raw[0] = ImuBmi270_ReadI16(&data[0]);
  accel_raw[1] = ImuBmi270_ReadI16(&data[2]);
  accel_raw[2] = ImuBmi270_ReadI16(&data[4]);
  gyro_raw[0] = ImuBmi270_ReadI16(&data[6]);
  gyro_raw[1] = ImuBmi270_ReadI16(&data[8]);
  gyro_raw[2] = ImuBmi270_ReadI16(&data[10]);
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

static uint8_t ImuBmi270_WriteBytes(uint8_t reg, const uint8_t *data, uint8_t len)
{
  HAL_StatusTypeDef status;
  uint8_t tx[BMI270_CONFIG_CHUNK_SIZE + 1U];

  if (data == 0 || len == 0U || len > BMI270_CONFIG_CHUNK_SIZE)
  {
    return 0U;
  }

  tx[0] = (uint8_t)(reg & (uint8_t)~BMI270_READ_BIT);
  for (uint8_t i = 0U; i < len; ++i)
  {
    tx[i + 1U] = data[i];
  }

  ImuBmi270_CsLow();
  status = HAL_SPI_Transmit(&hspi2, tx, (uint16_t)(len + 1U), BMI270_SPI_TIMEOUT_MS);
  ImuBmi270_CsHigh();
  if (status != HAL_OK)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_SPI);
    return 0U;
  }
  return 1U;
}

static uint8_t ImuBmi270_SetInitAddress(uint16_t byte_offset)
{
  uint16_t word_addr = (uint16_t)(byte_offset / 2U);

  if (ImuBmi270_WriteReg(BMI270_REG_INIT_ADDR_0, (uint8_t)(word_addr & 0x0FU)) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_INIT_ADDR_1, (uint8_t)((word_addr >> 4U) & 0xFFU)) == 0U)
  {
    return 0U;
  }
  return 1U;
}

static uint8_t ImuBmi270_LoadConfigFile(void)
{
  uint32_t offset = 0U;

  if (ImuBmi270_WriteReg(BMI270_REG_INIT_CTRL, BMI270_INIT_CTRL_PREPARE) == 0U)
  {
    return 0U;
  }

  while (offset < bmi270_config_file_size)
  {
    uint32_t remaining = bmi270_config_file_size - offset;
    uint8_t chunk_len = (remaining > BMI270_CONFIG_CHUNK_SIZE) ?
                        BMI270_CONFIG_CHUNK_SIZE : (uint8_t)remaining;

    if ((chunk_len & 1U) != 0U)
    {
      ImuBmi270_SetError(IMU_BMI270_ERROR_CONFIG);
      return 0U;
    }
    if (ImuBmi270_SetInitAddress((uint16_t)offset) == 0U)
    {
      return 0U;
    }
    if (ImuBmi270_WriteBytes(BMI270_REG_INIT_DATA, &bmi270_config_file[offset], chunk_len) == 0U)
    {
      return 0U;
    }
    offset += chunk_len;
  }

  if (ImuBmi270_WriteReg(BMI270_REG_INIT_CTRL, BMI270_INIT_CTRL_COMPLETE) == 0U)
  {
    return 0U;
  }
  return 1U;
}

static uint8_t ImuBmi270_WaitInitOk(void)
{
  uint8_t status = 0U;

  for (uint32_t elapsed_ms = 0U; elapsed_ms < BMI270_INIT_TIMEOUT_MS; elapsed_ms += BMI270_INIT_POLL_DELAY_MS)
  {
    HAL_Delay(BMI270_INIT_POLL_DELAY_MS);
    if (ImuBmi270_ReadReg(BMI270_REG_INTERNAL_STATUS, &status) == 0U)
    {
      return 0U;
    }
    if ((status & BMI270_INTERNAL_STATUS_MSG_MASK) == BMI270_INTERNAL_STATUS_INIT_OK)
    {
      return 1U;
    }
  }

  ImuBmi270_SetError(IMU_BMI270_ERROR_CONFIG);
  return 0U;
}

void ImuBmi270_Init(void)
{
  imu_state = (imu_bmi270_state_t){0};
  imu_state.enabled = 1U;
  imu_next_init_retry_ms = 0U;
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
  HAL_Delay(BMI270_SPI_SELECT_DELAY_MS);
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

  if (ImuBmi270_WriteReg(BMI270_REG_PWR_CONF, BMI270_PWR_CONF_APS_OFF) == 0U)
  {
    return 0U;
  }
  HAL_Delay(BMI270_SPI_SELECT_DELAY_MS);
  if (ImuBmi270_LoadConfigFile() == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WaitInitOk() == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_ACC_CONF, BMI270_ACC_CONF_100HZ_PERF) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_ACC_RANGE, BMI270_ACC_RANGE_2G) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_GYR_CONF, BMI270_GYR_CONF_100HZ_NOISE_PERF) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_GYR_RANGE, BMI270_GYR_RANGE_500DPS) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_PWR_CTRL, BMI270_PWR_CTRL_ACC_GYR_TEMP_ON) == 0U)
  {
    return 0U;
  }
  HAL_Delay(2U);
  imu_state.enabled = 1U;
  imu_state.filter_initialized = 0U;
  ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
  return 1U;
}

uint8_t ImuBmi270_Update(void)
{
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  uint32_t primask;
  uint32_t now_ms;
  uint32_t update_ms;
  uint32_t elapsed_ms;
  float accel_g[3];
  float gyro_raw_dps[3];
  float gyro_corrected_dps[3];
  float accel_roll_deg;
  float accel_pitch_deg;
  float attitude_dt_s;

  if (imu_state.enabled == 0U)
  {
    return 1U;
  }
  if (imu_state.online == 0U)
  {
    now_ms = HAL_GetTick();
    if ((int32_t)(now_ms - imu_next_init_retry_ms) < 0)
    {
      return 0U;
    }
    if (ImuBmi270_ConfigNow() == 0U)
    {
      imu_next_init_retry_ms = now_ms + BMI270_INIT_RETRY_MS;
      return 0U;
    }
  }

  if (ImuBmi270_ReadRawFrame(accel_raw, gyro_raw) == 0U)
  {
    return 0U;
  }
  update_ms = HAL_GetTick();

  for (uint8_t i = 0U; i < 3U; ++i)
  {
    accel_g[i] = (float)accel_raw[i] / BMI270_ACCEL_LSB_PER_G;
    gyro_raw_dps[i] = (float)gyro_raw[i] / BMI270_GYRO_LSB_PER_DPS;
    gyro_corrected_dps[i] = gyro_raw_dps[i] - imu_state.gyro_bias_dps[i];
  }
  ImuBmi270_AccelEulerDeg(accel_g, &accel_roll_deg, &accel_pitch_deg);

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    imu_state.accel_raw[i] = accel_raw[i];
    imu_state.gyro_raw[i] = gyro_raw[i];
    imu_state.gyro_corrected_dps[i] = gyro_corrected_dps[i];
  }
  if (imu_state.filter_initialized == 0U)
  {
    for (uint8_t i = 0U; i < 3U; ++i)
    {
      imu_state.accel_g[i] = accel_g[i];
      imu_state.gyro_filtered_dps[i] = gyro_corrected_dps[i];
      imu_state.gyro_dps[i] = gyro_corrected_dps[i];
    }
    imu_state.roll_deg = accel_roll_deg;
    imu_state.pitch_deg = accel_pitch_deg;
    imu_state.yaw_deg = 0.0f;
    imu_state.filter_initialized = 1U;
  }
  else
  {
    for (uint8_t i = 0U; i < 3U; ++i)
    {
      imu_state.accel_g[i] = ImuBmi270_Filter(imu_state.accel_g[i], accel_g[i], BMI270_ACCEL_FILTER_ALPHA);
      imu_state.gyro_filtered_dps[i] = ImuBmi270_Filter(imu_state.gyro_filtered_dps[i],
                                                        gyro_corrected_dps[i],
                                                        BMI270_GYRO_FILTER_ALPHA);
      imu_state.gyro_dps[i] = imu_state.gyro_filtered_dps[i];
    }
    elapsed_ms = update_ms - imu_state.last_update_ms;
    if ((elapsed_ms > 0U) && (((float)elapsed_ms * 0.001f) <= BMI270_ATTITUDE_MAX_DT_S))
    {
      attitude_dt_s = (float)elapsed_ms * 0.001f;
      imu_state.roll_deg = ImuBmi270_Filter(imu_state.roll_deg + (gyro_corrected_dps[0] * attitude_dt_s),
                                            accel_roll_deg,
                                            BMI270_ATTITUDE_FILTER_ALPHA);
      imu_state.pitch_deg = ImuBmi270_Filter(imu_state.pitch_deg + (gyro_corrected_dps[1] * attitude_dt_s),
                                             accel_pitch_deg,
                                             BMI270_ATTITUDE_FILTER_ALPHA);
      imu_state.yaw_deg = ImuBmi270_WrapAngleDeg(imu_state.yaw_deg + (gyro_corrected_dps[2] * attitude_dt_s));
    }
    else
    {
      imu_state.roll_deg = ImuBmi270_Filter(imu_state.roll_deg, accel_roll_deg, BMI270_ATTITUDE_FILTER_ALPHA);
      imu_state.pitch_deg = ImuBmi270_Filter(imu_state.pitch_deg, accel_pitch_deg, BMI270_ATTITUDE_FILTER_ALPHA);
    }
  }
  imu_state.last_update_ms = update_ms;
  imu_state.online = 1U;
  __set_PRIMASK(primask);

  ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
  return 1U;
}

uint8_t ImuBmi270_CalibrateGyro(uint16_t samples, uint16_t delay_ms)
{
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  float sum[3] = {0.0f, 0.0f, 0.0f};
  float min_dps[3] = {0.0f, 0.0f, 0.0f};
  float max_dps[3] = {0.0f, 0.0f, 0.0f};
  uint32_t primask;

  if (samples == 0U)
  {
    samples = BMI270_GYRO_CAL_DEFAULT_SAMPLES;
  }
  if (delay_ms == 0U)
  {
    delay_ms = BMI270_GYRO_CAL_DEFAULT_DELAY_MS;
  }
  if (samples < BMI270_GYRO_CAL_MIN_SAMPLES)
  {
    samples = BMI270_GYRO_CAL_MIN_SAMPLES;
  }
  if (samples > BMI270_GYRO_CAL_MAX_SAMPLES)
  {
    samples = BMI270_GYRO_CAL_MAX_SAMPLES;
  }

  if (imu_state.online == 0U && ImuBmi270_ConfigNow() == 0U)
  {
    return 0U;
  }

  for (uint16_t sample = 0U; sample < samples; ++sample)
  {
    if (ImuBmi270_ReadRawFrame(accel_raw, gyro_raw) == 0U)
    {
      return 0U;
    }

    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
      float dps = (float)gyro_raw[axis] / BMI270_GYRO_LSB_PER_DPS;

      if (sample == 0U)
      {
        min_dps[axis] = dps;
        max_dps[axis] = dps;
      }
      if (dps < min_dps[axis])
      {
        min_dps[axis] = dps;
      }
      if (dps > max_dps[axis])
      {
        max_dps[axis] = dps;
      }
      if (ImuBmi270_AbsFloat(dps) > BMI270_GYRO_CAL_MAX_ABS_DPS)
      {
        return 0U;
      }
      sum[axis] += dps;
    }

    HAL_Delay(delay_ms);
  }

  for (uint8_t i = 0U; i < 3U; ++i)
  {
    if ((max_dps[i] - min_dps[i]) > BMI270_GYRO_CAL_STILL_SPAN_DPS)
    {
      return 0U;
    }
  }

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    imu_state.gyro_bias_dps[i] = sum[i] / (float)samples;
    imu_state.gyro_corrected_dps[i] = 0.0f;
    imu_state.gyro_filtered_dps[i] = 0.0f;
    imu_state.gyro_dps[i] = 0.0f;
  }
  imu_state.gyro_calibrated = 1U;
  imu_state.filter_initialized = 0U;
  __set_PRIMASK(primask);

  ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
  return 1U;
}

void ImuBmi270_ClearCalibration(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    imu_state.gyro_bias_dps[i] = 0.0f;
    imu_state.gyro_corrected_dps[i] = 0.0f;
    imu_state.gyro_filtered_dps[i] = 0.0f;
    imu_state.gyro_dps[i] = 0.0f;
  }
  imu_state.gyro_calibrated = 0U;
  imu_state.filter_initialized = 0U;
  __set_PRIMASK(primask);
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
