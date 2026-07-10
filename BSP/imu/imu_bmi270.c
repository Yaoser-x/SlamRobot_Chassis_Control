#include "imu_bmi270.h"

#include "imu_bmi270_calibration.h"
#include "imu_bmi270_config.h"
#include "imu_bmi270_fifo.h"
#include "imu_bmi270_math.h"
#include "imu_bmi270_time.h"
#include "main.h"
#include "spi.h"

#include <math.h>
#include <string.h>

#define BMI270_REG_CHIP_ID       0x00U
#define BMI270_REG_ERR_REG       0x02U
#define BMI270_REG_DATA_8        0x0CU
#define BMI270_REG_SENSORTIME_0  0x18U
#define BMI270_REG_INTERNAL_STATUS 0x21U
#define BMI270_REG_FIFO_LENGTH_0 0x24U
#define BMI270_REG_FIFO_DATA     0x26U
#define BMI270_REG_ACC_CONF      0x40U
#define BMI270_REG_ACC_RANGE     0x41U
#define BMI270_REG_GYR_CONF      0x42U
#define BMI270_REG_GYR_RANGE     0x43U
#define BMI270_REG_FIFO_DOWNS    0x45U
#define BMI270_REG_FIFO_WTM_0    0x46U
#define BMI270_REG_FIFO_WTM_1    0x47U
#define BMI270_REG_FIFO_CONFIG_0 0x48U
#define BMI270_REG_FIFO_CONFIG_1 0x49U
#define BMI270_REG_INT1_IO_CTRL  0x53U
#define BMI270_REG_INT_MAP_DATA  0x58U
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
#define BMI270_FIFO_READ_MAX_BYTES 128U
#define BMI270_FIFO_MAX_SAMPLES 8U
#define BMI270_INTERNAL_STATUS_MSG_MASK 0x0FU
#define BMI270_INTERNAL_STATUS_INIT_OK 0x01U
#define BMI270_INIT_CTRL_PREPARE 0x00U
#define BMI270_INIT_CTRL_COMPLETE 0x01U
#define BMI270_ACCEL_LSB_PER_G   16384.0f
#define BMI270_GYRO_LSB_PER_DPS  65.6f
#define BMI270_ACCEL_FILTER_ALPHA 0.20f
#define BMI270_GYRO_FILTER_ALPHA 0.20f
#define BMI270_ATTITUDE_MAX_DT_S 0.100f
#define BMI270_DIRECT_FALLBACK_DT_S 0.010f
#define BMI270_GYRO_SATURATION_DPS 490.0f
#define BMI270_ACCEL_REJECT_MIN_G 0.40f
#define BMI270_ACCEL_REJECT_MAX_G 1.80f
#define BMI270_GYRO_CAL_DEFAULT_SAMPLES 500U
#define BMI270_GYRO_CAL_MIN_SAMPLES 50U
#define BMI270_GYRO_CAL_MAX_SAMPLES 2000U
#define BMI270_GYRO_CAL_DEFAULT_DELAY_MS 10U
#define BMI270_GYRO_CAL_SETTLE_MS 30U
#define BMI270_GYRO_CAL_STILL_SPAN_DPS 5.0f
#define BMI270_GYRO_CAL_MAX_ABS_DPS 20.0f
#define BMI270_GYRO_AUTO_CAL_ENABLED 1U
#define BMI270_GYRO_AUTO_CAL_SAMPLES 500U
#define BMI270_GYRO_AUTO_CAL_DELAY_MS 10U
#define BMI270_GYRO_AUTO_CAL_START_DELAY_MS 1000U
#define BMI270_GYRO_AUTO_CAL_RETRY_MS 2000U
#define BMI270_GYRO_AUTO_CAL_MAX_ATTEMPTS 5U

static imu_bmi270_state_t imu_state;
static uint32_t imu_next_init_retry_ms;
static uint32_t imu_gyro_auto_cal_next_ms;
static imu_bmi270_profile_id_t imu_selected_profile = IMU_BMI270_PROFILE_PERFORMANCE;
static imu_bmi270_calibration_t imu_calibration;
static imu_bmi270_mahony_t imu_fusion;
static imu_bmi270_mahony_params_t imu_fusion_params;
static volatile uint8_t imu_gyro_calibration_active;

static void ImuBmi270_ServiceAutoCal(uint32_t now_ms);

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

static void ImuBmi270_UpdateGyroCalDiag(uint8_t reason,
                                        uint8_t axis,
                                        const float sum_dps[3],
                                        const float min_dps[3],
                                        const float max_dps[3],
                                        uint16_t sample_count)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  imu_state.gyro_cal_fail_reason = reason;
  imu_state.gyro_cal_fail_axis = axis;
  imu_state.gyro_cal_sample_count = sample_count;
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    if (sum_dps != 0 && min_dps != 0 && max_dps != 0 && sample_count != 0U)
    {
      imu_state.gyro_cal_mean_dps[i] = sum_dps[i] / (float)sample_count;
      imu_state.gyro_cal_min_dps[i] = min_dps[i];
      imu_state.gyro_cal_max_dps[i] = max_dps[i];
      imu_state.gyro_cal_span_dps[i] = max_dps[i] - min_dps[i];
    }
    else
    {
      imu_state.gyro_cal_mean_dps[i] = 0.0f;
      imu_state.gyro_cal_min_dps[i] = 0.0f;
      imu_state.gyro_cal_max_dps[i] = 0.0f;
      imu_state.gyro_cal_span_dps[i] = 0.0f;
    }
  }
  __set_PRIMASK(primask);
}

static void ImuBmi270_ScheduleAutoCal(uint32_t now_ms, uint32_t delay_ms)
{
  imu_gyro_auto_cal_next_ms = now_ms + delay_ms;
  if (imu_state.gyro_auto_cal_enabled != 0U && imu_state.gyro_calibrated == 0U)
  {
    imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_WAIT;
  }
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

static void ImuBmi270_SetError(uint8_t error)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  imu_state.last_error = error;
  if (error != IMU_BMI270_ERROR_NONE)
  {
    imu_state.error_count++;
    if (error == IMU_BMI270_ERROR_SPI)
    {
      imu_state.quality_flags |= IMU_BMI270_QUALITY_SPI_ERROR;
      imu_state.quality_latched_flags |= IMU_BMI270_QUALITY_SPI_ERROR;
      imu_state.spi_error_count++;
    }
    else if (error == IMU_BMI270_ERROR_CONFIG || error == IMU_BMI270_ERROR_CHIP_ID)
    {
      imu_state.quality_flags |= IMU_BMI270_QUALITY_INIT_FAILED;
      imu_state.quality_latched_flags |= IMU_BMI270_QUALITY_INIT_FAILED;
      imu_state.init_failure_count++;
    }
    else if (error == IMU_BMI270_ERROR_FIFO)
    {
      imu_state.quality_flags |= IMU_BMI270_QUALITY_FIFO_OVERFLOW;
      imu_state.quality_latched_flags |= IMU_BMI270_QUALITY_FIFO_OVERFLOW;
      imu_state.fifo_overflow_count++;
    }
    else if (error == IMU_BMI270_ERROR_TIMESTAMP)
    {
      imu_state.quality_flags |= IMU_BMI270_QUALITY_TIMESTAMP_ERROR;
      imu_state.quality_latched_flags |= IMU_BMI270_QUALITY_TIMESTAMP_ERROR;
      imu_state.timestamp_error_count++;
    }
    else if (error == IMU_BMI270_ERROR_PROFILE_VERIFY)
    {
      imu_state.quality_flags |= IMU_BMI270_QUALITY_PROFILE_MISMATCH;
      imu_state.quality_latched_flags |= IMU_BMI270_QUALITY_PROFILE_MISMATCH;
    }
  }
  __set_PRIMASK(primask);
}

static void ImuBmi270_SetQuality(uint32_t flags)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  imu_state.quality_flags |= flags;
  imu_state.quality_latched_flags |= flags;
  if ((flags & IMU_BMI270_QUALITY_GYRO_SATURATION) != 0UL)
  {
    imu_state.gyro_saturation_count++;
  }
  if ((flags & IMU_BMI270_QUALITY_ACCEL_ANOMALY) != 0UL)
  {
    imu_state.accel_anomaly_count++;
  }
  if ((flags & IMU_BMI270_QUALITY_ATTITUDE_INVALID) != 0UL)
  {
    imu_state.attitude_invalid_count++;
  }
  if ((flags & IMU_BMI270_QUALITY_POLL_FALLBACK) != 0UL)
  {
    imu_state.poll_fallback_count++;
  }
  __set_PRIMASK(primask);
}

static void ImuBmi270_ClearTransientQuality(void)
{
  imu_state.quality_flags &= ~(IMU_BMI270_QUALITY_SPI_ERROR |
                               IMU_BMI270_QUALITY_INIT_FAILED |
                               IMU_BMI270_QUALITY_FIFO_OVERFLOW |
                               IMU_BMI270_QUALITY_TIMESTAMP_ERROR |
                               IMU_BMI270_QUALITY_GYRO_SATURATION |
                               IMU_BMI270_QUALITY_ACCEL_ANOMALY |
                               IMU_BMI270_QUALITY_ATTITUDE_INVALID |
                               IMU_BMI270_QUALITY_POLL_FALLBACK |
                               IMU_BMI270_QUALITY_PROFILE_MISMATCH);
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
  uint8_t rx[BMI270_FIFO_READ_MAX_BYTES + 2U] = {0U};
  uint8_t tx[BMI270_FIFO_READ_MAX_BYTES + 2U] = {0U};

  if (data == 0 || len == 0U || len > BMI270_FIFO_READ_MAX_BYTES)
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
  if (ImuBmi270_RawFrameHasSignal(accel_raw, gyro_raw) == 0U)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_INVALID_FRAME);
    return 0U;
  }
  return 1U;
}

static uint8_t ImuBmi270_ReadSensorTime(uint32_t *sensor_time)
{
  uint8_t data[3];

  if (sensor_time == 0)
  {
    return 0U;
  }
  if (ImuBmi270_ReadBytes(BMI270_REG_SENSORTIME_0, data, sizeof(data)) == 0U)
  {
    return 0U;
  }

  *sensor_time = ((uint32_t)data[0]) | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16);
  return 1U;
}

static uint8_t ImuBmi270_ReadFifoLength(uint16_t *fifo_len)
{
  uint8_t data[2];

  if (fifo_len == 0)
  {
    return 0U;
  }
  if (ImuBmi270_ReadBytes(BMI270_REG_FIFO_LENGTH_0, data, sizeof(data)) == 0U)
  {
    return 0U;
  }

  *fifo_len = (uint16_t)((((uint16_t)data[1] & 0x3FU) << 8) | data[0]);
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

static uint8_t ImuBmi270_ApplyProfile(const imu_bmi270_profile_t *profile)
{
  imu_bmi270_profile_check_t check;

  if (profile == 0)
  {
    return 0U;
  }

  if (ImuBmi270_WriteReg(BMI270_REG_PWR_CONF, profile->pwr_conf) == 0U)
  {
    return 0U;
  }
  HAL_Delay(BMI270_SPI_SELECT_DELAY_MS);
  if (ImuBmi270_WriteReg(BMI270_REG_ACC_CONF, profile->acc_conf) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_ACC_RANGE, profile->acc_range) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_GYR_CONF, profile->gyr_conf) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_GYR_RANGE, profile->gyr_range) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_FIFO_DOWNS, profile->fifo_downs) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_FIFO_WTM_0, profile->fifo_wtm_0) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_FIFO_WTM_1, profile->fifo_wtm_1) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_FIFO_CONFIG_0, profile->fifo_config_0) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_FIFO_CONFIG_1, profile->fifo_config_1) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_INT1_IO_CTRL, profile->int1_io_ctrl) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_INT_MAP_DATA, profile->int_map_data) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WriteReg(BMI270_REG_PWR_CTRL, profile->pwr_ctrl) == 0U)
  {
    return 0U;
  }
  HAL_Delay(2U);

  memset(&check, 0, sizeof(check));
  if (ImuBmi270_ReadReg(BMI270_REG_ACC_CONF, &check.acc_conf) == 0U ||
      ImuBmi270_ReadReg(BMI270_REG_ACC_RANGE, &check.acc_range) == 0U ||
      ImuBmi270_ReadReg(BMI270_REG_GYR_CONF, &check.gyr_conf) == 0U ||
      ImuBmi270_ReadReg(BMI270_REG_GYR_RANGE, &check.gyr_range) == 0U ||
      ImuBmi270_ReadReg(BMI270_REG_PWR_CONF, &check.pwr_conf) == 0U ||
      ImuBmi270_ReadReg(BMI270_REG_PWR_CTRL, &check.pwr_ctrl) == 0U ||
      ImuBmi270_ReadReg(BMI270_REG_FIFO_CONFIG_0, &check.fifo_config_0) == 0U ||
      ImuBmi270_ReadReg(BMI270_REG_FIFO_CONFIG_1, &check.fifo_config_1) == 0U ||
      ImuBmi270_ReadReg(BMI270_REG_INT1_IO_CTRL, &check.int1_io_ctrl) == 0U ||
      ImuBmi270_ReadReg(BMI270_REG_INT_MAP_DATA, &check.int_map_data) == 0U)
  {
    return 0U;
  }

  if (ImuBmi270Profile_Check(profile, &check) == 0U)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_PROFILE_VERIFY);
    return 0U;
  }
  return 1U;
}

void ImuBmi270_Init(void)
{
  imu_state = (imu_bmi270_state_t){0};
  imu_state.enabled = 1U;
  imu_state.profile = (uint8_t)imu_selected_profile;
  imu_state.init_state = IMU_BMI270_INIT_STATE_RESET;
  imu_state.quaternion[0] = 1.0f;
  imu_state.gyro_auto_cal_enabled = BMI270_GYRO_AUTO_CAL_ENABLED;
  imu_state.gyro_auto_cal_state = (BMI270_GYRO_AUTO_CAL_ENABLED != 0U) ?
    IMU_BMI270_GYRO_AUTO_CAL_WAIT : IMU_BMI270_GYRO_AUTO_CAL_DISABLED;
  imu_next_init_retry_ms = 0U;
  imu_gyro_auto_cal_next_ms = 0U;
  imu_gyro_calibration_active = 0U;
  (void)ImuBmi270Calibration_Load(&imu_calibration);
  imu_fusion_params = ImuBmi270Mahony_DefaultParams();
  ImuBmi270Mahony_Init(&imu_fusion);
  ImuBmi270_CsHigh();
}

uint8_t ImuBmi270_SetEnabled(uint8_t enabled)
{
  imu_state.enabled = (enabled != 0U) ? 1U : 0U;
  imu_state.init_state = (enabled != 0U) ? imu_state.init_state : IMU_BMI270_INIT_STATE_DISABLED;
  if (enabled == 0U)
  {
    imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_DISABLED;
  }
  else if (imu_state.gyro_auto_cal_enabled != 0U && imu_state.gyro_calibrated == 0U)
  {
    ImuBmi270_ScheduleAutoCal(HAL_GetTick(), BMI270_GYRO_AUTO_CAL_START_DELAY_MS);
  }
  return 1U;
}

uint8_t ImuBmi270_SetProfile(imu_bmi270_profile_id_t profile)
{
  if (ImuBmi270Profile_Get(profile) == 0)
  {
    return 0U;
  }
  imu_selected_profile = profile;
  imu_state.profile = (uint8_t)profile;
  imu_state.online = 0U;
  imu_state.filter_initialized = 0U;
  imu_state.gyro_calibrated = 0U;
  imu_state.gyro_auto_cal_attempts = 0U;
  ImuBmi270_ScheduleAutoCal(HAL_GetTick(), BMI270_GYRO_AUTO_CAL_START_DELAY_MS);
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
  const imu_bmi270_profile_t *profile = ImuBmi270Profile_Get(imu_selected_profile);

  imu_state.online = 0U;
  imu_state.sensor_time_valid = 0U;
  imu_state.init_state = IMU_BMI270_INIT_STATE_PROBE;
  if (ImuBmi270_WriteReg(BMI270_REG_CMD, BMI270_CMD_SOFT_RESET) == 0U)
  {
    return 0U;
  }
  HAL_Delay(5U);
  if (ImuBmi270_ProbeNow() == 0U)
  {
    return 0U;
  }
  imu_state.online = 0U;

  if (ImuBmi270_WriteReg(BMI270_REG_PWR_CONF, profile->pwr_conf) == 0U)
  {
    return 0U;
  }
  HAL_Delay(BMI270_SPI_SELECT_DELAY_MS);

  imu_state.init_state = IMU_BMI270_INIT_STATE_LOAD_CONFIG;
  if (ImuBmi270_LoadConfigFile() == 0U)
  {
    return 0U;
  }
  if (ImuBmi270_WaitInitOk() == 0U)
  {
    return 0U;
  }
  imu_state.init_state = IMU_BMI270_INIT_STATE_VERIFY_PROFILE;
  if (ImuBmi270_ApplyProfile(profile) == 0U)
  {
    return 0U;
  }
  imu_state.enabled = 1U;
  imu_state.online = 1U;
  imu_state.filter_initialized = 0U;
  imu_state.profile = (uint8_t)imu_selected_profile;
  imu_state.init_state = IMU_BMI270_INIT_STATE_SAMPLING;
  ImuBmi270Mahony_Init(&imu_fusion);
  if (imu_state.gyro_calibrated == 0U)
  {
    imu_state.gyro_auto_cal_attempts = 0U;
    ImuBmi270_ScheduleAutoCal(HAL_GetTick(), BMI270_GYRO_AUTO_CAL_START_DELAY_MS);
  }
  ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
  return 1U;
}

static void ImuBmi270_ProcessMeasurement(const int16_t accel_raw[3],
                                          const int16_t gyro_raw[3],
                                          uint32_t sensor_time,
                                          uint8_t sensor_time_valid,
                                          float fallback_dt_s)
{
  uint32_t primask;
  uint32_t update_ms = HAL_GetTick();
  float sensor_accel_g[3];
  float body_accel_g[3];
  float ros_accel_g[3];
  float gyro_raw_dps[3];
  float gyro_corrected_dps[3];
  float body_gyro_dps[3];
  float ros_gyro_dps[3];
  float euler_deg[3];
  float dt_s = fallback_dt_s;
  uint8_t dt_valid = 0U;
  float accel_norm;

  primask = __get_PRIMASK();
  __disable_irq();
  ImuBmi270_ClearTransientQuality();
  __set_PRIMASK(primask);

  for (uint8_t i = 0U; i < 3U; ++i)
  {
    sensor_accel_g[i] = (((float)accel_raw[i] / BMI270_ACCEL_LSB_PER_G) -
                         imu_calibration.accel_bias_g[i]) * imu_calibration.accel_scale[i];
    gyro_raw_dps[i] = (float)gyro_raw[i] / BMI270_GYRO_LSB_PER_DPS;
    gyro_corrected_dps[i] = gyro_raw_dps[i] -
                            imu_state.gyro_bias_dps[i] -
                            imu_calibration.gyro_bias_dps[i];
    if (ImuBmi270_AbsFloat(gyro_raw_dps[i]) >= BMI270_GYRO_SATURATION_DPS)
    {
      ImuBmi270_SetQuality(IMU_BMI270_QUALITY_GYRO_SATURATION);
    }
  }

  ImuBmi270Coordinate_Apply(imu_calibration.sensor_to_body, sensor_accel_g, body_accel_g);
  ImuBmi270Coordinate_Apply(imu_calibration.sensor_to_body, gyro_corrected_dps, body_gyro_dps);
  ImuBmi270Coordinate_BodyToRos(body_accel_g, ros_accel_g);
  ImuBmi270Coordinate_BodyToRos(body_gyro_dps, ros_gyro_dps);

  accel_norm = sqrtf((body_accel_g[0] * body_accel_g[0]) +
                     (body_accel_g[1] * body_accel_g[1]) +
                     (body_accel_g[2] * body_accel_g[2]));
  if (accel_norm < BMI270_ACCEL_REJECT_MIN_G || accel_norm > BMI270_ACCEL_REJECT_MAX_G)
  {
    ImuBmi270_SetQuality(IMU_BMI270_QUALITY_ACCEL_ANOMALY);
  }

  if (sensor_time_valid != 0U && imu_state.sensor_time_valid != 0U)
  {
    if (ImuBmi270Time_DeltaSeconds(sensor_time, imu_state.sensor_time, BMI270_ATTITUDE_MAX_DT_S, &dt_s) != 0U)
    {
      dt_valid = 1U;
    }
    else
    {
      ImuBmi270_SetError(IMU_BMI270_ERROR_TIMESTAMP);
      ImuBmi270_SetQuality(IMU_BMI270_QUALITY_ATTITUDE_INVALID);
    }
  }
  else if (fallback_dt_s > 0.0f)
  {
    dt_valid = 1U;
    ImuBmi270_SetQuality(IMU_BMI270_QUALITY_POLL_FALLBACK);
  }

  if (dt_valid != 0U)
  {
    ImuBmi270Mahony_Update(&imu_fusion, body_gyro_dps, body_accel_g, dt_s, &imu_fusion_params);
    if ((imu_fusion.status_flags & IMU_BMI270_FUSION_ACCEL_DEGRADED) != 0UL)
    {
      ImuBmi270_SetQuality(IMU_BMI270_QUALITY_ACCEL_ANOMALY);
    }
    if ((imu_fusion.status_flags & IMU_BMI270_FUSION_INVALID_DT) != 0UL)
    {
      ImuBmi270_SetQuality(IMU_BMI270_QUALITY_ATTITUDE_INVALID);
    }
  }
  ImuBmi270Quaternion_ToEulerDeg(&imu_fusion.q, euler_deg);

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
      imu_state.accel_g[i] = sensor_accel_g[i];
      imu_state.body_accel_g[i] = body_accel_g[i];
      imu_state.ros_accel_g[i] = ros_accel_g[i];
      imu_state.gyro_filtered_dps[i] = body_gyro_dps[i];
      imu_state.gyro_dps[i] = body_gyro_dps[i];
      imu_state.body_gyro_dps[i] = body_gyro_dps[i];
      imu_state.ros_gyro_dps[i] = ros_gyro_dps[i];
    }
    imu_state.filter_initialized = 1U;
  }
  else
  {
    for (uint8_t i = 0U; i < 3U; ++i)
    {
      imu_state.accel_g[i] = ImuBmi270_Filter(imu_state.accel_g[i], sensor_accel_g[i], BMI270_ACCEL_FILTER_ALPHA);
      imu_state.body_accel_g[i] = ImuBmi270_Filter(imu_state.body_accel_g[i], body_accel_g[i], BMI270_ACCEL_FILTER_ALPHA);
      imu_state.ros_accel_g[i] = ImuBmi270_Filter(imu_state.ros_accel_g[i], ros_accel_g[i], BMI270_ACCEL_FILTER_ALPHA);
      imu_state.gyro_filtered_dps[i] = ImuBmi270_Filter(imu_state.gyro_filtered_dps[i],
                                                        body_gyro_dps[i],
                                                        BMI270_GYRO_FILTER_ALPHA);
      imu_state.gyro_dps[i] = imu_state.gyro_filtered_dps[i];
      imu_state.body_gyro_dps[i] = imu_state.gyro_filtered_dps[i];
      imu_state.ros_gyro_dps[i] = ImuBmi270_Filter(imu_state.ros_gyro_dps[i], ros_gyro_dps[i], BMI270_GYRO_FILTER_ALPHA);
    }
  }
  imu_state.quaternion[0] = imu_fusion.q.w;
  imu_state.quaternion[1] = imu_fusion.q.x;
  imu_state.quaternion[2] = imu_fusion.q.y;
  imu_state.quaternion[3] = imu_fusion.q.z;
  imu_state.roll_deg = euler_deg[0];
  imu_state.pitch_deg = euler_deg[1];
  imu_state.yaw_deg = ImuBmi270_WrapAngleDeg(euler_deg[2]);
  imu_state.accel_correction_weight = imu_fusion.accel_weight;
  imu_state.sensor_time = sensor_time & IMU_BMI270_SENSOR_TIME_MASK;
  imu_state.sensor_time_valid = sensor_time_valid;
  imu_state.sample_count++;
  imu_state.last_update_ms = update_ms;
  imu_state.online = 1U;
  imu_state.init_state = IMU_BMI270_INIT_STATE_SAMPLING;
  __set_PRIMASK(primask);
}

static uint8_t ImuBmi270_UpdateDirect(float fallback_dt_s)
{
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  uint32_t sensor_time = 0UL;
  uint8_t sensor_time_valid = 0U;

  if (ImuBmi270_ReadRawFrame(accel_raw, gyro_raw) == 0U)
  {
    return 0U;
  }
  sensor_time_valid = ImuBmi270_ReadSensorTime(&sensor_time);
  ImuBmi270_ProcessMeasurement(accel_raw, gyro_raw, sensor_time, sensor_time_valid, fallback_dt_s);
  return 1U;
}

static uint8_t ImuBmi270_UpdateFifo(void)
{
  uint16_t fifo_len = 0U;
  uint8_t fifo[BMI270_FIFO_READ_MAX_BYTES];
  imu_bmi270_fifo_sample_t samples[BMI270_FIFO_MAX_SAMPLES];
  imu_bmi270_fifo_parse_result_t parse;
  uint32_t last_time;
  uint8_t any_processed = 0U;

  if (ImuBmi270_ReadFifoLength(&fifo_len) == 0U)
  {
    return 0U;
  }
  if (fifo_len == 0U)
  {
    return 0U;
  }
  if (fifo_len > BMI270_FIFO_READ_MAX_BYTES)
  {
    fifo_len = BMI270_FIFO_READ_MAX_BYTES;
    ImuBmi270_SetError(IMU_BMI270_ERROR_FIFO);
  }
  if (ImuBmi270_ReadBytes(BMI270_REG_FIFO_DATA, fifo, (uint8_t)fifo_len) == 0U)
  {
    return 0U;
  }
  if (ImuBmi270Fifo_Parse(fifo, fifo_len, samples, BMI270_FIFO_MAX_SAMPLES, &parse) == 0U)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_FIFO);
    return 0U;
  }
  if ((parse.flags & IMU_BMI270_FIFO_PARSE_SKIP_FRAME) != 0UL)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_FIFO);
  }

  last_time = parse.sensor_time & IMU_BMI270_SENSOR_TIME_MASK;
  for (uint32_t i = 0UL; i < parse.sample_count && i < BMI270_FIFO_MAX_SAMPLES; ++i)
  {
    uint32_t sample_time = last_time;
    uint8_t sample_time_valid = parse.sensor_time_valid;

    if (parse.sensor_time_valid != 0U && parse.sample_count > 0UL)
    {
      uint32_t remaining = (parse.sample_count - 1UL) - i;
      sample_time = (last_time - (remaining * IMU_BMI270_SENSOR_TIME_100HZ_TICKS)) &
                    IMU_BMI270_SENSOR_TIME_MASK;
    }
    if (samples[i].accel_valid != 0U && samples[i].gyro_valid != 0U)
    {
      if (ImuBmi270_RawFrameHasSignal(samples[i].accel_raw, samples[i].gyro_raw) != 0U)
      {
        ImuBmi270_ProcessMeasurement(samples[i].accel_raw, samples[i].gyro_raw,
                                     sample_time, sample_time_valid, 0.0f);
        any_processed = 1U;
      }
      else
      {
        ImuBmi270_SetError(IMU_BMI270_ERROR_INVALID_FRAME);
      }
    }
  }

  return any_processed;
}

uint8_t ImuBmi270_Update(void)
{
  uint32_t now_ms = HAL_GetTick();

  if (imu_gyro_calibration_active != 0U)
  {
    return 1U;
  }
  if (imu_state.enabled == 0U)
  {
    return 1U;
  }
  if (imu_state.online == 0U)
  {
    if ((int32_t)(now_ms - imu_next_init_retry_ms) < 0)
    {
      imu_state.init_state = IMU_BMI270_INIT_STATE_RETRY_WAIT;
      return 0U;
    }
    if (ImuBmi270_ConfigNow() == 0U)
    {
      imu_next_init_retry_ms = now_ms + BMI270_INIT_RETRY_MS;
      imu_state.init_state = IMU_BMI270_INIT_STATE_RETRY_WAIT;
      return 0U;
    }
  }

  ImuBmi270_ServiceAutoCal(HAL_GetTick());

  if (ImuBmi270_UpdateFifo() != 0U)
  {
    ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
    return 1U;
  }

  if (ImuBmi270_UpdateDirect(BMI270_DIRECT_FALLBACK_DT_S) == 0U)
  {
    return 0U;
  }

  ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
  return 1U;
}

void ImuBmi270_OnDataReadyFromIsr(void)
{
  imu_state.drdy_count++;
}

uint8_t ImuBmi270_CalibrateGyro(uint16_t samples, uint16_t delay_ms)
{
  int16_t accel_raw[3];
  int16_t gyro_raw[3];
  float sum[3] = {0.0f, 0.0f, 0.0f};
  float accel_sum_g[3] = {0.0f, 0.0f, 0.0f};
  float min_dps[3] = {0.0f, 0.0f, 0.0f};
  float max_dps[3] = {0.0f, 0.0f, 0.0f};
  imu_bmi270_quaternion_t calibrated_q;
  uint8_t calibrated_q_valid = 0U;
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

  imu_gyro_calibration_active = 1U;
  HAL_Delay(BMI270_GYRO_CAL_SETTLE_MS);
  ImuBmi270_UpdateGyroCalDiag(IMU_BMI270_GYRO_CAL_FAIL_NONE, 0xFFU, 0, 0, 0, 0U);

  if (imu_state.online == 0U && ImuBmi270_ConfigNow() == 0U)
  {
    ImuBmi270_UpdateGyroCalDiag(IMU_BMI270_GYRO_CAL_FAIL_CONFIG, 0xFFU, 0, 0, 0, 0U);
    imu_gyro_calibration_active = 0U;
    return 0U;
  }

  for (uint16_t sample = 0U; sample < samples; ++sample)
  {
    uint8_t fail_axis = 0xFFU;

    if (ImuBmi270_ReadRawFrame(accel_raw, gyro_raw) == 0U)
    {
      ImuBmi270_UpdateGyroCalDiag(IMU_BMI270_GYRO_CAL_FAIL_READ,
                                  0xFFU,
                                  sum,
                                  min_dps,
                                  max_dps,
                                  sample);
      imu_gyro_calibration_active = 0U;
      return 0U;
    }

    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
      float dps = (float)gyro_raw[axis] / BMI270_GYRO_LSB_PER_DPS;
      float accel_g = (((float)accel_raw[axis] / BMI270_ACCEL_LSB_PER_G) -
                       imu_calibration.accel_bias_g[axis]) *
                      imu_calibration.accel_scale[axis];

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
      sum[axis] += dps;
      if (fail_axis == 0xFFU && ImuBmi270_AbsFloat(dps) > BMI270_GYRO_CAL_MAX_ABS_DPS)
      {
        fail_axis = axis;
      }
      accel_sum_g[axis] += accel_g;
    }

    if (fail_axis != 0xFFU)
    {
      ImuBmi270_UpdateGyroCalDiag(IMU_BMI270_GYRO_CAL_FAIL_ABS,
                                  fail_axis,
                                  sum,
                                  min_dps,
                                  max_dps,
                                  (uint16_t)(sample + 1U));
      imu_gyro_calibration_active = 0U;
      return 0U;
    }

    HAL_Delay(delay_ms);
  }

  ImuBmi270_UpdateGyroCalDiag(IMU_BMI270_GYRO_CAL_FAIL_NONE,
                              0xFFU,
                              sum,
                              min_dps,
                              max_dps,
                              samples);
  {
    uint8_t span_axis = 0xFFU;

    if (ImuBmi270_GyroCalSpanWithinLimit(min_dps,
                                         max_dps,
                                         BMI270_GYRO_CAL_STILL_SPAN_DPS,
                                         &span_axis) == 0U)
    {
      ImuBmi270_UpdateGyroCalDiag(IMU_BMI270_GYRO_CAL_FAIL_SPAN,
                                  span_axis,
                                  sum,
                                  min_dps,
                                  max_dps,
                                  samples);
      imu_gyro_calibration_active = 0U;
      return 0U;
    }
  }

  {
    float sensor_accel_g[3];
    float body_accel_g[3];

    for (uint8_t i = 0U; i < 3U; ++i)
    {
      sensor_accel_g[i] = accel_sum_g[i] / (float)samples;
    }
    ImuBmi270Coordinate_Apply(imu_calibration.sensor_to_body, sensor_accel_g, body_accel_g);
    calibrated_q_valid = ImuBmi270Quaternion_FromAccel(body_accel_g, &calibrated_q);
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
  imu_state.gyro_auto_cal_last_result = 1U;
  if (imu_state.gyro_auto_cal_enabled != 0U)
  {
    imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_DONE;
  }
  if (calibrated_q_valid != 0U)
  {
    imu_fusion.q = calibrated_q;
    imu_fusion.integral[0] = 0.0f;
    imu_fusion.integral[1] = 0.0f;
    imu_fusion.integral[2] = 0.0f;
    imu_fusion.accel_weight = 1.0f;
    imu_fusion.status_flags = 0UL;
    imu_fusion.initialized = 1U;
  }
  else
  {
    ImuBmi270Mahony_Init(&imu_fusion);
  }
  __set_PRIMASK(primask);

  ImuBmi270_SetError(IMU_BMI270_ERROR_NONE);
  imu_gyro_calibration_active = 0U;
  return 1U;
}

static void ImuBmi270_ServiceAutoCal(uint32_t now_ms)
{
  if (imu_state.gyro_calibrated != 0U)
  {
    if (imu_state.gyro_auto_cal_enabled != 0U)
    {
      imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_DONE;
    }
    return;
  }

  if (ImuBmi270_AutoCalDue(imu_state.gyro_auto_cal_enabled,
                           imu_state.online,
                           imu_state.gyro_calibrated,
                           imu_state.gyro_auto_cal_attempts,
                           BMI270_GYRO_AUTO_CAL_MAX_ATTEMPTS,
                           now_ms,
                           imu_gyro_auto_cal_next_ms) == 0U)
  {
    return;
  }

  imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_RUNNING;
  imu_state.gyro_auto_cal_attempts++;
  imu_state.gyro_auto_cal_last_result = 0U;
  if (ImuBmi270_CalibrateGyro(BMI270_GYRO_AUTO_CAL_SAMPLES,
                              BMI270_GYRO_AUTO_CAL_DELAY_MS) != 0U)
  {
    imu_state.gyro_auto_cal_last_result = 1U;
    imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_DONE;
    return;
  }

  imu_state.gyro_auto_cal_last_result = 0U;
  if (imu_state.gyro_auto_cal_attempts >= BMI270_GYRO_AUTO_CAL_MAX_ATTEMPTS)
  {
    imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_FAILED;
  }
  else
  {
    ImuBmi270_ScheduleAutoCal(HAL_GetTick(), BMI270_GYRO_AUTO_CAL_RETRY_MS);
    imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_RETRY_WAIT;
  }
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
  imu_state.gyro_auto_cal_attempts = 0U;
  imu_state.gyro_auto_cal_last_result = 0U;
  __set_PRIMASK(primask);
  ImuBmi270_ScheduleAutoCal(HAL_GetTick(), BMI270_GYRO_AUTO_CAL_START_DELAY_MS);
}

void ImuBmi270_ApplyGyroBias(const float bias_dps[3])
{
  uint32_t primask;

  if (bias_dps == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  for (uint8_t i = 0U; i < 3U; ++i)
  {
    imu_state.gyro_bias_dps[i] = bias_dps[i];
    imu_state.gyro_corrected_dps[i] = 0.0f;
    imu_state.gyro_filtered_dps[i] = 0.0f;
    imu_state.gyro_dps[i] = 0.0f;
  }
  imu_state.gyro_calibrated = 1U;
  imu_state.filter_initialized = 0U;
  imu_state.gyro_auto_cal_last_result = 1U;
  if (imu_state.gyro_auto_cal_enabled != 0U)
  {
    imu_state.gyro_auto_cal_state = IMU_BMI270_GYRO_AUTO_CAL_DONE;
  }
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
