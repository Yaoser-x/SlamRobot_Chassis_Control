#include "bmi270_driver.h"

#include "bmi270_device.h"
#include "bmi270_registers.h"
#include "bmi270_fifo_reader.h"
#include "main.h"

#define BMI270_INIT_RETRY_MS       1000U
#define BMI270_FIFO_READ_MAX_BYTES 128U
#define BMI270_ACCEL_LSB_PER_G     16384.0f
#define BMI270_GYRO_LSB_PER_DPS    65.6f

static bmi270_driver_state_t   driver_state;
static bmi270_sample_t         sample_queue[BMI270_DRIVER_QUEUE_CAPACITY];
static uint8_t                 queue_read;
static uint8_t                 queue_write;
static uint8_t                 queue_count;
static uint32_t                next_init_retry_ms;
static imu_bmi270_profile_id_t selected_profile = IMU_BMI270_PROFILE_PERFORMANCE;

static int16_t Bmi270Driver_ReadI16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static uint8_t Bmi270Driver_RawFrameHasSignal(const int16_t accel_raw[3], const int16_t gyro_raw[3])
{
    if (accel_raw == 0 || gyro_raw == 0)
    {
        return 0U;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        if (accel_raw[axis] != 0 || gyro_raw[axis] != 0)
        {
            return 1U;
        }
    }
    return 0U;
}

static void Bmi270Driver_SetError(bmi270_driver_error_t error)
{
    driver_state.last_error = (uint8_t)error;
    if (error == BMI270_DRIVER_ERROR_NONE)
    {
        return;
    }
    driver_state.error_count++;
    if (error == BMI270_DRIVER_ERROR_SPI)
    {
        driver_state.spi_error_count++;
    }
    else if (error == BMI270_DRIVER_ERROR_CONFIG || error == BMI270_DRIVER_ERROR_CHIP_ID
             || error == BMI270_DRIVER_ERROR_PROFILE_VERIFY)
    {
        driver_state.init_failure_count++;
    }
    else if (error == BMI270_DRIVER_ERROR_FIFO)
    {
        driver_state.fifo_error_count++;
    }
}

static uint8_t Bmi270Driver_ReadReg(uint8_t reg, uint8_t *value)
{
    if (Bmi270Bus_ReadReg(reg, value) == 0U)
    {
        Bmi270Driver_SetError(BMI270_DRIVER_ERROR_SPI);
        return 0U;
    }
    return 1U;
}

static uint8_t Bmi270Driver_WriteReg(uint8_t reg, uint8_t value)
{
    if (Bmi270Bus_WriteReg(reg, value) == 0U)
    {
        Bmi270Driver_SetError(BMI270_DRIVER_ERROR_SPI);
        return 0U;
    }
    return 1U;
}

static uint8_t Bmi270Driver_ReadBytes(uint8_t reg, uint8_t *data, uint8_t length)
{
    if (Bmi270Bus_ReadBytes(reg, data, length) == 0U)
    {
        Bmi270Driver_SetError(BMI270_DRIVER_ERROR_SPI);
        return 0U;
    }
    return 1U;
}

static uint8_t Bmi270Driver_WriteBytes(uint8_t reg, const uint8_t *data, uint8_t length)
{
    if (Bmi270Bus_WriteBytes(reg, data, length) == 0U)
    {
        Bmi270Driver_SetError(BMI270_DRIVER_ERROR_SPI);
        return 0U;
    }
    return 1U;
}

static const bmi270_device_io_t device_io = {
    .read_reg    = Bmi270Driver_ReadReg,
    .write_reg   = Bmi270Driver_WriteReg,
    .write_bytes = Bmi270Driver_WriteBytes,
};

static uint8_t Bmi270Driver_Enqueue(const int16_t accel_raw[3],
                                    const int16_t gyro_raw[3],
                                    uint32_t      sensor_time,
                                    uint8_t       sensor_time_valid)
{
    bmi270_sample_t *sample;

    if (Bmi270Driver_RawFrameHasSignal(accel_raw, gyro_raw) == 0U)
    {
        Bmi270Driver_SetError(BMI270_DRIVER_ERROR_INVALID_FRAME);
        return 0U;
    }
    if (queue_count == BMI270_DRIVER_QUEUE_CAPACITY)
    {
        queue_read = (uint8_t)((queue_read + 1U) % BMI270_DRIVER_QUEUE_CAPACITY);
        queue_count--;
    }
    sample  = &sample_queue[queue_write];
    *sample = (bmi270_sample_t){0};
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        sample->accel_raw[axis] = accel_raw[axis];
        sample->gyro_raw[axis]  = gyro_raw[axis];
        sample->accel_g[axis]   = (float)accel_raw[axis] / BMI270_ACCEL_LSB_PER_G;
        sample->gyro_dps[axis]  = (float)gyro_raw[axis] / BMI270_GYRO_LSB_PER_DPS;
    }
    sample->sensor_time       = sensor_time & 0x00FFFFFFUL;
    sample->sensor_time_valid = sensor_time_valid;
    sample->timestamp_ms      = HAL_GetTick();
    queue_write               = (uint8_t)((queue_write + 1U) % BMI270_DRIVER_QUEUE_CAPACITY);
    queue_count++;
    driver_state.sample_count++;
    driver_state.last_update_ms = sample->timestamp_ms;
    return 1U;
}

void Bmi270Driver_Init(void)
{
    driver_state            = (bmi270_driver_state_t){0};
    driver_state.enabled    = 1U;
    driver_state.profile    = (uint8_t)selected_profile;
    driver_state.init_state = BMI270_DRIVER_INIT_RESET;
    queue_read              = 0U;
    queue_write             = 0U;
    queue_count             = 0U;
    next_init_retry_ms      = 0U;
    Bmi270Bus_Deselect();
}

uint8_t Bmi270Driver_SetEnabled(uint8_t enabled)
{
    driver_state.enabled = (enabled != 0U) ? 1U : 0U;
    if (enabled == 0U)
    {
        driver_state.init_state = BMI270_DRIVER_INIT_DISABLED;
    }
    return 1U;
}

uint8_t Bmi270Driver_SetProfile(imu_bmi270_profile_id_t profile)
{
    if (ImuBmi270Profile_Get(profile) == 0)
    {
        return 0U;
    }
    selected_profile        = profile;
    driver_state.profile    = (uint8_t)profile;
    driver_state.online     = 0U;
    driver_state.init_state = BMI270_DRIVER_INIT_RESET;
    return 1U;
}

uint8_t Bmi270Driver_ProbeNow(void)
{
    uint8_t chip_id = 0U;

    (void)Bmi270Driver_ReadReg(BMI270_REG_CHIP_ID, &chip_id);
    HAL_Delay(1U);
    if (Bmi270Driver_ReadReg(BMI270_REG_CHIP_ID, &chip_id) == 0U)
    {
        return 0U;
    }
    driver_state.chip_id = chip_id;
    if (chip_id != BMI270_CHIP_ID)
    {
        Bmi270Driver_SetError(BMI270_DRIVER_ERROR_CHIP_ID);
        return 0U;
    }
    Bmi270Driver_SetError(BMI270_DRIVER_ERROR_NONE);
    return 1U;
}

uint8_t Bmi270Driver_ConfigNow(void)
{
    const imu_bmi270_profile_t *profile = ImuBmi270Profile_Get(selected_profile);
    bmi270_device_status_t      status;

    driver_state.online     = 0U;
    driver_state.init_state = BMI270_DRIVER_INIT_PROBE;
    if (profile == 0 || Bmi270Driver_WriteReg(BMI270_REG_CMD, BMI270_CMD_SOFT_RESET) == 0U)
    {
        return 0U;
    }
    HAL_Delay(5U);
    if (Bmi270Driver_ProbeNow() == 0U || Bmi270Driver_WriteReg(BMI270_REG_PWR_CONF, profile->pwr_conf) == 0U)
    {
        return 0U;
    }
    driver_state.init_state = BMI270_DRIVER_INIT_LOAD_CONFIG;
    status                  = Bmi270Device_LoadConfig(&device_io);
    if (status != BMI270_DEVICE_OK || Bmi270Device_WaitInitOk(&device_io) != BMI270_DEVICE_OK)
    {
        Bmi270Driver_SetError(BMI270_DRIVER_ERROR_CONFIG);
        return 0U;
    }
    driver_state.init_state = BMI270_DRIVER_INIT_VERIFY_PROFILE;
    status                  = Bmi270Device_ApplyProfile(&device_io, profile);
    if (status != BMI270_DEVICE_OK)
    {
        Bmi270Driver_SetError((status == BMI270_DEVICE_PROFILE_MISMATCH) ? BMI270_DRIVER_ERROR_PROFILE_VERIFY
                                                                         : BMI270_DRIVER_ERROR_CONFIG);
        return 0U;
    }
    driver_state.online     = 1U;
    driver_state.init_state = BMI270_DRIVER_INIT_SAMPLING;
    Bmi270Driver_SetError(BMI270_DRIVER_ERROR_NONE);
    return 1U;
}

static uint8_t Bmi270Driver_UpdateFifo(void)
{
    uint8_t                        data[2];
    uint16_t                       length;
    uint8_t                        fifo[BMI270_FIFO_READ_MAX_BYTES];
    imu_bmi270_fifo_sample_t       samples[BMI270_DRIVER_QUEUE_CAPACITY];
    imu_bmi270_fifo_parse_result_t parsed;
    uint8_t                        any = 0U;

    if (Bmi270Driver_ReadBytes(BMI270_REG_FIFO_LENGTH_0, data, sizeof(data)) == 0U)
    {
        return 0U;
    }
    length = (uint16_t)((((uint16_t)data[1] & 0x3FU) << 8) | data[0]);
    if (length == 0U)
    {
        return 0U;
    }
    if (length > BMI270_FIFO_READ_MAX_BYTES)
    {
        length = BMI270_FIFO_READ_MAX_BYTES;
        Bmi270Driver_SetError(BMI270_DRIVER_ERROR_FIFO);
    }
    if (Bmi270Driver_ReadBytes(BMI270_REG_FIFO_DATA, fifo, (uint8_t)length) == 0U
        || ImuBmi270Fifo_Parse(fifo, length, samples, BMI270_DRIVER_QUEUE_CAPACITY, &parsed) == 0U)
    {
        Bmi270Driver_SetError(BMI270_DRIVER_ERROR_FIFO);
        return 0U;
    }
    for (uint32_t index = 0U; index < parsed.sample_count; ++index)
    {
        uint32_t sample_time = parsed.sensor_time;
        if (parsed.sensor_time_valid != 0U)
        {
            sample_time = (parsed.sensor_time - ((parsed.sample_count - 1U - index) * 256U)) & 0x00FFFFFFUL;
        }
        if (samples[index].accel_valid != 0U && samples[index].gyro_valid != 0U)
        {
            any |= Bmi270Driver_Enqueue(samples[index].accel_raw,
                                        samples[index].gyro_raw,
                                        sample_time,
                                        parsed.sensor_time_valid);
        }
    }
    return any;
}

static uint8_t Bmi270Driver_UpdateDirect(void)
{
    uint8_t  data[12];
    uint8_t  time_data[3];
    int16_t  accel_raw[3];
    int16_t  gyro_raw[3];
    uint32_t sensor_time = 0U;
    uint8_t  sensor_time_valid;

    if (Bmi270Driver_ReadBytes(BMI270_REG_DATA_8, data, sizeof(data)) == 0U)
    {
        Bmi270Driver_SetError(BMI270_DRIVER_ERROR_READ);
        return 0U;
    }
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
        accel_raw[axis] = Bmi270Driver_ReadI16(&data[axis * 2U]);
        gyro_raw[axis]  = Bmi270Driver_ReadI16(&data[6U + axis * 2U]);
    }
    sensor_time_valid = Bmi270Driver_ReadBytes(BMI270_REG_SENSORTIME_0, time_data, sizeof(time_data));
    if (sensor_time_valid != 0U)
    {
        sensor_time = (uint32_t)time_data[0] | ((uint32_t)time_data[1] << 8) | ((uint32_t)time_data[2] << 16);
    }
    driver_state.poll_fallback_count++;
    return Bmi270Driver_Enqueue(accel_raw, gyro_raw, sensor_time, sensor_time_valid);
}

static void Bmi270Driver_UpdateTemperature(void)
{
    uint8_t data[2];
    int16_t raw;

    if (Bmi270Driver_ReadBytes(BMI270_REG_TEMP_0, data, sizeof(data)) == 0U)
    {
        driver_state.temperature_valid = 0U;
        return;
    }
    raw = Bmi270Driver_ReadI16(data);
    if (raw == INT16_MIN)
    {
        driver_state.temperature_valid = 0U;
        return;
    }
    driver_state.temperature_c     = ((float)raw / 512.0f) + 23.0f;
    driver_state.temperature_valid = 1U;
}

uint8_t Bmi270Driver_Update(void)
{
    const uint32_t now_ms = HAL_GetTick();
    uint8_t        updated;

    if (driver_state.enabled == 0U)
    {
        return 1U;
    }
    if (driver_state.online == 0U)
    {
        if ((int32_t)(now_ms - next_init_retry_ms) < 0)
        {
            driver_state.init_state = BMI270_DRIVER_INIT_RETRY_WAIT;
            return 0U;
        }
        if (Bmi270Driver_ConfigNow() == 0U)
        {
            next_init_retry_ms      = now_ms + BMI270_INIT_RETRY_MS;
            driver_state.init_state = BMI270_DRIVER_INIT_RETRY_WAIT;
            return 0U;
        }
    }
    updated = Bmi270Driver_UpdateFifo();
    if (updated == 0U)
    {
        updated = Bmi270Driver_UpdateDirect();
    }
    if (updated != 0U)
    {
        Bmi270Driver_UpdateTemperature();
        Bmi270Driver_SetError(BMI270_DRIVER_ERROR_NONE);
    }
    return updated;
}

uint8_t Bmi270Driver_TakeSample(bmi270_sample_t *sample)
{
    if (sample == 0 || queue_count == 0U)
    {
        return 0U;
    }
    *sample    = sample_queue[queue_read];
    queue_read = (uint8_t)((queue_read + 1U) % BMI270_DRIVER_QUEUE_CAPACITY);
    queue_count--;
    return 1U;
}

void Bmi270Driver_OnDataReadyFromIsr(void)
{
    driver_state.drdy_count++;
}

uint8_t Bmi270Driver_Diagnose(imu_bmi270_diag_t *diag)
{
    return Bmi270Bus_RunRecoveryProbe(BMI270_REG_CHIP_ID, diag);
}

void Bmi270Driver_GetState(bmi270_driver_state_t *state)
{
    if (state != 0)
    {
        *state = driver_state;
    }
}
