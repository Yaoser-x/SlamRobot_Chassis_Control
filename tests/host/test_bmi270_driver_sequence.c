#include "bmi270_driver.h"
#include "bmi270_device.h"
#include "bmi270_fifo_reader.h"
#include "bmi270_registers.h"

#include <stdio.h>
#include <stdlib.h>

static uint8_t pwr_conf_written;
static uint8_t delay_after_pwr_conf;
static uint8_t config_loaded;

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

void HAL_Delay(uint32_t delay_ms)
{
    if (pwr_conf_written != 0U && delay_ms >= 1U)
    {
        delay_after_pwr_conf = 1U;
    }
}

uint32_t HAL_GetTick(void)
{
    return 0U;
}

void Bmi270Bus_Deselect(void)
{
}

uint8_t Bmi270Bus_ReadReg(uint8_t reg, uint8_t *value)
{
    if (value == 0)
    {
        return 0U;
    }
    *value = (reg == BMI270_REG_CHIP_ID) ? BMI270_CHIP_ID : 0U;
    return 1U;
}

uint8_t Bmi270Bus_ReadBytes(uint8_t reg, uint8_t *data, uint8_t len)
{
    (void)reg;
    if (data == 0)
    {
        return 0U;
    }
    for (uint8_t index = 0U; index < len; ++index)
    {
        data[index] = 0U;
    }
    return 1U;
}

uint8_t Bmi270Bus_WriteReg(uint8_t reg, uint8_t value)
{
    (void)value;
    if (reg == BMI270_REG_PWR_CONF)
    {
        pwr_conf_written = 1U;
    }
    return 1U;
}

uint8_t Bmi270Bus_WriteBytes(uint8_t reg, const uint8_t *data, uint8_t len)
{
    (void)reg;
    (void)data;
    (void)len;
    return 1U;
}

uint8_t Bmi270Bus_RunRecoveryProbe(uint8_t chip_id_reg, imu_bmi270_diag_t *diag)
{
    (void)chip_id_reg;
    (void)diag;
    return 1U;
}

bmi270_device_status_t Bmi270Device_LoadConfig(const bmi270_device_io_t *io)
{
    (void)io;
    require_int(pwr_conf_written != 0U, "PWR_CONF is written before config upload");
    require_int(delay_after_pwr_conf != 0U, "config upload waits after PWR_CONF");
    config_loaded = 1U;
    return BMI270_DEVICE_OK;
}

bmi270_device_status_t Bmi270Device_WaitInitOk(const bmi270_device_io_t *io)
{
    (void)io;
    return (config_loaded != 0U) ? BMI270_DEVICE_OK : BMI270_DEVICE_CONFIG_ERROR;
}

bmi270_device_status_t Bmi270Device_ApplyProfile(const bmi270_device_io_t *io, const imu_bmi270_profile_t *profile)
{
    (void)io;
    return (profile != 0) ? BMI270_DEVICE_OK : BMI270_DEVICE_IO_ERROR;
}

uint8_t ImuBmi270Fifo_Parse(const uint8_t                  *fifo,
                            uint16_t                        length,
                            imu_bmi270_fifo_sample_t       *samples,
                            uint16_t                        capacity,
                            imu_bmi270_fifo_parse_result_t *result)
{
    (void)fifo;
    (void)length;
    (void)samples;
    (void)capacity;
    (void)result;
    return 0U;
}

int main(void)
{
    Bmi270Driver_Init();
    require_int(Bmi270Driver_ConfigNow() != 0U, "driver configuration succeeds");
    require_int(config_loaded != 0U, "configuration upload is reached");
    (void)puts("BMI270 driver sequence tests passed");
    return 0;
}
