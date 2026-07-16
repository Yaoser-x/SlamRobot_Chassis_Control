#include "bmi270_device.h"

#include "bmi270_registers.h"
#include "bmi270_config.h"
#include "main.h"

#include <string.h>

#define BMI270_DEVICE_CONFIG_CHUNK_SIZE  32U
#define BMI270_DEVICE_INIT_POLL_DELAY_MS 1U
#define BMI270_DEVICE_INIT_TIMEOUT_MS    25U

static uint8_t Bmi270Device_IoValid(const bmi270_device_io_t *io)
{
    return (io != 0 && io->read_reg != 0 && io->write_reg != 0 && io->write_bytes != 0) ? 1U : 0U;
}

static uint8_t Bmi270Device_SetInitAddress(const bmi270_device_io_t *io, uint16_t byte_offset)
{
    uint16_t word_addr = (uint16_t)(byte_offset / 2U);

    return (io->write_reg(BMI270_REG_INIT_ADDR_0, (uint8_t)(word_addr & 0x0FU)) != 0U
            && io->write_reg(BMI270_REG_INIT_ADDR_1, (uint8_t)((word_addr >> 4U) & 0xFFU)) != 0U)
               ? 1U
               : 0U;
}

bmi270_device_status_t Bmi270Device_LoadConfig(const bmi270_device_io_t *io)
{
    uint32_t offset = 0U;

    if (Bmi270Device_IoValid(io) == 0U || io->write_reg(BMI270_REG_INIT_CTRL, BMI270_INIT_CTRL_PREPARE) == 0U)
    {
        return BMI270_DEVICE_IO_ERROR;
    }
    while (offset < bmi270_config_file_size)
    {
        uint32_t remaining = bmi270_config_file_size - offset;
        uint8_t  chunk_len =
            (remaining > BMI270_DEVICE_CONFIG_CHUNK_SIZE) ? BMI270_DEVICE_CONFIG_CHUNK_SIZE : (uint8_t)remaining;

        if ((chunk_len & 1U) != 0U)
        {
            return BMI270_DEVICE_CONFIG_ERROR;
        }
        if (Bmi270Device_SetInitAddress(io, (uint16_t)offset) == 0U
            || io->write_bytes(BMI270_REG_INIT_DATA, &bmi270_config_file[offset], chunk_len) == 0U)
        {
            return BMI270_DEVICE_IO_ERROR;
        }
        offset += chunk_len;
    }
    return (io->write_reg(BMI270_REG_INIT_CTRL, BMI270_INIT_CTRL_COMPLETE) != 0U) ? BMI270_DEVICE_OK
                                                                                  : BMI270_DEVICE_IO_ERROR;
}

bmi270_device_status_t Bmi270Device_WaitInitOk(const bmi270_device_io_t *io)
{
    uint8_t status = 0U;

    if (Bmi270Device_IoValid(io) == 0U)
    {
        return BMI270_DEVICE_IO_ERROR;
    }
    for (uint32_t elapsed_ms = 0U; elapsed_ms < BMI270_DEVICE_INIT_TIMEOUT_MS;
         elapsed_ms += BMI270_DEVICE_INIT_POLL_DELAY_MS)
    {
        HAL_Delay(BMI270_DEVICE_INIT_POLL_DELAY_MS);
        if (io->read_reg(BMI270_REG_INTERNAL_STATUS, &status) == 0U)
        {
            return BMI270_DEVICE_IO_ERROR;
        }
        if ((status & BMI270_INTERNAL_STATUS_MSG_MASK) == BMI270_INTERNAL_STATUS_INIT_OK)
        {
            return BMI270_DEVICE_OK;
        }
    }
    return BMI270_DEVICE_CONFIG_ERROR;
}

bmi270_device_status_t Bmi270Device_ApplyProfile(const bmi270_device_io_t *io, const imu_bmi270_profile_t *profile)
{
    imu_bmi270_profile_check_t check;

    if (Bmi270Device_IoValid(io) == 0U || profile == 0)
    {
        return BMI270_DEVICE_IO_ERROR;
    }
    if (io->write_reg(BMI270_REG_PWR_CONF, profile->pwr_conf) == 0U)
    {
        return BMI270_DEVICE_IO_ERROR;
    }
    HAL_Delay(1U);
    if (io->write_reg(BMI270_REG_ACC_CONF, profile->acc_conf) == 0U
        || io->write_reg(BMI270_REG_ACC_RANGE, profile->acc_range) == 0U
        || io->write_reg(BMI270_REG_GYR_CONF, profile->gyr_conf) == 0U
        || io->write_reg(BMI270_REG_GYR_RANGE, profile->gyr_range) == 0U
        || io->write_reg(BMI270_REG_FIFO_DOWNS, profile->fifo_downs) == 0U
        || io->write_reg(BMI270_REG_FIFO_WTM_0, profile->fifo_wtm_0) == 0U
        || io->write_reg(BMI270_REG_FIFO_WTM_1, profile->fifo_wtm_1) == 0U
        || io->write_reg(BMI270_REG_FIFO_CONFIG_0, profile->fifo_config_0) == 0U
        || io->write_reg(BMI270_REG_FIFO_CONFIG_1, profile->fifo_config_1) == 0U
        || io->write_reg(BMI270_REG_INT1_IO_CTRL, profile->int1_io_ctrl) == 0U
        || io->write_reg(BMI270_REG_INT_MAP_DATA, profile->int_map_data) == 0U
        || io->write_reg(BMI270_REG_PWR_CTRL, profile->pwr_ctrl) == 0U)
    {
        return BMI270_DEVICE_IO_ERROR;
    }
    HAL_Delay(2U);
    memset(&check, 0, sizeof(check));
    if (io->read_reg(BMI270_REG_ACC_CONF, &check.acc_conf) == 0U
        || io->read_reg(BMI270_REG_ACC_RANGE, &check.acc_range) == 0U
        || io->read_reg(BMI270_REG_GYR_CONF, &check.gyr_conf) == 0U
        || io->read_reg(BMI270_REG_GYR_RANGE, &check.gyr_range) == 0U
        || io->read_reg(BMI270_REG_PWR_CONF, &check.pwr_conf) == 0U
        || io->read_reg(BMI270_REG_PWR_CTRL, &check.pwr_ctrl) == 0U
        || io->read_reg(BMI270_REG_FIFO_CONFIG_0, &check.fifo_config_0) == 0U
        || io->read_reg(BMI270_REG_FIFO_CONFIG_1, &check.fifo_config_1) == 0U
        || io->read_reg(BMI270_REG_INT1_IO_CTRL, &check.int1_io_ctrl) == 0U
        || io->read_reg(BMI270_REG_INT_MAP_DATA, &check.int_map_data) == 0U)
    {
        return BMI270_DEVICE_IO_ERROR;
    }
    return (ImuBmi270Profile_Check(profile, &check) != 0U) ? BMI270_DEVICE_OK : BMI270_DEVICE_PROFILE_MISMATCH;
}
