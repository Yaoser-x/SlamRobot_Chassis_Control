#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "bmi270_device.h"
#include "bmi270_registers.h"
#include "bmi270_config.h"

static uint8_t  registers[256];
static uint32_t burst_bytes;
static uint32_t burst_count;
static uint32_t delay_ms;

void HAL_Delay(uint32_t milliseconds)
{
    delay_ms += milliseconds;
}

static uint8_t FakeReadReg(uint8_t reg, uint8_t *value)
{
    if (value == 0)
    {
        return 0U;
    }
    *value = registers[reg];
    return 1U;
}

static uint8_t FakeWriteReg(uint8_t reg, uint8_t value)
{
    registers[reg] = value;
    return 1U;
}

static uint8_t FakeWriteBytes(uint8_t reg, const uint8_t *data, uint8_t len)
{
    assert(reg == BMI270_REG_INIT_DATA);
    assert(data != 0);
    assert(len != 0U && (len & 1U) == 0U && len <= 32U);
    burst_bytes += len;
    burst_count++;
    return 1U;
}

static const bmi270_device_io_t io = {
    .read_reg    = FakeReadReg,
    .write_reg   = FakeWriteReg,
    .write_bytes = FakeWriteBytes,
};

static void ResetFake(void)
{
    memset(registers, 0, sizeof(registers));
    burst_bytes = 0U;
    burst_count = 0U;
    delay_ms    = 0U;
}

static void TestConfigUpload(void)
{
    ResetFake();
    assert(Bmi270Device_LoadConfig(&io) == BMI270_DEVICE_OK);
    assert(burst_bytes == bmi270_config_file_size);
    assert(burst_count == (bmi270_config_file_size + 31U) / 32U);
    assert(registers[BMI270_REG_INIT_CTRL] == BMI270_INIT_CTRL_COMPLETE);
}

static void TestInitPolling(void)
{
    ResetFake();
    registers[BMI270_REG_INTERNAL_STATUS] = BMI270_INTERNAL_STATUS_INIT_OK;
    assert(Bmi270Device_WaitInitOk(&io) == BMI270_DEVICE_OK);
    assert(delay_ms == 1U);

    ResetFake();
    assert(Bmi270Device_WaitInitOk(&io) == BMI270_DEVICE_CONFIG_ERROR);
    assert(delay_ms == 25U);
}

static void TestProfileApplyAndVerify(void)
{
    const imu_bmi270_profile_t *profile = ImuBmi270Profile_Get(IMU_BMI270_PROFILE_PERFORMANCE);

    ResetFake();
    assert(profile != 0);
    assert(Bmi270Device_ApplyProfile(&io, profile) == BMI270_DEVICE_OK);
    assert(registers[BMI270_REG_ACC_CONF] == profile->acc_conf);
    assert(registers[BMI270_REG_GYR_RANGE] == profile->gyr_range);
    assert(registers[BMI270_REG_FIFO_CONFIG_1] == profile->fifo_config_1);
    assert(registers[BMI270_REG_PWR_CTRL] == profile->pwr_ctrl);
    assert(delay_ms == 3U);
}

int main(void)
{
    TestConfigUpload();
    TestInitPolling();
    TestProfileApplyAndVerify();
    puts("BMI270 device tests passed");
    return 0;
}
