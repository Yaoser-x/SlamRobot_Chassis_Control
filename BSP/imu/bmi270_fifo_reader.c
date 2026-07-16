#include "bmi270_fifo_reader.h"

#include <string.h>

#define BMI270_FIFO_HEADER_ACC       0x84U
#define BMI270_FIFO_HEADER_GYR       0x88U
#define BMI270_FIFO_HEADER_GYR_ACC   0x8CU
#define BMI270_FIFO_HEADER_SENS_TIME 0x44U
#define BMI270_FIFO_HEADER_SKIP      0x40U
#define BMI270_FIFO_HEADER_INPUT_CFG 0x48U
#define BMI270_FIFO_HEADER_OVERREAD  0x80U

static int16_t ImuBmi270Fifo_ReadI16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8) | data[0]);
}

static uint8_t ImuBmi270Fifo_FramePayloadLen(uint8_t header, uint8_t *accel, uint8_t *gyro, uint8_t *payload_len)
{
    *accel       = 0U;
    *gyro        = 0U;
    *payload_len = 0U;

    switch (header)
    {
        case BMI270_FIFO_HEADER_ACC:
            *accel       = 1U;
            *payload_len = 6U;
            return 1U;
        case BMI270_FIFO_HEADER_GYR:
            *gyro        = 1U;
            *payload_len = 6U;
            return 1U;
        case BMI270_FIFO_HEADER_GYR_ACC:
            *accel       = 1U;
            *gyro        = 1U;
            *payload_len = 12U;
            return 1U;
        default:
            return 0U;
    }
}

uint8_t ImuBmi270Fifo_Parse(const uint8_t                  *fifo,
                            uint16_t                        fifo_len,
                            imu_bmi270_fifo_sample_t       *samples,
                            uint16_t                        max_samples,
                            imu_bmi270_fifo_parse_result_t *result)
{
    uint16_t offset = 0U;

    if (fifo == 0 || result == 0)
    {
        return 0U;
    }

    memset(result, 0, sizeof(*result));
    if (samples != 0 && max_samples > 0U)
    {
        memset(samples, 0, sizeof(samples[0]) * max_samples);
    }

    while (offset < fifo_len)
    {
        uint8_t header      = fifo[offset++];
        uint8_t accel       = 0U;
        uint8_t gyro        = 0U;
        uint8_t payload_len = 0U;

        if (header == BMI270_FIFO_HEADER_OVERREAD)
        {
            result->flags |= IMU_BMI270_FIFO_PARSE_OVERREAD;
            break;
        }

        if (header == BMI270_FIFO_HEADER_SENS_TIME)
        {
            if ((uint16_t)(fifo_len - offset) < 3U)
            {
                result->flags |= IMU_BMI270_FIFO_PARSE_TRUNCATED;
                return 0U;
            }
            result->sensor_time =
                ((uint32_t)fifo[offset]) | ((uint32_t)fifo[offset + 1U] << 8) | ((uint32_t)fifo[offset + 2U] << 16);
            result->sensor_time_valid = 1U;
            offset                    = (uint16_t)(offset + 3U);
            continue;
        }

        if (header == BMI270_FIFO_HEADER_SKIP)
        {
            if (offset >= fifo_len)
            {
                result->flags |= IMU_BMI270_FIFO_PARSE_TRUNCATED;
                return 0U;
            }
            result->flags |= IMU_BMI270_FIFO_PARSE_SKIP_FRAME;
            result->skipped_frame_count += fifo[offset++];
            continue;
        }

        if (header == BMI270_FIFO_HEADER_INPUT_CFG)
        {
            if (offset >= fifo_len)
            {
                result->flags |= IMU_BMI270_FIFO_PARSE_TRUNCATED;
                return 0U;
            }
            result->flags |= IMU_BMI270_FIFO_PARSE_INPUT_CFG;
            offset++;
            continue;
        }

        if (ImuBmi270Fifo_FramePayloadLen(header, &accel, &gyro, &payload_len) == 0U)
        {
            result->flags |= IMU_BMI270_FIFO_PARSE_TRUNCATED;
            return 0U;
        }
        if ((uint16_t)(fifo_len - offset) < payload_len)
        {
            result->flags |= IMU_BMI270_FIFO_PARSE_TRUNCATED;
            return 0U;
        }

        if (samples != 0 && result->sample_count < max_samples)
        {
            imu_bmi270_fifo_sample_t *sample = &samples[result->sample_count];
            uint16_t                  cursor = offset;

            if (accel != 0U && gyro != 0U)
            {
                sample->gyro_raw[0]  = ImuBmi270Fifo_ReadI16(&fifo[cursor]);
                sample->gyro_raw[1]  = ImuBmi270Fifo_ReadI16(&fifo[cursor + 2U]);
                sample->gyro_raw[2]  = ImuBmi270Fifo_ReadI16(&fifo[cursor + 4U]);
                sample->gyro_valid   = 1U;
                cursor               = (uint16_t)(cursor + 6U);
                sample->accel_raw[0] = ImuBmi270Fifo_ReadI16(&fifo[cursor]);
                sample->accel_raw[1] = ImuBmi270Fifo_ReadI16(&fifo[cursor + 2U]);
                sample->accel_raw[2] = ImuBmi270Fifo_ReadI16(&fifo[cursor + 4U]);
                sample->accel_valid  = 1U;
            }
            else if (accel != 0U)
            {
                sample->accel_raw[0] = ImuBmi270Fifo_ReadI16(&fifo[cursor]);
                sample->accel_raw[1] = ImuBmi270Fifo_ReadI16(&fifo[cursor + 2U]);
                sample->accel_raw[2] = ImuBmi270Fifo_ReadI16(&fifo[cursor + 4U]);
                sample->accel_valid  = 1U;
            }
            else if (gyro != 0U)
            {
                sample->gyro_raw[0] = ImuBmi270Fifo_ReadI16(&fifo[cursor]);
                sample->gyro_raw[1] = ImuBmi270Fifo_ReadI16(&fifo[cursor + 2U]);
                sample->gyro_raw[2] = ImuBmi270Fifo_ReadI16(&fifo[cursor + 4U]);
                sample->gyro_valid  = 1U;
            }
        }

        if (result->sample_count < 0xFFFFFFFFUL)
        {
            result->sample_count++;
        }
        offset = (uint16_t)(offset + payload_len);
    }

    return 1U;
}
