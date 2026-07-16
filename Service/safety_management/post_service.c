#include "post_service.h"
#include "platform_time.h"
#include "debug_uart_transport.h"

#include <stdio.h>
#include <string.h>

#ifndef POWER_ON_SELF_TEST_HOST_TEST
#include "imu_bmi270.h"

#include "led_status.h"

#include "motor_driver.h"

#include "power_management_service.h"
#include "state_estimation_service.h"

#endif

static post_result_t last_post_result;
#ifndef POWER_ON_SELF_TEST_HOST_TEST
static uint32_t post_runtime_start_ms;
#endif

const char *POST_ItemStatusString(post_item_status_t status)
{
    switch (status)
    {
        case POST_ITEM_OK:
            return "OK";
        case POST_ITEM_FAIL:
            return "FAIL";
        default:
            return "PENDING";
    }
}

void POST_Evaluate(const post_inputs_t *inputs, post_result_t *result)
{
    if (inputs == 0 || result == 0)
    {
        return;
    }

    *result                         = (post_result_t){0};
    result->drv_fault_mask          = inputs->drv_fault_mask;
    result->adc_current_valid       = inputs->adc_current_valid;
    result->imu_chip_id             = inputs->imu_chip_id;
    result->encoder_speed_valid_all = inputs->encoder_speed_valid_all;
    result->drv_status              = (inputs->drv_fault_mask == 0U) ? POST_ITEM_OK : POST_ITEM_FAIL;
    result->imu_status              = (inputs->imu_chip_id == 0x24U) ? POST_ITEM_OK : POST_ITEM_FAIL;

    if (inputs->drv_fault_mask != 0U)
    {
        result->error_flags |= POST_ERROR_DRV_FAULT;
    }
    if (inputs->imu_chip_id != 0x24U)
    {
        result->error_flags |= POST_ERROR_IMU;
    }

    if (inputs->runtime_checks_ready == 0U)
    {
        result->adc_status     = POST_ITEM_PENDING;
        result->encoder_status = POST_ITEM_PENDING;
        return;
    }

    result->done           = 1U;
    result->adc_status     = (inputs->adc_current_valid != 0U) ? POST_ITEM_OK : POST_ITEM_FAIL;
    result->encoder_status = (inputs->encoder_speed_valid_all != 0U) ? POST_ITEM_OK : POST_ITEM_FAIL;
    if (inputs->adc_current_valid == 0U)
    {
        result->error_flags |= POST_ERROR_ADC;
    }
    if (inputs->encoder_speed_valid_all == 0U)
    {
        result->error_flags |= POST_ERROR_ENCODER;
    }
}

void POST_GetResult(post_result_t *result)
{
    if (result != 0)
    {
        *result = last_post_result;
    }
}

#ifdef POWER_ON_SELF_TEST_HOST_TEST
void PostService_Run(void)
{
    last_post_result = (post_result_t){0};
}

void PostService_UpdateRuntime(uint32_t now_ms)
{
    (void)now_ms;
}
#else
static void POST_WriteLine(const post_result_t *result)
{
    char tx[192];
    int  len = snprintf(tx,
                       sizeof(tx),
                       "POST: done=%u drv=%s(mask=0x%02X) adc=%s imu=%s(chip=0x%02X) enc=%s errors=0x%08lX\r\n",
                       result->done,
                       POST_ItemStatusString(result->drv_status),
                       result->drv_fault_mask,
                       POST_ItemStatusString(result->adc_status),
                       POST_ItemStatusString(result->imu_status),
                       result->imu_chip_id,
                       POST_ItemStatusString(result->encoder_status),
                       (unsigned long)result->error_flags);
    if (len > 0)
    {
        DebugUartTransport_Write((const uint8_t *)tx, (uint16_t)strlen(tx), 100U);
    }
}

void PostService_Run(void)
{
    post_inputs_t        inputs;
    motor_driver_state_t motor_state;

    inputs = (post_inputs_t){0};

    MotorDriver_UpdateFaults();
    MotorDriver_GetState(&motor_state);
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        if (motor_state.fault_active[i] != 0U)
        {
            inputs.drv_fault_mask |= (uint8_t)(1U << i);
        }
    }

    inputs.imu_chip_id = (ImuBmi270_ProbeNow() != 0U) ? 0x24U : 0x00U;

    POST_Evaluate(&inputs, &last_post_result);
    post_runtime_start_ms = PlatformTime_NowMs();
    if (last_post_result.error_flags != 0UL)
    {
        LedStatus_SetMode(LED_STATUS_FAULT);
    }
    POST_WriteLine(&last_post_result);
}

void PostService_UpdateRuntime(uint32_t now_ms)
{
    post_inputs_t                   inputs = {0};
    power_management_status_t       power;
    state_estimation_wheel_status_t wheel;

    (void)now_ms; /* time comparison uses PlatformTime_NowMs() for consistent pre-scheduler baseline */

    if (last_post_result.done != 0U)
    {
        return;
    }

    (void)PowerManagement_GetStatus(&power);
    (void)StateEstimation_GetWheel(&wheel);
    if ((power.current_valid == 0U || wheel.speed_valid_all == 0U)
        && (PlatformTime_NowMs() - post_runtime_start_ms) < POST_RUNTIME_READY_TIMEOUT_MS)
    {
        return;
    }

    inputs.drv_fault_mask          = last_post_result.drv_fault_mask;
    inputs.adc_current_valid       = power.current_valid;
    inputs.imu_chip_id             = last_post_result.imu_chip_id;
    inputs.encoder_speed_valid_all = wheel.speed_valid_all;
    inputs.runtime_checks_ready    = 1U;
    POST_Evaluate(&inputs, &last_post_result);
    if (last_post_result.error_flags != 0UL)
    {
        LedStatus_SetMode(LED_STATUS_FAULT);
    }
    POST_WriteLine(&last_post_result);
}
#endif
