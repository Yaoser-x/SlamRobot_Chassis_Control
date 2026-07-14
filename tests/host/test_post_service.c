#include "post_service.h"

#include <stdio.h>

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        (void)printf("FAIL: %s\n", message);
        __builtin_exit(1);
    }
}

static void test_evaluate_all_ok(void)
{
    post_inputs_t inputs = {
        .drv_fault_mask          = 0U,
        .adc_current_valid       = 1U,
        .imu_chip_id             = 0x24U,
        .encoder_speed_valid_all = 1U,
        .runtime_checks_ready    = 1U,
    };
    post_result_t result;

    POST_Evaluate(&inputs, &result);

    require_int(result.done == 1U, "post marked done");
    require_int(result.error_flags == 0UL, "post ok has no errors");
    require_int(result.drv_fault_mask == 0U, "post keeps drv mask");
    require_int(result.adc_status == POST_ITEM_OK, "adc is ready");
    require_int(result.encoder_status == POST_ITEM_OK, "encoder is ready");
}

static void test_evaluate_keeps_runtime_checks_pending_before_scheduler(void)
{
    post_inputs_t inputs = {
        .drv_fault_mask          = 0U,
        .adc_current_valid       = 0U,
        .imu_chip_id             = 0x24U,
        .encoder_speed_valid_all = 0U,
        .runtime_checks_ready    = 0U,
    };
    post_result_t result;

    POST_Evaluate(&inputs, &result);

    require_int(result.done == 0U, "post waits for runtime checks");
    require_int(result.error_flags == 0UL, "pending runtime checks are not errors");
    require_int(result.adc_status == POST_ITEM_PENDING, "adc starts pending");
    require_int(result.encoder_status == POST_ITEM_PENDING, "encoder starts pending");
}

static void test_evaluate_reports_each_failed_runtime_subsystem(void)
{
    post_inputs_t inputs = {
        .drv_fault_mask          = 0x05U,
        .adc_current_valid       = 0U,
        .imu_chip_id             = 0x00U,
        .encoder_speed_valid_all = 0U,
        .runtime_checks_ready    = 1U,
    };
    post_result_t result;

    POST_Evaluate(&inputs, &result);

    require_int((result.error_flags & POST_ERROR_DRV_FAULT) != 0UL, "drv fault bit");
    require_int((result.error_flags & POST_ERROR_ADC) != 0UL, "adc bit");
    require_int((result.error_flags & POST_ERROR_IMU) != 0UL, "imu bit");
    require_int((result.error_flags & POST_ERROR_ENCODER) != 0UL, "encoder bit");
    require_int(result.drv_fault_mask == 0x05U, "fault mask retained");
}

int main(void)
{
    test_evaluate_all_ok();
    test_evaluate_keeps_runtime_checks_pending_before_scheduler();
    test_evaluate_reports_each_failed_runtime_subsystem();

    (void)printf("PASS: power-on self-test host tests\n");
    return 0;
}
