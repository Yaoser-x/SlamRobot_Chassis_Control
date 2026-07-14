#ifndef POWER_ON_SELF_TEST_H
#define POWER_ON_SELF_TEST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define POST_ERROR_DRV_FAULT          (1UL << 0)
#define POST_ERROR_ADC                (1UL << 1)
#define POST_ERROR_IMU                (1UL << 2)
#define POST_ERROR_ENCODER            (1UL << 3)
#define POST_RUNTIME_READY_TIMEOUT_MS 2000U

    typedef enum
    {
        POST_ITEM_PENDING = 0,
        POST_ITEM_OK      = 1,
        POST_ITEM_FAIL    = 2
    } post_item_status_t;

    typedef struct
    {
        uint8_t drv_fault_mask;
        uint8_t adc_current_valid;
        uint8_t imu_chip_id;
        uint8_t encoder_speed_valid_all;
        uint8_t runtime_checks_ready;
    } post_inputs_t;

    typedef struct
    {
        uint8_t            done;
        uint8_t            drv_fault_mask;
        uint8_t            adc_current_valid;
        uint8_t            imu_chip_id;
        uint8_t            encoder_speed_valid_all;
        post_item_status_t drv_status;
        post_item_status_t adc_status;
        post_item_status_t imu_status;
        post_item_status_t encoder_status;
        uint32_t           error_flags;
    } post_result_t;

    /** Evaluates a POST snapshot without accessing hardware. */
    void POST_Evaluate(const post_inputs_t *inputs, post_result_t *result);
    /** Refreshes ADC and encoder checks after the scheduler has started. */
    void        PostService_UpdateRuntime(uint32_t now_ms);
    void        PostService_Run(void);
    void        POST_GetResult(post_result_t *result);
    const char *POST_ItemStatusString(post_item_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* POWER_ON_SELF_TEST_H */
