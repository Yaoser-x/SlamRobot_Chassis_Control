#include "app_init.h"

#include "chassis_service.h"
#include "current_sensor_service.h"
#include "encoder_service.h"
#include "esp12f_flash_bridge.h"
#include "esp12f_service.h"
#include "imu_calibration_service.h"
#include "imu_service.h"
#include "led_status.h"
#include "line_control_service.h"
#include "line_uart.h"
#include "oled_ui.h"
#include "param_persistence.h"
#include "param_service.h"
#include "post_service.h"
#include "platform_reset.h"
#include "ps2_control_service.h"
#include "reset_reason_service.h"
#include "safety_service.h"
#include "ssd1306.h"
#include "system_snapshot_service.h"
#include "task_health_service.h"
#include "upper_uart_service.h"
#include "usart1_debug_console.h"

void App_Init(void)
{
    flash_param_bundle_t bundle;
    param_model_t        params;
    uint8_t              params_loaded;
    uint8_t              first_calibration_save_needed = 1U;

    ResetReasonService_Capture(PlatformReset_ReadReasonFlags());
    PlatformReset_ClearReasonFlags();
    TaskHealthService_Reset();
    ParamService_SetDefaults();
    params_loaded = (ParamPersistence_Load(&bundle) == FLASH_PARAM_STATUS_OK) ? 1U : 0U;
    if (params_loaded != 0U)
    {
        params = bundle.params;
        (void)ParamService_Set(&params);
    }
    EncoderService_Init();
    CurrentSensorService_Init();
    ImuService_Init();
    if (params_loaded != 0U)
    {
        (void)ImuService_ApplyCalibration(&bundle.imu_calibration);
        if (params.current_zero_valid != 0U)
        {
            CurrentSensorService_ApplyZeroCalibration(params.current_zero_raw);
        }
        first_calibration_save_needed =
            (bundle.imu_calibration.gyro_bias_dps[0] == 0.0f && bundle.imu_calibration.gyro_bias_dps[1] == 0.0f
             && bundle.imu_calibration.gyro_bias_dps[2] == 0.0f)
                ? 1U
                : 0U;
    }
    ImuCalibrationService_Init(first_calibration_save_needed);
    LedStatus_Init();
    SafetyService_Init();
    ChassisService_Init();
    UpperUartService_Init();
    (void)ParamService_GetSnapshot(&params);
    LineUart_Init();
    LineUart_SetThresholdConfig(params.line_threshold_raw, params.line_active_low);
    LineUart_InitSensor();
    LineControlService_Init();
    Ps2ControlService_Init();
    Esp12fService_Init();
    SystemSnapshotService_Init();
    Esp12fFlashBridge_Init();
    Usart1DebugConsole_Init();
    PostService_Run();
    SSD1306_Init();
    OLED_UI_Init();
}
