#include "app_init.h"

#include "app_imu_calibration.h"
#include "app_publish_model.h"
#include "command_management_service.h"
#include "esp12f_flash_bridge.h"
#include "esp12f_service.h"
#include "led_status.h"
#include "line_following_service.h"
#include "motion_control_service.h"
#include "line_uart.h"
#include "oled_ui.h"
#include "parameter_management_service.h"
#include "post_service.h"
#include "power_management_service.h"
#include "platform_reset.h"
#include "safety_management_service.h"
#include "ssd1306.h"
#include "state_estimation_service.h"
#include "system_monitoring_service.h"
#include "teleoperation_service.h"
#include "upper_uart_service.h"
#include "usart1_debug_console.h"

uint8_t App_InitWithConfig(const robot_config_t *config)
{
    param_model_t                 params;
    parameter_management_status_t parameter_status;
    app_publish_model_config_t    publish_config;
    uint8_t                       params_loaded;
    uint8_t                       first_calibration_save_needed = 1U;

    if (RobotConfig_Validate(config) == 0U)
    {
        return 0U;
    }

    if (SystemMonitoring_Init(&config->system, PlatformReset_ReadReasonFlags()) == 0U)
    {
        return 0U;
    }
    PlatformReset_ClearReasonFlags();
    if (ParameterManagement_Init(&config->parameter) == 0U)
    {
        return 0U;
    }
    if (config->parameter.load_flash_on_boot != 0U)
    {
        (void)ParameterManagement_Load();
    }
    (void)ParameterManagement_GetStatus(&parameter_status);
    params_loaded = parameter_status.flash_loaded;
    params        = parameter_status.params;
    if (StateEstimation_Init(&config->state) == 0U)
    {
        return 0U;
    }
    if (PowerManagement_Init(&config->power) == 0U)
    {
        return 0U;
    }
    if (CommandManagement_Init(&config->command) == 0U)
    {
        return 0U;
    }
    if (SafetyManagement_Init(&config->safety) == 0U)
    {
        return 0U;
    }
    if (params_loaded != 0U)
    {
        (void)StateEstimation_ApplyImuCalibration(&parameter_status.imu_calibration);
        if (params.current_zero_valid != 0U)
        {
            PowerManagement_ApplyCurrentZeroCalibration(params.current_zero_raw);
        }
        first_calibration_save_needed = (parameter_status.imu_calibration.gyro_bias_dps[0] == 0.0f
                                         && parameter_status.imu_calibration.gyro_bias_dps[1] == 0.0f
                                         && parameter_status.imu_calibration.gyro_bias_dps[2] == 0.0f)
                                            ? 1U
                                            : 0U;
    }
    AppImuCalibration_Init(first_calibration_save_needed,
                           config->parameter.persist_imu_calibration,
                           config->parameter.persist_current_zero);
    LedStatus_Init();
    if (MotionControl_Init(&config->motion) == 0U)
    {
        return 0U;
    }
    if (UpperUartService_Init(&config->communication) == 0U)
    {
        return 0U;
    }
    (void)ParameterManagement_GetSnapshot(&params);
    LineUart_Init();
    LineUart_SetThresholdConfig(params.line_threshold_raw, params.line_active_low);
    LineUart_InitSensor();
    if (LineFollowing_Init(&config->line) == 0U || Teleoperation_Init(&config->teleoperation) == 0U)
    {
        return 0U;
    }
    if (Esp12fService_Init(&config->communication) == 0U)
    {
        return 0U;
    }
    publish_config = (app_publish_model_config_t){
        .host_timeout_ms   = config->display.rpi_timeout_ms,
        .esp12f_timeout_ms = config->command.esp12f_timeout_ms,
        .line_timeout_ms   = config->display.line_timeout_ms,
    };
    if (AppPublishModel_Init(&publish_config) == 0U)
    {
        return 0U;
    }
    Esp12fFlashBridge_Init();
    Usart1DebugConsole_Init();
    PostService_Run();
    SSD1306_Init();
    OLED_UI_Init();
    return 1U;
}

void App_Init(void)
{
    if (App_InitWithConfig(RobotConfig_GetDefault()) == 0U)
    {
        PlatformReset_FatalStop();
    }
}
