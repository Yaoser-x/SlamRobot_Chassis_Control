#include "app_init.h"
#include "chassis_runtime_coordinator.h"

#include "system_publish_snapshot_service.h"
#include "app_hardware_init.h"
#include "command_management_service.h"
#include "esp12f_flash_bridge.h"
#include "wireless_communication_service.h"
#include "line_following_service.h"
#include "line_calibration_orchestrator.h"
#include "imu_calibration_orchestrator.h"
#include "motion_control_service.h"
#include "oled_ui.h"
#include "parameter_management_service.h"
#include "power_on_self_test_service.h"
#include "power_management_service.h"
#include "platform_reset.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"
#include "state_estimation_maintenance.h"
#include "system_monitoring_service.h"
#include "teleoperation_service.h"
#include "host_communication_service.h"
#include "firmware_identity_provider.h"
#include "usart1_debug_console.h"

uint8_t App_InitWithConfig(const robot_config_t *config)
{
    param_model_t                     params;
    parameter_management_status_t     parameter_status;
    uint8_t                           params_loaded;
    communication_firmware_identity_t firmware_identity;
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
        uint8_t calibration_valid = ParameterImuCalibration_Validate(&parameter_status.imu_calibration);
        if (calibration_valid != 0U)
        {
            (void)StateEstimation_ApplyImuCalibration(&parameter_status.imu_calibration);
        }
        if (params.current_zero_valid != 0U)
        {
            PowerManagement_ApplyCurrentZeroCalibration(params.current_zero_raw);
        }
    }
    if (StateEstimation_ArmAutomaticImuCalibration() != (uint8_t)STATE_ESTIMATION_RESULT_OK)
    {
        return 0U;
    }
    ImuCalibrationOrchestrator_Init(config->parameter.persist_imu_calibration, config->parameter.persist_current_zero);
    AppHardware_InitStatusLed();
    if (MotionControl_Init(&config->motion) == 0U)
    {
        return 0U;
    }
    firmware_identity = FirmwareIdentityProvider_Build();
    if (HostCommunication_Init(&config->communication, &firmware_identity) == 0U)
    {
        return 0U;
    }
    (void)ParameterManagement_GetSnapshot(&params);
    AppHardware_InitLineSensor(params.line_threshold_raw, params.line_active_low);
    LineCalibrationOrchestrator_Init(0U);
    if (LineFollowing_Init(&config->line) == 0U || Teleoperation_Init(&config->teleoperation) == 0U)
    {
        return 0U;
    }
    if (WirelessCommunication_Init(&config->communication, &firmware_identity) == 0U)
    {
        return 0U;
    }
    SystemPublishSnapshot_Init();
    Esp12fFlashBridge_Init();
    Usart1DebugConsole_Init();
    PostService_Run();
    AppHardware_InitDisplay();
    OLED_UI_Init();
    ChassisRuntimeCoordinator_Init();
    return 1U;
}

void App_Init(void)
{
    if (App_InitWithConfig(RobotConfig_GetDefault()) == 0U)
    {
        PlatformReset_FatalStop();
    }
}
