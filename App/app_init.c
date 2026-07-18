#include "app_init.h"

#include "system_publish_snapshot_service.h"
#include "system_publish_snapshot_provider.h"
#include "app_hardware_init.h"
#include "command_management_service.h"
#include "esp12f_flash_bridge.h"
#include "wireless_communication_service.h"
#include "line_following_service.h"
#include "line_following_composition.h"
#include "motion_control_service.h"
#include "motion_control_maintenance.h"
#include "oled_ui.h"
#include "parameter_management_service.h"
#include "power_on_self_test_service.h"
#include "power_management_service.h"
#include "platform_reset.h"
#include "safety_management_service.h"
#include "state_estimation_service.h"
#include "state_estimation_maintenance.h"
#include "state_estimation_composition.h"
#include "system_monitoring_service.h"
#include "teleoperation_service.h"
#include "host_communication_service.h"
#include "usart1_debug_console.h"

static uint8_t App_GetCalibrationMotionFacts(int16_t output_permille[4], uint8_t *enabled_mask)
{
    motion_control_status_t status;

    if (output_permille == 0 || enabled_mask == 0)
    {
        return 0U;
    }
    (void)MotionControl_GetStatus(&status);
    for (uint8_t index = 0U; index < 4U; ++index)
    {
        output_permille[index] = status.motor_effective_output_permille[index];
    }
    *enabled_mask = status.motor_enabled_mask;
    return 1U;
}

static uint8_t App_BeginCalibrationMaintenance(void)
{
    return (MotionControl_BeginMaintenance() == MOTION_CONTROL_MAINTENANCE_OK) ? 1U : 0U;
}

static void App_EndCalibrationMaintenance(void)
{
    MotionControl_EndMaintenance();
}

static void App_SetCurrentZeroPersistence(uint8_t enabled)
{
    ParameterManagement_SetCurrentZeroPersistence(enabled);
}

static uint8_t App_PersistImuCalibration(const imu_calibration_t *calibration, uint8_t persist_current_zero)
{
    param_model_t             params;
    power_management_status_t current;

    if (calibration == 0)
    {
        return 0U;
    }
    (void)ParameterManagement_GetSnapshot(&params);
    (void)PowerManagement_GetStatus(&current);
    if (persist_current_zero != 0U && current.current_zero_valid != 0U)
    {
        for (uint8_t index = 0U; index < 4U; ++index)
        {
            params.current_zero_raw[index] = current.current_zero_raw[index];
        }
        params.current_zero_valid = 1U;
        (void)ParameterManagement_Set(&params);
    }
    else if (persist_current_zero == 0U)
    {
        ParameterManagement_SetCurrentZeroPersistence(0U);
    }
    ParameterManagement_SetImuCalibration(calibration);
    return ParameterManagement_Save();
}

static uint8_t App_SaveParameters(void)
{
    return ParameterManagement_Save();
}

uint8_t App_InitWithConfig(const robot_config_t *config)
{
    param_model_t                              params;
    parameter_management_status_t              parameter_status;
    communication_publish_model_config_t       publish_config;
    uint8_t                                    params_loaded;
    uint8_t                                    first_calibration_save_needed = 1U;
    const state_estimation_calibration_ports_t calibration_ports             = {
                    .get_motion_facts             = App_GetCalibrationMotionFacts,
                    .begin_maintenance            = App_BeginCalibrationMaintenance,
                    .end_maintenance              = App_EndCalibrationMaintenance,
                    .persist                      = App_PersistImuCalibration,
                    .set_current_zero_persistence = App_SetCurrentZeroPersistence,
    };
    const line_following_calibration_ports_t line_calibration_ports = {
        .begin_maintenance = App_BeginCalibrationMaintenance,
        .end_maintenance   = App_EndCalibrationMaintenance,
        .save_parameters   = App_SaveParameters,
    };

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
        uint8_t calibration_applied =
            (calibration_valid != 0U) ? StateEstimation_ApplyImuCalibration(&parameter_status.imu_calibration) : 0U;
        if (params.current_zero_valid != 0U)
        {
            PowerManagement_ApplyCurrentZeroCalibration(params.current_zero_raw);
        }
        first_calibration_save_needed = (calibration_applied != 0U) ? 0U : 1U;
    }
    StateEstimation_InitCalibrationCoordinator(&calibration_ports,
                                               first_calibration_save_needed,
                                               config->parameter.persist_imu_calibration,
                                               config->parameter.persist_current_zero);
    AppHardware_InitStatusLed();
    if (MotionControl_Init(&config->motion) == 0U)
    {
        return 0U;
    }
    if (HostCommunication_Init(&config->communication) == 0U)
    {
        return 0U;
    }
    (void)ParameterManagement_GetSnapshot(&params);
    AppHardware_InitLineSensor(params.line_threshold_raw, params.line_active_low);
    LineFollowing_ConfigureCalibrationPorts(&line_calibration_ports);
    if (LineFollowing_Init(&config->line) == 0U || Teleoperation_Init(&config->teleoperation) == 0U)
    {
        return 0U;
    }
    if (WirelessCommunication_Init(&config->communication) == 0U)
    {
        return 0U;
    }
    publish_config = (communication_publish_model_config_t){
        .host_timeout_ms   = config->display.rpi_timeout_ms,
        .esp12f_timeout_ms = config->command.esp12f_timeout_ms,
        .line_timeout_ms   = config->display.line_timeout_ms,
    };
    if (SystemPublishSnapshot_Init(&publish_config, AppSystemPublishSnapshot_Collect) == 0U)
    {
        return 0U;
    }
    Esp12fFlashBridge_Init();
    Usart1DebugConsole_Init();
    PostService_Run();
    AppHardware_InitDisplay();
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
