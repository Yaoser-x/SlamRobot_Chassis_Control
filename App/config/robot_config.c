#include "robot_config.h"

#include "parameter_management_service.h"

static const robot_config_t robot_default_config = {
    .motion =
        {
            .max_linear_mps                     = 0.5f,
            .max_angular_rps                    = 10.0f,
            .open_loop_full_mps                 = 0.5f,
            .angular_epsilon_rps                = 0.0001f,
            .speed_ramp_mps2                    = 1.0f,
            .angular_ramp_rps2                  = 10.0f,
            .maintenance_max_speed_mps          = 0.02f,
            .pid_correction_limit               = 500.0f,
            .pid_stop_epsilon_mps               = 0.005f,
            .pid_direction_epsilon_mps          = 0.02f,
            .pid_feedback_min_target_mps        = 0.08f,
            .pid_feedback_min_speed_mps         = 0.01f,
            .test_mode_lease_ms                 = 400U,
            .encoder_feedback_timeout_ms        = 150U,
            .pid_feedback_loss_count            = 50U,
            .pid_enabled                        = 1U,
            .wheel_speed_proportional_scale     = 1U,
            .motor_current_limiter_observe_only = 1U,
            .current_soft_limit_enabled         = 0U,
        },
    .state =
        {
            .wheel_feedback_timeout_ms = 150U,
            .imu_fresh_timeout_ms      = 50U,
        },
    .power =
        {
            .current_zero_max_speed_mps = 0.02f,
            .update_period_ms           = 20U,
        },
    .safety =
        {
            .battery_low_warn_v           = 10.5f,
            .battery_low_clear_v          = 11.0f,
            .battery_critical_v           = 9.0f,
            .battery_recover_v            = 9.6f,
            .battery_critical_debounce_ms = 500U,
            .battery_recover_debounce_ms  = 2000U,
            .update_period_ms             = 20U,
            .overcurrent_startup_blank_ms = 200U,
            .overcurrent_startup_rearm_ms = 200U,
            .battery_low_monitor_enabled  = 1U,
            .overcurrent_fault_enabled    = 0U,
            .current_observe_a            = {1.5f, 1.5f, 1.5f, 1.5f},
            .current_fault_a              = {2.5f, 2.5f, 2.5f, 2.5f},
            .current_fault_debounce_ms    = 100U,
        },
    .command =
        {
            .host_timeout_ms   = 200U,
            .ps2_timeout_ms    = 500U,
            .esp12f_timeout_ms = 500U,
            .line_timeout_ms   = 50U,
            .debug_timeout_ms  = 2000U,
        },
    .teleoperation =
        {
            .linear_max_mps             = 0.5f,
            .angular_max_rps            = 5.0f,
            .dpad_linear_mps            = 0.5f,
            .dpad_angular_rps           = 5.0f,
            .manual_cancel_threshold    = 0.12f,
            .heading_quarter_timeout_ms = 6000U,
            .heading_full_timeout_ms    = 20000U,
            .heading_imu_fresh_ms       = 50U,
            .idle_release_ms            = 2000U,
            .axis_center                = 128,
            .axis_deadzone              = 18,
            .offline_fail_limit         = 3U,
            .macro_l1_mask              = 0x04U,
            .macro_r1_mask              = 0x08U,
            .macro_l2_mask              = 0x01U,
            .macro_r2_mask              = 0x02U,
            .line_toggle_mask           = 0x10U,
            .linecal_floor_mask         = 0x80U,
            .linecal_line_mask          = 0x20U,
        },
    .line =
        {
            .angular_max_rps        = 2.0f,
            .sensor_timeout_ms      = 50U,
            .default_enabled        = 0U,
            .detect_threshold_count = 1U,
        },
    .communication =
        {
            .host_status_period_ms     = 50U,
            .host_imu_status_period_ms = 20U,
            .host_diagnostic_period_ms = 200U,
            .esp12f_status_period_ms   = 100U,
        },
    .parameter =
        {
            .factory_defaults =
                {
                    .version                       = PARAM_MODEL_VERSION,
                    .max_linear_mps                = 0.5f,
                    .max_angular_rps               = 10.0f,
                    .speed_ramp_mps2               = 1.0f,
                    .angular_ramp_rps2             = 10.0f,
                    .wheel_radius_m                = 0.035f,
                    .track_width_m                 = 0.176f,
                    .pid_kp                        = {50.0f, 1000.0f, 1200.0f, 100.0f},
                    .pid_ki                        = {8.0f, 800.0f, 1000.0f, 0.0f},
                    .pid_kd                        = {0.05f, 0.15f, 0.18f, 0.0f},
                    .pid_integral_limit            = 60.0f,
                    .motor_dir                     = {-1, -1, 1, 1},
                    .encoder_dir                   = {1, 1, -1, -1},
                    .line_threshold_raw            = {500U, 500U, 500U, 500U, 500U, 500U, 500U, 500U},
                    .line_active_low               = 1U,
                    .line_kp                       = 0.6f,
                    .line_kd                       = 0.05f,
                    .line_speed_mps                = 0.15f,
                    .line_slowdown_gain            = 0.7f,
                    .line_detect_debounce_frames   = 2U,
                    .line_lost_debounce_frames     = 2U,
                    .current_observe_a             = {1.5f, 1.5f, 1.5f, 1.5f},
                    .current_soft_limit_a          = {2.0f, 2.0f, 2.0f, 2.0f},
                    .current_fault_a               = {2.5f, 2.5f, 2.5f, 2.5f},
                    .current_fault_debounce_ms     = 100U,
                    .straight_wheel_coupling_gain  = 0.30f,
                    .straight_max_speed_mps        = 0.30f,
                    .straight_heading_hold_enabled = 0U,
                },
            .load_flash_on_boot      = 1U,
            .persist_imu_calibration = 1U,
            .persist_current_zero    = 1U,
        },
    .system =
        {
            .task_timeout_ms = {80U, 40U, 40U, 80U, 40U, 40U, 80U, 200U, 400U},
        },
    .tasks =
        {
            [APP_TASK_SAFETY] = {20U, 4096U, APP_TASK_PRIORITY_HIGH, 0U},
            [APP_TASK_MOTOR]  = {10U, 2048U, APP_TASK_PRIORITY_ABOVE_NORMAL, 0U},
            [APP_TASK_HOST]   = {5U, 2048U, APP_TASK_PRIORITY_NORMAL, 0U},
            [APP_TASK_IMU]    = {10U, 2048U, APP_TASK_PRIORITY_NORMAL, 1U},
            [APP_TASK_LINE]   = {5U, 4096U, APP_TASK_PRIORITY_BELOW_NORMAL, 0U},
            [APP_TASK_ESP12F] = {5U, 2048U, APP_TASK_PRIORITY_BELOW_NORMAL, 0U},
            [APP_TASK_PS2]    = {20U, 2048U, APP_TASK_PRIORITY_NORMAL, 0U},
            [APP_TASK_LED]    = {50U, 512U, APP_TASK_PRIORITY_LOW, 0U},
            [APP_TASK_OLED]   = {100U, 2048U, APP_TASK_PRIORITY_LOW, 0U},
            [APP_TASK_DEBUG]  = {10U, 12288U, APP_TASK_PRIORITY_BELOW_NORMAL, 0U},
        },
    .display =
        {
            .welcome_duration_ms   = 5000U,
            .selfcheck_item_ms     = 600U,
            .error_blink_period_ms = 500U,
            .rpi_timeout_ms        = 500U,
            .line_timeout_ms       = 50U,
            .selfcheck_total_items = 8U,
        },
};

const robot_config_t *RobotConfig_GetDefault(void)
{
    return &robot_default_config;
}

static uint8_t RobotConfig_MotionMatches(const motion_control_config_t *config)
{
    const motion_control_config_t *expected = &robot_default_config.motion;

    return (config->max_linear_mps == expected->max_linear_mps && config->max_angular_rps == expected->max_angular_rps
            && config->open_loop_full_mps == expected->open_loop_full_mps
            && config->angular_epsilon_rps == expected->angular_epsilon_rps
            && config->speed_ramp_mps2 == expected->speed_ramp_mps2
            && config->angular_ramp_rps2 == expected->angular_ramp_rps2
            && config->maintenance_max_speed_mps == expected->maintenance_max_speed_mps
            && config->pid_correction_limit == expected->pid_correction_limit
            && config->pid_stop_epsilon_mps == expected->pid_stop_epsilon_mps
            && config->pid_direction_epsilon_mps == expected->pid_direction_epsilon_mps
            && config->pid_feedback_min_target_mps == expected->pid_feedback_min_target_mps
            && config->pid_feedback_min_speed_mps == expected->pid_feedback_min_speed_mps
            && config->test_mode_lease_ms == expected->test_mode_lease_ms
            && config->encoder_feedback_timeout_ms == expected->encoder_feedback_timeout_ms
            && config->pid_feedback_loss_count == expected->pid_feedback_loss_count
            && config->motor_current_limiter_observe_only == expected->motor_current_limiter_observe_only
            && config->current_soft_limit_enabled == expected->current_soft_limit_enabled
            && config->pid_enabled == expected->pid_enabled
            && config->wheel_speed_proportional_scale == expected->wheel_speed_proportional_scale)
               ? 1U
               : 0U;
}

static uint8_t RobotConfig_TeleoperationMatches(const teleoperation_config_t *config)
{
    const teleoperation_config_t *expected = &robot_default_config.teleoperation;

    return (config->linear_max_mps == expected->linear_max_mps && config->angular_max_rps == expected->angular_max_rps
            && config->dpad_linear_mps == expected->dpad_linear_mps
            && config->dpad_angular_rps == expected->dpad_angular_rps
            && config->manual_cancel_threshold == expected->manual_cancel_threshold
            && config->heading_quarter_timeout_ms == expected->heading_quarter_timeout_ms
            && config->heading_full_timeout_ms == expected->heading_full_timeout_ms
            && config->heading_imu_fresh_ms == expected->heading_imu_fresh_ms
            && config->idle_release_ms == expected->idle_release_ms && config->axis_center == expected->axis_center
            && config->axis_deadzone == expected->axis_deadzone
            && config->offline_fail_limit == expected->offline_fail_limit
            && config->macro_l1_mask == expected->macro_l1_mask && config->macro_r1_mask == expected->macro_r1_mask
            && config->macro_l2_mask == expected->macro_l2_mask && config->macro_r2_mask == expected->macro_r2_mask
            && config->line_toggle_mask == expected->line_toggle_mask
            && config->linecal_floor_mask == expected->linecal_floor_mask
            && config->linecal_line_mask == expected->linecal_line_mask)
               ? 1U
               : 0U;
}

static uint8_t RobotConfig_TasksMatch(const robot_config_t *config)
{
    for (uint32_t index = 0U; index < (uint32_t)APP_TASK_COUNT; ++index)
    {
        const app_task_config_t *actual   = &config->tasks[index];
        const app_task_config_t *expected = &robot_default_config.tasks[index];
        if (actual->period_ms != expected->period_ms || actual->stack_size_bytes != expected->stack_size_bytes
            || actual->priority != expected->priority || actual->event_driven != expected->event_driven)
        {
            return 0U;
        }
    }
    for (uint32_t index = 0U; index < SYSTEM_MONITORING_TASK_COUNT; ++index)
    {
        if (config->system.task_timeout_ms[index] != robot_default_config.system.task_timeout_ms[index])
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t RobotConfig_SafetyMatches(const safety_management_config_t *config)
{
    const safety_management_config_t *expected = &robot_default_config.safety;

    if (config->current_fault_debounce_ms != expected->current_fault_debounce_ms)
    {
        return 0U;
    }
    for (uint8_t index = 0U; index < SAFETY_MANAGEMENT_MOTOR_COUNT; ++index)
    {
        if (config->current_observe_a[index] != expected->current_observe_a[index]
            || config->current_fault_a[index] != expected->current_fault_a[index])
        {
            return 0U;
        }
    }
    return 1U;
}

uint8_t RobotConfig_Validate(const robot_config_t *config)
{
    if (config == 0)
    {
        return 0U;
    }

    /* Until each owner consumes its config, only the characterized product
     * aggregate is accepted. Compare fields, never structure padding. */
    if (RobotConfig_MotionMatches(&config->motion) == 0U
        || RobotConfig_TeleoperationMatches(&config->teleoperation) == 0U || RobotConfig_TasksMatch(config) == 0U
        || RobotConfig_SafetyMatches(&config->safety) == 0U)
    {
        return 0U;
    }

    return (config->state.wheel_feedback_timeout_ms == robot_default_config.state.wheel_feedback_timeout_ms
            && config->state.imu_fresh_timeout_ms == robot_default_config.state.imu_fresh_timeout_ms
            && config->power.current_zero_max_speed_mps == robot_default_config.power.current_zero_max_speed_mps
            && config->power.update_period_ms == robot_default_config.power.update_period_ms
            && config->safety.battery_low_warn_v == robot_default_config.safety.battery_low_warn_v
            && config->safety.battery_low_clear_v == robot_default_config.safety.battery_low_clear_v
            && config->safety.battery_critical_v == robot_default_config.safety.battery_critical_v
            && config->safety.battery_recover_v == robot_default_config.safety.battery_recover_v
            && config->safety.battery_critical_debounce_ms == robot_default_config.safety.battery_critical_debounce_ms
            && config->safety.battery_recover_debounce_ms == robot_default_config.safety.battery_recover_debounce_ms
            && config->safety.update_period_ms == robot_default_config.safety.update_period_ms
            && config->safety.overcurrent_startup_blank_ms == robot_default_config.safety.overcurrent_startup_blank_ms
            && config->safety.overcurrent_startup_rearm_ms == robot_default_config.safety.overcurrent_startup_rearm_ms
            && config->safety.battery_low_monitor_enabled == robot_default_config.safety.battery_low_monitor_enabled
            && config->safety.overcurrent_fault_enabled == robot_default_config.safety.overcurrent_fault_enabled
            && config->command.host_timeout_ms == robot_default_config.command.host_timeout_ms
            && config->command.ps2_timeout_ms == robot_default_config.command.ps2_timeout_ms
            && config->command.esp12f_timeout_ms == robot_default_config.command.esp12f_timeout_ms
            && config->command.line_timeout_ms == robot_default_config.command.line_timeout_ms
            && config->command.debug_timeout_ms == robot_default_config.command.debug_timeout_ms
            && config->line.angular_max_rps == robot_default_config.line.angular_max_rps
            && config->line.sensor_timeout_ms == robot_default_config.line.sensor_timeout_ms
            && config->line.default_enabled == robot_default_config.line.default_enabled
            && config->line.detect_threshold_count == robot_default_config.line.detect_threshold_count
            && config->communication.host_status_period_ms == robot_default_config.communication.host_status_period_ms
            && config->communication.host_imu_status_period_ms
                   == robot_default_config.communication.host_imu_status_period_ms
            && config->communication.host_diagnostic_period_ms
                   == robot_default_config.communication.host_diagnostic_period_ms
            && config->communication.esp12f_status_period_ms
                   == robot_default_config.communication.esp12f_status_period_ms
            && ParameterManagement_Validate(&config->parameter.factory_defaults) != 0U
            && config->parameter.load_flash_on_boot <= 1U && config->parameter.persist_imu_calibration <= 1U
            && config->parameter.persist_current_zero <= 1U
            && config->display.welcome_duration_ms == robot_default_config.display.welcome_duration_ms
            && config->display.selfcheck_item_ms == robot_default_config.display.selfcheck_item_ms
            && config->display.error_blink_period_ms == robot_default_config.display.error_blink_period_ms
            && config->display.rpi_timeout_ms == robot_default_config.display.rpi_timeout_ms
            && config->display.line_timeout_ms == robot_default_config.display.line_timeout_ms
            && config->display.selfcheck_total_items == robot_default_config.display.selfcheck_total_items)
               ? 1U
               : 0U;
}
