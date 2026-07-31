#include "robot_config.h"

#include "motion_control_service.h"
#include "parameter_management_service.h"
#include "safety_management_service.h"
#include "teleoperation_service.h"

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
            .remote_velocity_requires_imu = 0U,
            .motion_permit_valid_ms       = 40U,
            .current_observe_a            = {1.5f, 1.5f, 1.5f, 1.5f},
            .current_fault_a              = {2.5f, 2.5f, 2.5f, 2.5f},
            .current_fault_debounce_ms    = 100U,
        },
    .command =
        {
            .host_timeout_ms        = 200U,
            .ps2_timeout_ms         = 500U,
            .esp12f_timeout_ms      = 500U,
            .line_timeout_ms        = 50U,
            .debug_timeout_ms       = 2000U,
            .remote_max_lifetime_ms = 2000U,
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
    .control_mode =
        {
            .takeover_enter_threshold  = 0.15f,
            .takeover_exit_threshold   = 0.10f,
            .manual_neutral_restore_ms = 2000U,
            .takeover_confirm_samples  = 3U,
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
                    .pid_kd                        = {0.0f, 0.0f, 0.0f, 0.0f},
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

static uint8_t RobotConfig_MotionValid(const motion_control_config_t *config)
{
    if (config == 0)
    {
        return 0U;
    }
    /* NaN/Inf guard: ordered comparisons with finite operands return false when either operand is NaN. */
    if (!(config->max_linear_mps > 0.0f) || config->max_linear_mps > 10.0f)
    {
        return 0U;
    }
    if (!(config->max_angular_rps > 0.0f) || config->max_angular_rps > 50.0f)
    {
        return 0U;
    }
    if (!(config->open_loop_full_mps > 0.0f) || config->open_loop_full_mps > config->max_linear_mps)
    {
        return 0U;
    }
    if (!(config->angular_epsilon_rps >= 0.0f))
    {
        return 0U;
    }
    if (!(config->speed_ramp_mps2 >= 0.0f) || config->speed_ramp_mps2 > 100.0f)
    {
        return 0U;
    }
    if (!(config->angular_ramp_rps2 >= 0.0f) || config->angular_ramp_rps2 > 500.0f)
    {
        return 0U;
    }
    if (!(config->maintenance_max_speed_mps >= 0.0f) || config->maintenance_max_speed_mps > config->max_linear_mps)
    {
        return 0U;
    }
    if (!(config->pid_correction_limit >= 0.0f) || config->pid_correction_limit > 2000.0f)
    {
        return 0U;
    }
    if (!(config->pid_stop_epsilon_mps >= 0.0f))
    {
        return 0U;
    }
    if (!(config->pid_direction_epsilon_mps >= 0.0f))
    {
        return 0U;
    }
    if (!(config->pid_feedback_min_target_mps >= 0.0f) || config->pid_feedback_min_target_mps > config->max_linear_mps)
    {
        return 0U;
    }
    if (!(config->pid_feedback_min_speed_mps >= 0.0f) || config->pid_feedback_min_speed_mps > config->max_linear_mps)
    {
        return 0U;
    }
    if (config->test_mode_lease_ms == 0UL || config->test_mode_lease_ms > 10000UL)
    {
        return 0U;
    }
    if (config->encoder_feedback_timeout_ms == 0UL || config->encoder_feedback_timeout_ms > 1000UL)
    {
        return 0U;
    }
    if (config->pid_feedback_loss_count == 0U)
    {
        return 0U;
    }
    if (config->pid_enabled > 1U)
    {
        return 0U;
    }
    if (config->wheel_speed_proportional_scale > 1U)
    {
        return 0U;
    }
    if (config->motor_current_limiter_observe_only > 1U)
    {
        return 0U;
    }
    if (config->current_soft_limit_enabled > 1U)
    {
        return 0U;
    }
    return 1U;
}

static uint8_t RobotConfig_TeleoperationValid(const teleoperation_config_t *config)
{
    if (config == 0)
    {
        return 0U;
    }
    if (!(config->linear_max_mps > 0.0f) || config->linear_max_mps > 10.0f)
    {
        return 0U;
    }
    if (!(config->angular_max_rps > 0.0f) || config->angular_max_rps > 50.0f)
    {
        return 0U;
    }
    if (!(config->dpad_linear_mps > 0.0f) || config->dpad_linear_mps > config->linear_max_mps)
    {
        return 0U;
    }
    if (!(config->dpad_angular_rps > 0.0f) || config->dpad_angular_rps > config->angular_max_rps)
    {
        return 0U;
    }
    if (!(config->manual_cancel_threshold >= 0.0f) || config->manual_cancel_threshold > 1.0f)
    {
        return 0U;
    }
    if (config->heading_quarter_timeout_ms == 0UL || config->heading_quarter_timeout_ms > 60000UL)
    {
        return 0U;
    }
    if (config->heading_full_timeout_ms == 0UL || config->heading_full_timeout_ms > 120000UL)
    {
        return 0U;
    }
    if (config->heading_imu_fresh_ms == 0UL || config->heading_imu_fresh_ms > 500UL)
    {
        return 0U;
    }
    if (config->idle_release_ms == 0UL || config->idle_release_ms > 30000UL)
    {
        return 0U;
    }
    if (config->axis_center < 0 || config->axis_center > 255)
    {
        return 0U;
    }
    if (config->axis_deadzone < 0 || config->axis_deadzone > 127)
    {
        return 0U;
    }
    if (config->offline_fail_limit == 0U)
    {
        return 0U;
    }
    /* Button masks are bit patterns — any value is legal architecturally. */
    return 1U;
}

static uint8_t RobotConfig_TasksValid(const robot_config_t *config)
{
    const robot_config_t *defaults;
    uint32_t              index;

    if (config == 0)
    {
        return 0U;
    }
    defaults = RobotConfig_GetDefault();
    for (index = 0U; index < (uint32_t)APP_TASK_COUNT; ++index)
    {
        const app_task_config_t *task   = &config->tasks[index];
        const app_task_config_t *frozen = &defaults->tasks[index];

        /* Beta5 freezes task period, stack, priority, and event-driven flag.
           Freertos.c creates tasks with hardcoded attributes, and every task
           loop reads RobotConfig_GetDefault()->tasks[...].period_ms.  Changing
           these values at runtime has no effect — this validation gate
           prevents the false impression that they are configurable. */
        if (task->period_ms != frozen->period_ms || task->stack_size_bytes != frozen->stack_size_bytes
            || (uint8_t)task->priority != (uint8_t)frozen->priority || task->event_driven != frozen->event_driven)
        {
            return 0U;
        }
    }
    for (index = 0U; index < SYSTEM_MONITORING_TASK_COUNT; ++index)
    {
        if (config->system.task_timeout_ms[index] != defaults->system.task_timeout_ms[index])
        {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t RobotConfig_SafetyValid(const safety_management_config_t *config)
{
    if (config == 0)
    {
        return 0U;
    }
    /* Battery thresholds: warn < critical violates the monitor's hysteresis. */
    if (!(config->battery_low_warn_v > 0.0f) || config->battery_low_warn_v > 30.0f)
    {
        return 0U;
    }
    if (!(config->battery_low_clear_v >= config->battery_low_warn_v) || config->battery_low_clear_v > 30.0f)
    {
        return 0U;
    }
    if (!(config->battery_critical_v > 0.0f) || config->battery_critical_v > config->battery_low_warn_v)
    {
        return 0U;
    }
    if (!(config->battery_recover_v > config->battery_critical_v) || config->battery_recover_v > 30.0f)
    {
        return 0U;
    }
    if (config->battery_critical_debounce_ms == 0UL || config->battery_critical_debounce_ms > 10000UL)
    {
        return 0U;
    }
    if (config->battery_recover_debounce_ms == 0UL || config->battery_recover_debounce_ms > 60000UL)
    {
        return 0U;
    }
    if (config->update_period_ms == 0UL || config->update_period_ms > 1000UL)
    {
        return 0U;
    }
    if (config->overcurrent_startup_blank_ms > 10000UL)
    {
        return 0U;
    }
    if (config->overcurrent_startup_rearm_ms > 60000UL)
    {
        return 0U;
    }
    if (config->battery_low_monitor_enabled > 1U)
    {
        return 0U;
    }
    if (config->overcurrent_fault_enabled > 1U)
    {
        return 0U;
    }
    if (config->remote_velocity_requires_imu > 1U)
    {
        return 0U;
    }
    if (config->motion_permit_valid_ms < config->update_period_ms
        || config->motion_permit_valid_ms > config->update_period_ms * 2UL)
    {
        return 0U;
    }
    if (config->current_fault_debounce_ms == 0U || config->current_fault_debounce_ms > 5000U)
    {
        return 0U;
    }
    for (uint8_t index = 0U; index < SAFETY_MANAGEMENT_MOTOR_COUNT; ++index)
    {
        if (!(config->current_observe_a[index] >= 0.0f) || config->current_observe_a[index] > 100.0f)
        {
            return 0U;
        }
        if (!(config->current_fault_a[index] >= config->current_observe_a[index])
            || config->current_fault_a[index] > 100.0f)
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

    /* Delegate per-capability validation to the owning Service so that
       App and Service never disagree on boundary values.  Each Service
       Init() reuses the same ValidateConfig() entry point. */
    if (RobotConfig_MotionValid(&config->motion) == 0U || MotionControl_ValidateConfig(&config->motion) == 0U
        || RobotConfig_TeleoperationValid(&config->teleoperation) == 0U
        || Teleoperation_ValidateConfig(&config->teleoperation) == 0U
        || ControlModeConfig_Validate(&config->control_mode) == 0U || RobotConfig_TasksValid(config) == 0U
        || RobotConfig_SafetyValid(&config->safety) == 0U || SafetyManagement_ValidateConfig(&config->safety) == 0U)
    {
        return 0U;
    }

    /* State Estimation: wheel/IMU freshness timeouts. */
    if (config->state.wheel_feedback_timeout_ms == 0UL || config->state.wheel_feedback_timeout_ms > 5000UL)
    {
        return 0U;
    }
    if (config->state.imu_fresh_timeout_ms == 0UL || config->state.imu_fresh_timeout_ms > 1000UL)
    {
        return 0U;
    }

    /* Power Management: stationary threshold and update period. */
    if (!(config->power.current_zero_max_speed_mps >= 0.0f) || config->power.current_zero_max_speed_mps > 1.0f)
    {
        return 0U;
    }
    if (config->power.update_period_ms == 0UL || config->power.update_period_ms > 1000UL)
    {
        return 0U;
    }

    /* Command Management: per-source timeouts. */
    if (config->command.host_timeout_ms == 0UL || config->command.host_timeout_ms > 10000UL)
    {
        return 0U;
    }
    if (config->command.ps2_timeout_ms == 0UL || config->command.ps2_timeout_ms > 10000UL)
    {
        return 0U;
    }
    if (config->command.esp12f_timeout_ms == 0UL || config->command.esp12f_timeout_ms > 10000UL)
    {
        return 0U;
    }
    if (config->command.line_timeout_ms == 0UL || config->command.line_timeout_ms > 5000UL)
    {
        return 0U;
    }
    if (config->command.debug_timeout_ms == 0UL || config->command.debug_timeout_ms > 30000UL)
    {
        return 0U;
    }
    if (config->command.remote_max_lifetime_ms < config->command.host_timeout_ms
        || config->command.remote_max_lifetime_ms < config->command.esp12f_timeout_ms
        || config->command.remote_max_lifetime_ms > 60000UL)
    {
        return 0U;
    }

    /* Line Following: angular limit and sensor freshness. */
    if (!(config->line.angular_max_rps > 0.0f) || config->line.angular_max_rps > 50.0f)
    {
        return 0U;
    }
    if (config->line.sensor_timeout_ms == 0UL || config->line.sensor_timeout_ms > 1000UL)
    {
        return 0U;
    }
    if (config->line.default_enabled > 1U)
    {
        return 0U;
    }
    if (config->line.detect_threshold_count == 0U || config->line.detect_threshold_count > 8U)
    {
        return 0U;
    }

    /* Communication: telemetry publication periods. */
    if (config->communication.host_status_period_ms == 0UL || config->communication.host_status_period_ms > 10000UL)
    {
        return 0U;
    }
    if (config->communication.host_imu_status_period_ms == 0UL
        || config->communication.host_imu_status_period_ms > 1000UL)
    {
        return 0U;
    }
    if (config->communication.host_diagnostic_period_ms == 0UL
        || config->communication.host_diagnostic_period_ms > 10000UL)
    {
        return 0U;
    }
    if (config->communication.esp12f_status_period_ms == 0UL || config->communication.esp12f_status_period_ms > 10000UL)
    {
        return 0U;
    }

    /* Parameter: delegate to ParameterManagement for the factory model. */
    if (ParameterManagement_Validate(&config->parameter.factory_defaults) == 0U)
    {
        return 0U;
    }
    if (config->parameter.load_flash_on_boot > 1U || config->parameter.persist_imu_calibration > 1U
        || config->parameter.persist_current_zero > 1U)
    {
        return 0U;
    }

    /* Display: OLED UI timing. */
    if (config->display.welcome_duration_ms == 0UL || config->display.welcome_duration_ms > 60000UL)
    {
        return 0U;
    }
    if (config->display.selfcheck_item_ms == 0UL || config->display.selfcheck_item_ms > 5000UL)
    {
        return 0U;
    }
    if (config->display.error_blink_period_ms == 0UL || config->display.error_blink_period_ms > 5000UL)
    {
        return 0U;
    }
    if (config->display.rpi_timeout_ms == 0UL || config->display.rpi_timeout_ms > 10000UL)
    {
        return 0U;
    }
    if (config->display.line_timeout_ms == 0UL || config->display.line_timeout_ms > 5000UL)
    {
        return 0U;
    }
    if (config->display.selfcheck_total_items == 0U || config->display.selfcheck_total_items > 32U)
    {
        return 0U;
    }

    return 1U;
}
