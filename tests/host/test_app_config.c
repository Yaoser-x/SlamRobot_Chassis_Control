#include "robot_config.h"

#include "bsp_config.h"
#include "control_config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        fflush(stderr);
        _Exit(1);
    }
}

static void test_default_matches_beta4_constants(void)
{
    const robot_config_t *config = RobotConfig_GetDefault();

    require_int(config != 0, "default aggregate exists");
    require_int(config->motion.max_linear_mps == CHASSIS_MAX_LINEAR_MPS, "motion max linear");
    require_int(config->motion.max_angular_rps == CHASSIS_MAX_ANGULAR_RPS, "motion max angular");
    require_int(config->motion.open_loop_full_mps == CHASSIS_OPENLOOP_FULL_MPS, "motion open loop scale");
    require_int(config->motion.speed_ramp_mps2 == CHASSIS_SPEED_RAMP_MPS2, "motion linear ramp");
    require_int(config->motion.angular_ramp_rps2 == CHASSIS_ANGULAR_RAMP_RPS2, "motion angular ramp");
    require_int(config->motion.test_mode_lease_ms == CHASSIS_TEST_MODE_LEASE_MS, "motion test lease");
    require_int(config->motion.encoder_feedback_timeout_ms == CHASSIS_ENCODER_FEEDBACK_TIMEOUT_MS,
                "motion encoder timeout");
    require_int(config->state.wheel_feedback_timeout_ms == CHASSIS_ENCODER_FEEDBACK_TIMEOUT_MS, "state wheel timeout");
    require_int(config->power.current_zero_max_speed_mps == CHASSIS_MAINTENANCE_MAX_SPEED_MPS,
                "power stationary speed");

    require_int(config->command.host_timeout_ms == CONTROL_TIMEOUT_UPPER_MS, "host timeout");
    require_int(config->command.ps2_timeout_ms == CONTROL_TIMEOUT_PS2_MS, "PS2 timeout");
    require_int(config->command.esp12f_timeout_ms == CONTROL_TIMEOUT_ESP12F_MS, "ESP timeout");
    require_int(config->command.line_timeout_ms == CONTROL_TIMEOUT_LINE_MS, "line timeout");
    require_int(config->command.debug_timeout_ms == CONTROL_TIMEOUT_DEBUG_MS, "debug timeout");

    require_int(config->safety.battery_low_warn_v == BATTERY_LOW_WARN_V, "battery warning");
    require_int(config->safety.battery_low_clear_v == BATTERY_LOW_CLEAR_V, "battery warning clear");
    require_int(config->safety.battery_critical_v == BATTERY_CRITICAL_V, "battery critical");
    require_int(config->safety.battery_recover_v == BATTERY_RECOVER_V, "battery recovery");
    require_int(config->safety.battery_critical_debounce_ms == BATTERY_CRITICAL_DEBOUNCE_MS,
                "battery critical debounce");
    require_int(config->safety.battery_recover_debounce_ms == BATTERY_RECOVER_DEBOUNCE_MS, "battery recovery debounce");
    require_int(config->safety.update_period_ms == CHASSIS_ADC_PERIOD_MS, "safety update period");
    require_int(config->safety.overcurrent_startup_blank_ms == MOTOR_OVERCURRENT_STARTUP_BLANK_MS,
                "overcurrent startup blank");
    require_int(config->safety.overcurrent_startup_rearm_ms == MOTOR_OVERCURRENT_STARTUP_REARM_MS,
                "overcurrent startup rearm");
    require_int(config->safety.overcurrent_fault_enabled == MOTOR_ADC_OVERCURRENT_FAULT_ENABLED,
                "overcurrent fault policy");

    require_int(config->teleoperation.linear_max_mps == PS2_LINEAR_MAX_MPS, "PS2 linear max");
    require_int(config->teleoperation.angular_max_rps == PS2_ANGULAR_MAX_RPS, "PS2 angular max");
    require_int(config->teleoperation.idle_release_ms == PS2_IDLE_RELEASE_MS, "PS2 idle release");
    require_int(config->teleoperation.heading_imu_fresh_ms == PS2_HEADING_IMU_FRESH_MS, "PS2 IMU freshness");
    require_int(config->control_mode.takeover_enter_threshold == 0.15f, "PS2 takeover enter threshold");
    require_int(config->control_mode.takeover_exit_threshold == 0.10f, "PS2 takeover exit threshold");
    require_int(config->control_mode.takeover_confirm_samples == 3U, "PS2 takeover debounce samples");
    require_int(config->control_mode.manual_neutral_restore_ms == 2000U, "PS2 neutral restore duration");
    require_int(config->line.angular_max_rps == LINE_ANGULAR_MAX_RPS, "line angular limit");
    require_int(config->line.sensor_timeout_ms == LINE_SENSOR_TIMEOUT_MS, "line freshness");
    require_int(config->command.remote_max_lifetime_ms == 2000U, "remote command hard lifetime");
    require_int(config->communication.host_status_period_ms == UPPER_UART_STATUS_PERIOD_MS, "host status period");
    require_int(config->communication.host_imu_status_period_ms == UPPER_IMU_STATUS_PERIOD_MS, "host IMU period");
    require_int(config->communication.host_diagnostic_period_ms == UPPER_DIAGNOSTIC_PERIOD_MS,
                "host diagnostic period");
    require_int(config->communication.esp12f_status_period_ms == ESP12F_STATUS_PERIOD_MS, "ESP status period");
    require_int(config->parameter.factory_defaults.wheel_radius_m == CHASSIS_WHEEL_RADIUS_M,
                "parameter factory wheel radius");
    require_int(config->parameter.factory_defaults.track_width_m == CHASSIS_TRACK_WIDTH_M,
                "parameter factory track width");
    require_int(config->parameter.factory_defaults.pid_kp[MOTOR_ID_M2] == CHASSIS_PID_KP_M2,
                "parameter factory M2 PID");
    require_int(config->parameter.factory_defaults.pid_kp[MOTOR_ID_M3] == CHASSIS_PID_KP_M3,
                "parameter factory M3 PID");
    require_int(config->parameter.factory_defaults.motor_dir[MOTOR_ID_M2] == CHASSIS_M2_MOTOR_DIR,
                "parameter factory M2 direction");
    require_int(config->parameter.factory_defaults.encoder_dir[MOTOR_ID_M3] == CHASSIS_M3_ENCODER_DIR,
                "parameter factory M3 encoder direction");

    require_int(config->tasks[APP_TASK_SAFETY].period_ms == CHASSIS_ADC_PERIOD_MS, "safety period");
    require_int(config->tasks[APP_TASK_MOTOR].period_ms == CHASSIS_CONTROL_PERIOD_MS, "motor period");
    require_int(config->tasks[APP_TASK_HOST].period_ms == UPPER_UART_TASK_PERIOD_MS, "host period");
    require_int(config->tasks[APP_TASK_IMU].period_ms == CHASSIS_IMU_PERIOD_MS, "IMU timeout period");
    require_int(config->tasks[APP_TASK_IMU].event_driven == 1U, "IMU remains event driven");
    require_int(config->tasks[APP_TASK_LINE].period_ms == CHASSIS_LINE_PERIOD_MS, "line period");
    require_int(config->tasks[APP_TASK_ESP12F].period_ms == CHASSIS_ESP12F_PERIOD_MS, "ESP period");
    require_int(config->tasks[APP_TASK_PS2].period_ms == CHASSIS_PS2_PERIOD_MS, "PS2 period");
    require_int(config->tasks[APP_TASK_LED].period_ms == CHASSIS_LED_PERIOD_MS, "LED period");
    require_int(config->tasks[APP_TASK_OLED].period_ms == OLED_TASK_PERIOD_MS, "OLED period");
    require_int(config->tasks[APP_TASK_DEBUG].period_ms == 10U, "debug period");
    require_int(config->display.welcome_duration_ms == OLED_WELCOME_DURATION_MS, "welcome duration");
    require_int(config->display.selfcheck_item_ms == OLED_SELFCHECK_ITEM_MS, "selfcheck item period");
    require_int(config->display.selfcheck_total_items == OLED_SELFCHECK_TOTAL_ITEMS, "selfcheck item count");
    require_int(CHASSIS_TRACK_WIDTH_M == 0.176f, "track width compatibility remains 0.176 m");
}

static void test_validation_accepts_legal_config_and_rejects_illegal(void)
{
    robot_config_t copy = *RobotConfig_GetDefault();

    /* Default copy is legal. */
    require_int(RobotConfig_Validate(&copy) == 1U, "default config validates");

    /* Legal deviation (within range) is accepted. */
    copy.command.host_timeout_ms = 500U;
    require_int(RobotConfig_Validate(&copy) == 1U, "legal command timeout within range is accepted");

    copy                                = *RobotConfig_GetDefault();
    copy.command.remote_max_lifetime_ms = copy.command.host_timeout_ms - 1U;
    require_int(RobotConfig_Validate(&copy) == 0U, "remote maximum lifetime cannot be shorter than source lease");

    /* Illegal: zero max linear speed is rejected. */
    copy                       = *RobotConfig_GetDefault();
    copy.motion.max_linear_mps = 0.0f;
    require_int(RobotConfig_Validate(&copy) == 0U, "zero max linear speed is rejected");

    /* Illegal: battery critical above low warn is rejected. */
    copy                           = *RobotConfig_GetDefault();
    copy.safety.battery_critical_v = 12.0f;
    copy.safety.battery_low_warn_v = 10.0f;
    require_int(RobotConfig_Validate(&copy) == 0U, "battery critical above low warn is rejected");

    /* Illegal: zero task period is rejected. */
    copy                                 = *RobotConfig_GetDefault();
    copy.tasks[APP_TASK_MOTOR].period_ms = 0UL;
    require_int(RobotConfig_Validate(&copy) == 0U, "zero task period is rejected");

    /* Illegal: invalid factory parameters. */
    copy                                           = *RobotConfig_GetDefault();
    copy.parameter.factory_defaults.wheel_radius_m = 0.0f;
    require_int(RobotConfig_Validate(&copy) == 0U, "invalid injected factory parameters are rejected");

    require_int(RobotConfig_Validate(0) == 0U, "null config is rejected");

    /* Beta5.3.2 boundary alignment: App & Service must agree on rejections. */
    copy                             = *RobotConfig_GetDefault();
    copy.motion.pid_correction_limit = 0.0f;
    require_int(RobotConfig_Validate(&copy) == 0U, "pid correction limit == 0 is rejected by Service");

    copy                             = *RobotConfig_GetDefault();
    copy.teleoperation.axis_deadzone = 127;
    require_int(RobotConfig_Validate(&copy) == 0U, "axis deadzone == 127 is rejected by Service");

    copy                                      = *RobotConfig_GetDefault();
    copy.control_mode.takeover_exit_threshold = copy.control_mode.takeover_enter_threshold;
    require_int(RobotConfig_Validate(&copy) == 0U, "takeover hysteresis requires distinct enter and exit thresholds");

    copy                            = *RobotConfig_GetDefault();
    copy.safety.battery_low_clear_v = copy.safety.battery_low_warn_v;
    require_int(RobotConfig_Validate(&copy) == 0U, "battery clear == warn is rejected by Service");

    copy                                     = *RobotConfig_GetDefault();
    copy.safety.overcurrent_startup_blank_ms = 0UL;
    require_int(RobotConfig_Validate(&copy) == 0U, "overcurrent startup blank == 0 is rejected by Service");

    /* Beta5.3.2 task freeze: any deviation from frozen defaults is rejected. */
    copy                                 = *RobotConfig_GetDefault();
    copy.tasks[APP_TASK_MOTOR].period_ms = 20UL;
    require_int(RobotConfig_Validate(&copy) == 0U, "task period deviation is rejected");

    copy                                         = *RobotConfig_GetDefault();
    copy.tasks[APP_TASK_SAFETY].stack_size_bytes = 8192UL;
    require_int(RobotConfig_Validate(&copy) == 0U, "task stack deviation is rejected");

    copy                                = *RobotConfig_GetDefault();
    copy.tasks[APP_TASK_DEBUG].priority = APP_TASK_PRIORITY_HIGH;
    require_int(RobotConfig_Validate(&copy) == 0U, "task priority deviation is rejected");
}

int main(void)
{
    test_default_matches_beta4_constants();
    test_validation_accepts_legal_config_and_rejects_illegal();
    puts("app config tests passed");
    return 0;
}
