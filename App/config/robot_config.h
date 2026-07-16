#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

#include "feature_config.h"
#include "task_config.h"

#include "command_management_config.h"
#include "communication_config.h"
#include "line_following_config.h"
#include "motion_control_config.h"
#include "parameter_management_config.h"
#include "power_management_config.h"
#include "safety_management_config.h"
#include "state_estimation_config.h"
#include "system_monitoring_config.h"
#include "teleoperation_config.h"

typedef struct
{
    uint32_t welcome_duration_ms;
    uint32_t selfcheck_item_ms;
    uint32_t error_blink_period_ms;
    uint32_t rpi_timeout_ms;
    uint32_t line_timeout_ms;
    uint8_t  selfcheck_total_items;
} app_display_config_t;

/** @brief App-owned aggregate of all Beta5 product and scheduling values. */
typedef struct
{
    motion_control_config_t       motion;
    state_estimation_config_t     state;
    power_management_config_t     power;
    safety_management_config_t    safety;
    command_management_config_t   command;
    teleoperation_config_t        teleoperation;
    line_following_config_t       line;
    communication_config_t        communication;
    parameter_management_config_t parameter;
    system_monitoring_config_t    system;
    app_task_config_t             tasks[APP_TASK_COUNT];
    app_display_config_t          display;
} robot_config_t;

/** @brief Return the immutable Beta4-compatible product configuration. */
const robot_config_t *RobotConfig_GetDefault(void);

/** @brief Validate frozen timing, command, and product configuration constraints. */
uint8_t RobotConfig_Validate(const robot_config_t *config);

#endif
