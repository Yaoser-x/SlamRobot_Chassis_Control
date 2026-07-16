#include "debug_straight_telemetry.h"

#include <stdio.h>

static float SideAverage(float left, float right)
{
    return 0.5f * (left + right);
}

size_t DebugStraightTelemetry_FormatJson(char                           *buffer,
                                         size_t                          buffer_size,
                                         const motion_control_status_t  *chassis,
                                         const power_adc_driver_state_t *adc)
{
    int written;

    if ((buffer == NULL) || (buffer_size == 0U) || (chassis == NULL) || (adc == NULL))
    {
        return 0U;
    }
    written = snprintf(buffer,
                       buffer_size,
                       ",\"requested_mps\":%.5f,\"target_mps\":%.5f,"
                       "\"actual_left_mps\":%.5f,\"actual_right_mps\":%.5f,"
                       "\"speed_valid\":%u,\"pwm_saturated\":%u,\"source\":%u,\"battery_v\":%.3f,"
                       "\"straight_direction\":%d,\"straight_transition_distance_m\":%.5f,"
                       "\"straight_trim_mps\":%.5f,\"straight_wheel_correction_mps\":%.5f,"
                       "\"straight_heading_error_deg\":%.5f,"
                       "\"straight_heading_integral_deg_s\":%.5f,"
                       "\"straight_heading_correction_mps\":%.5f,"
                       "\"straight_total_correction_mps\":%.5f,\"straight_derated\":%u,"
                       "\"straight_out_of_range\":%u",
                       SideAverage(chassis->left_requested_mps, chassis->right_requested_mps),
                       SideAverage(chassis->left_target_mps, chassis->right_target_mps),
                       chassis->left_actual_mps,
                       chassis->right_actual_mps,
                       (uint8_t)(chassis->left_speed_valid && chassis->right_speed_valid),
                       chassis->pwm_saturated,
                       chassis->control_source,
                       adc->battery_voltage,
                       chassis->straight_direction,
                       chassis->straight_transition_distance_m,
                       chassis->straight_trim_mps,
                       chassis->straight_wheel_correction_mps,
                       chassis->straight_heading_error_deg,
                       chassis->straight_heading_integral_deg_s,
                       chassis->straight_heading_correction_mps,
                       chassis->straight_total_correction_mps,
                       chassis->straight_derated,
                       chassis->straight_out_of_range);
    return (written > 0) ? (size_t)written : 0U;
}

size_t DebugStraightTelemetry_FormatCsvHeader(char *buffer, size_t buffer_size)
{
    int written;

    if ((buffer == NULL) || (buffer_size == 0U))
    {
        return 0U;
    }
    written = snprintf(buffer,
                       buffer_size,
                       "requested_mps,target_mps,actual_left_mps,actual_right_mps,"
                       "speed_valid,pwm_saturated,source,battery_v,"
                       "straight_direction,straight_transition_distance_m,straight_trim_mps,"
                       "straight_wheel_correction_mps,straight_heading_error_deg,"
                       "straight_heading_integral_deg_s,straight_heading_correction_mps,"
                       "straight_total_correction_mps,straight_derated,straight_out_of_range");
    return (written > 0) ? (size_t)written : 0U;
}

size_t DebugStraightTelemetry_FormatCsv(char                           *buffer,
                                        size_t                          buffer_size,
                                        const motion_control_status_t  *chassis,
                                        const power_adc_driver_state_t *adc)
{
    int written;

    if ((buffer == NULL) || (buffer_size == 0U) || (chassis == NULL) || (adc == NULL))
    {
        return 0U;
    }
    written = snprintf(buffer,
                       buffer_size,
                       "%.5f,%.5f,%.5f,%.5f,%u,%u,%u,%.3f,%d,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%u,%u",
                       SideAverage(chassis->left_requested_mps, chassis->right_requested_mps),
                       SideAverage(chassis->left_target_mps, chassis->right_target_mps),
                       chassis->left_actual_mps,
                       chassis->right_actual_mps,
                       (uint8_t)(chassis->left_speed_valid && chassis->right_speed_valid),
                       chassis->pwm_saturated,
                       chassis->control_source,
                       adc->battery_voltage,
                       chassis->straight_direction,
                       chassis->straight_transition_distance_m,
                       chassis->straight_trim_mps,
                       chassis->straight_wheel_correction_mps,
                       chassis->straight_heading_error_deg,
                       chassis->straight_heading_integral_deg_s,
                       chassis->straight_heading_correction_mps,
                       chassis->straight_total_correction_mps,
                       chassis->straight_derated,
                       chassis->straight_out_of_range);
    return (written > 0) ? (size_t)written : 0U;
}
