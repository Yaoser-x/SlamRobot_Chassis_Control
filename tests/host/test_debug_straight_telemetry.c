#include "debug_straight_telemetry.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void check(int condition, const char *message)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    chassis_service_snapshot_t chassis = {0};
    adc_monitor_state_t        adc     = {0};
    char                       json[768];
    char                       csv[768];
    char                       header[768];
    size_t                     json_len;
    size_t                     csv_len;

    chassis.left_requested_mps              = 0.12345f;
    chassis.right_requested_mps             = 0.12345f;
    chassis.left_target_mps                 = -0.29999f;
    chassis.right_target_mps                = 0.29999f;
    chassis.left_actual_mps                 = -0.22222f;
    chassis.right_actual_mps                = 0.11111f;
    chassis.left_speed_valid                = 1U;
    chassis.right_speed_valid               = 1U;
    chassis.pwm_saturated                   = 1U;
    chassis.control_source                  = 4U;
    chassis.straight_direction              = -1;
    chassis.straight_transition_distance_m  = 12345.678f;
    chassis.straight_trim_mps               = -0.075f;
    chassis.straight_wheel_correction_mps   = 99.99999f;
    chassis.straight_heading_error_deg      = -99999.0f;
    chassis.straight_heading_integral_deg_s = 30.0f;
    chassis.straight_heading_correction_mps = -99.99999f;
    chassis.straight_total_correction_mps   = 0.075f;
    chassis.straight_derated                = 1U;
    chassis.straight_out_of_range           = 1U;
    adc.battery_voltage                     = 99.999f;

    json_len = DebugStraightTelemetry_FormatJson(json, sizeof(json), &chassis, &adc);
    csv_len  = DebugStraightTelemetry_FormatCsv(csv, sizeof(csv), &chassis, &adc);
    check(json_len < sizeof(json) && csv_len < sizeof(csv), "worst suffix fits bounded buffers");
    check(strstr(json, "\"requested_mps\":0.12345") != NULL, "JSON requested speed");
    check(strstr(json, "\"actual_left_mps\":-0.22222") != NULL, "JSON aggregate left speed");
    check(strstr(json, "\"actual_right_mps\":0.11111") != NULL, "JSON aggregate right speed");
    check(strstr(json, "\"straight_direction\":-1") != NULL, "JSON direction");
    check(strstr(json, "\"battery_v\":99.999") != NULL, "JSON battery");
    check(strstr(json, "\"straight_derated\":1") != NULL, "JSON derating");
    check(strstr(json, "\"straight_out_of_range\":1") != NULL, "JSON compensation range");
    check(DebugStraightTelemetry_FormatCsvHeader(header, sizeof(header)) < sizeof(header),
          "CSV header fits bounded buffer");
    check(strstr(header,
                 "requested_mps,target_mps,actual_left_mps,actual_right_mps,speed_valid,pwm_saturated,source,battery_v")
              != NULL,
          "CSV stable field order");
    check(strstr(csv, "0.12345,0.00000,-0.22222,0.11111,1,1,4,99.999") == csv,
          "CSV and JSON derive common snapshot values");
    puts("PASS: debug straight telemetry");
    return 0;
}
