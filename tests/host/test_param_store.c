#include "param_store.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t fake_primask;

uint32_t __get_PRIMASK(void)
{
    return fake_primask;
}

void __disable_irq(void)
{
    fake_primask = 1U;
}

void __set_PRIMASK(uint32_t primask)
{
    fake_primask = primask;
}

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        (void)printf("FAIL: %s\n", message);
        __builtin_exit(1);
    }
}

static void require_close(float actual, float expected, float tolerance, const char *message)
{
    if (fabsf(actual - expected) > tolerance)
    {
        (void)printf("FAIL: %s actual=%f expected=%f\n", message, actual, expected);
        __builtin_exit(1);
    }
}

static void test_defaults_match_safe_runtime_values(void)
{
    param_store_t params;

    ParamStore_Defaults(&params);

    require_int(params.version == PARAM_STORE_VERSION, "default version");
    require_int(PARAM_STORE_VERSION == 3UL, "parameter schema version three");
    require_close(params.max_linear_mps, 0.5f, 0.0001f, "default max linear");
    require_close(params.wheel_radius_m, 0.035f, 0.0001f, "default wheel radius");
    require_int(params.motor_dir[1] == -1, "default M2 motor direction");
    require_int(params.encoder_dir[2] == -1, "default M3 encoder direction");
    require_int(params.line_threshold_raw[0] == 500U, "default line threshold");
    require_int(params.line_active_low == 1U, "default black line polarity");
    require_close(params.line_kp, 0.6f, 0.0001f, "default line kp");
    require_close(params.current_fault_a[0], 2.5f, 0.0001f, "default current fault threshold");
    require_close(params.straight_wheel_coupling_gain, 0.30f, 0.0001f, "straight coupling default 0.30");
    require_close(params.straight_heading_kp, 0.0f, 0.0001f, "straight heading kp defaults safe off");
    require_close(params.straight_heading_ki, 0.0f, 0.0001f, "straight heading ki defaults safe off");
    require_close(params.straight_max_speed_mps, 0.30f, 0.0001f, "straight compensation range defaults to HIL ceiling");
    require_int(params.straight_heading_hold_enabled == 0U, "straight heading hold disabled until HIL");
    require_int(ParamStore_Validate(&params) == 1U, "defaults validate");
}

static void test_named_float_set_get_rejects_unsafe_values(void)
{
    float         value = 0.0f;
    param_store_t params;

    ParamStore_Defaults(&params);

    require_int(ParamStore_SetFloat(&params, "max_linear_mps", 0.42f) == 1U, "set known float");
    require_int(ParamStore_GetFloat(&params, "max_linear_mps", &value) == 1U, "get known float");
    require_close(value, 0.42f, 0.0001f, "set/get value");
    require_int(ParamStore_SetFloat(&params, "max_linear_mps", -0.1f) == 0U, "reject negative max linear");
    require_int(ParamStore_SetFloat(&params, "not_a_param", 1.0f) == 0U, "reject unknown param");

    require_int(ParamStore_SetFloat(&params, "straight_trim_forward_015_mps", -0.10f) == 1U, "accept trim lower bound");
    require_int(ParamStore_SetFloat(&params, "straight_trim_reverse_030_mps", 0.10f) == 1U, "accept trim upper bound");
    require_int(ParamStore_SetFloat(&params, "straight_trim_forward_030_mps", 0.101f) == 0U, "reject trim above bound");
    require_int(ParamStore_SetFloat(&params, "straight_heading_ki", 0.02f) == 1U, "accept heading ki upper bound");
    require_int(ParamStore_SetFloat(&params, "straight_heading_ki", 0.021f) == 0U, "reject heading ki above bound");
    require_int(ParamStore_SetFloat(&params, "straight_heading_integral_limit_deg_s", 30.0f) == 1U,
                "accept heading integral upper bound");
    require_int(ParamStore_SetFloat(&params, "straight_max_speed_mps", 0.05f) == 1U,
                "accept straight speed lower bound");
    require_int(ParamStore_SetFloat(&params, "straight_max_speed_mps", 0.049f) == 0U,
                "reject straight speed below bound");
}

static void test_named_int_set_get_and_range_check(void)
{
    int32_t       value = -1;
    param_store_t params;

    ParamStore_Defaults(&params);

    /* ----- straight_heading_hold_enabled (u8, 0..1) ----- */
    require_int(ParamStore_SetInt(&params, "straight_heading_hold_enabled", 1) == 1U,
                "set heading hold enabled via int binding");
    require_int(params.straight_heading_hold_enabled == 1U, "heading hold stored as 1");
    require_int(ParamStore_GetInt(&params, "straight_heading_hold_enabled", &value) == 1U,
                "get heading hold via int binding");
    require_int(value == 1, "heading hold int value = 1");
    require_int(ParamStore_SetInt(&params, "straight_heading_hold_enabled", 2) == 0U,
                "reject heading hold = 2 (out of range)");
    require_int(ParamStore_SetInt(&params, "straight_heading_hold_enabled", -1) == 0U,
                "reject heading hold = -1 (out of range)");

    /* Float API no longer resolves this param */
    {
        float fv = 0.0f;
        require_int(ParamStore_GetFloat(&params, "straight_heading_hold_enabled", &fv) == 0U,
                    "float getter rejects int-only param");
        require_int(ParamStore_SetFloat(&params, "straight_heading_hold_enabled", 1.0f) == 0U,
                    "float setter rejects int-only param");
    }

    /* ----- line_active_low (u8, 0..1) ----- */
    require_int(ParamStore_GetInt(&params, "line_active_low", &value) == 1U, "get line_active_low");
    require_int(value == 1, "default line_active_low = 1");
    require_int(ParamStore_SetInt(&params, "line_active_low", 0) == 1U, "set line_active_low = 0");
    require_int(ParamStore_SetInt(&params, "line_active_low", 2) == 0U, "reject line_active_low = 2");

    /* ----- current_fault_debounce_ms (u16, 20..2000) ----- */
    require_int(ParamStore_SetInt(&params, "current_fault_debounce_ms", 200) == 1U,
                "set current_fault_debounce_ms = 200");
    require_int(ParamStore_GetInt(&params, "current_fault_debounce_ms", &value) == 1U, "get current_fault_debounce_ms");
    require_int(value == 200, "current_fault_debounce_ms = 200");
    require_int(ParamStore_SetInt(&params, "current_fault_debounce_ms", 10) == 0U,
                "reject current_fault_debounce_ms = 10 (< 20)");
    require_int(ParamStore_SetInt(&params, "current_fault_debounce_ms", 3000) == 0U,
                "reject current_fault_debounce_ms = 3000 (> 2000)");

    /* ----- unknown name ----- */
    require_int(ParamStore_SetInt(&params, "not_a_param", 1) == 0U, "reject unknown int param");
    require_int(ParamStore_GetInt(&params, "not_a_param", &value) == 0U, "get unknown int param returns 0");
}

static void test_global_copy_is_isolated(void)
{
    param_store_t params;
    param_store_t roundtrip;

    ParamStore_Defaults(&params);
    params.speed_ramp_mps2 = 1.5f;
    ParamStore_Set(&params);
    params.speed_ramp_mps2 = 9.0f;
    ParamStore_Get(&roundtrip);

    require_close(roundtrip.speed_ramp_mps2, 1.5f, 0.0001f, "global store copies input");
}

static void test_snapshot_generation_is_consistent(void)
{
    param_store_t params;
    param_store_t snapshot;
    uint32_t      generation_before;
    uint32_t      generation_after;

    fake_primask = 0U;
    ParamStore_SetDefaults();
    generation_before    = ParamStore_GetSnapshot(&snapshot);
    params               = snapshot;
    params.track_width_m = 0.190f;

    require_int(ParamStore_Set(&params) != 0U, "valid runtime update accepted");
    generation_after = ParamStore_GetSnapshot(&snapshot);

    require_int(generation_after > generation_before, "generation advances");
    require_close(snapshot.track_width_m, 0.190f, 0.0001f, "snapshot matches generation");
    require_int(fake_primask == 0U, "snapshot restores interrupt state");
}

static void test_invalid_set_does_not_advance_generation(void)
{
    param_store_t params;
    uint32_t      generation_before;
    uint32_t      generation_after;

    generation_before     = ParamStore_GetSnapshot(&params);
    params.wheel_radius_m = 0.0f;
    require_int(ParamStore_Set(&params) == 0U, "invalid runtime update rejected");
    generation_after = ParamStore_GetSnapshot(&params);
    require_int(generation_after == generation_before, "rejected update preserves generation");
}

static void test_all_direction_fields_publish_in_one_snapshot(void)
{
    param_store_t params;
    param_store_t snapshot;
    const int8_t  motor_dir[MOTOR_ID_COUNT]   = {1, -1, 1, -1};
    const int8_t  encoder_dir[MOTOR_ID_COUNT] = {-1, 1, -1, 1};

    ParamStore_Defaults(&params);
    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        params.motor_dir[i]   = motor_dir[i];
        params.encoder_dir[i] = encoder_dir[i];
    }
    require_int(ParamStore_Set(&params) != 0U, "all direction combinations accepted");
    (void)ParamStore_GetSnapshot(&snapshot);
    require_int(memcmp(snapshot.motor_dir, motor_dir, sizeof(motor_dir)) == 0,
                "four motor directions share one atomic snapshot");
    require_int(memcmp(snapshot.encoder_dir, encoder_dir, sizeof(encoder_dir)) == 0,
                "four encoder directions share one atomic snapshot");
}

int main(void)
{
    test_defaults_match_safe_runtime_values();
    test_named_float_set_get_rejects_unsafe_values();
    test_named_int_set_get_and_range_check();
    test_global_copy_is_isolated();
    test_snapshot_generation_is_consistent();
    test_invalid_set_does_not_advance_generation();
    test_all_direction_fields_publish_in_one_snapshot();

    (void)printf("PASS: param store host tests\n");
    return 0;
}
