#include "param_store.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

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
  require_close(params.max_linear_mps, 0.5f, 0.0001f, "default max linear");
  require_close(params.wheel_radius_m, 0.035f, 0.0001f, "default wheel radius");
  require_int(params.motor_dir[1] == -1, "default M2 motor direction");
  require_int(params.encoder_dir[2] == -1, "default M3 encoder direction");
  require_int(ParamStore_Validate(&params) == 1U, "defaults validate");
}

static void test_named_float_set_get_rejects_unsafe_values(void)
{
  float value = 0.0f;
  param_store_t params;

  ParamStore_Defaults(&params);

  require_int(ParamStore_SetFloat(&params, "max_linear_mps", 0.42f) == 1U,
              "set known float");
  require_int(ParamStore_GetFloat(&params, "max_linear_mps", &value) == 1U,
              "get known float");
  require_close(value, 0.42f, 0.0001f, "set/get value");
  require_int(ParamStore_SetFloat(&params, "max_linear_mps", -0.1f) == 0U,
              "reject negative max linear");
  require_int(ParamStore_SetFloat(&params, "not_a_param", 1.0f) == 0U,
              "reject unknown param");
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

int main(void)
{
  test_defaults_match_safe_runtime_values();
  test_named_float_set_get_rejects_unsafe_values();
  test_global_copy_is_isolated();

  (void)printf("PASS: param store host tests\n");
  return 0;
}
