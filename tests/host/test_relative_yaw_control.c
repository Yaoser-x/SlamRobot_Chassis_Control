#include "relative_yaw_controller.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static void require_close(float actual, float expected, float tolerance, const char *message)
{
    if (fabsf(actual - expected) > tolerance)
    {
        (void)fprintf(stderr, "FAIL: %s (actual=%f expected=%f)\n", message, actual, expected);
        exit(1);
    }
}

/* 航向累积改用陀螺仪速率积分：accumulated += yaw_rate_dps * dt */
static void test_gyro_rate_integration_basic(void)
{
    relative_yaw_control_t control;
    float                  angular_z = 0.0f;

    RelativeYawControl_Init(&control);
    require_int(RelativeYawControl_Start(&control, 90.0f, 0.0f, 100U, 4000U) != 0U, "relative yaw starts");

    /* 10ms at 200 deg/s = 2.0 deg */
    require_int(RelativeYawControl_Update(&control, 0.0f, 200.0f, 110U, &angular_z) != 0U,
                "relative yaw remains active");
    require_close(control.accumulated_delta_deg, 2.0f, 0.001f, "200dps * 10ms = 2.0 deg");

    /* another 20ms at 200 deg/s = 4.0 deg more, total 6.0 */
    (void)RelativeYawControl_Update(&control, 0.0f, 200.0f, 130U, &angular_z);
    require_close(control.accumulated_delta_deg, 6.0f, 0.001f, "200dps * 30ms = 6.0 deg total");
}

/* 长时间积分：模拟 360° 旋转 */
static void test_full_turn_gyro_integration(void)
{
    relative_yaw_control_t control;
    float                  angular_z = 0.0f;

    RelativeYawControl_Init(&control);
    (void)RelativeYawControl_Start(&control, 360.0f, 0.0f, 0U, 10000U);

    /* 每 100ms 累积 36°, 10 步 = 360° */
    uint32_t t = 100U;
    for (int i = 0; i < 10; i++)
    {
        (void)RelativeYawControl_Update(&control, 0.0f, 360.0f, t, &angular_z);
        t += 100U;
    }
    require_close(control.target_delta_deg, 360.0f, 0.001f, "full turn target stays 360");
    require_close(control.accumulated_delta_deg, 360.0f, 0.01f, "10 * 100ms * 360dps = 360 deg");
}

/* 输出限幅和 settling 检测 */
static void test_output_limit_and_settle_window(void)
{
    relative_yaw_control_t control;
    float                  angular_z = 0.0f;
    uint32_t               t;

    RelativeYawControl_Init(&control);
    (void)RelativeYawControl_Start(&control, 90.0f, 0.0f, 100U, 4000U);

    /* 大误差 → 输出应达到上限 1.5 rad/s */
    (void)RelativeYawControl_Update(&control, 0.0f, 0.0f, 110U, &angular_z);
    require_close(angular_z, 1.5f, 0.001f, "large error caps output at 1.5 rad/s");

    /* 累积到 89°（误差 1° 在 tolerance 内），但速率还高 */
    RelativeYawControl_Init(&control);
    (void)RelativeYawControl_Start(&control, 90.0f, 0.0f, 100U, 4000U);
    t = 100U;
    for (int i = 0; i < 20; i++)
    {
        /* 每步 890dps * 5ms = 4.45°, 20 步 = 89° */
        t += 5U;
        (void)RelativeYawControl_Update(&control, 0.0f, 890.0f, t, &angular_z);
    }
    require_close(control.accumulated_delta_deg, 89.0f, 0.1f, "accumulated reaches 89 deg");

    /* 高速率不应完成 */
    (void)RelativeYawControl_Update(&control, 0.0f, 10.0f, t, &angular_z);
    t += 10U;
    require_int(control.active != 0U, "high yaw rate prevents completion");

    /* 低速率 + 小误差 → 进入 settling */
    (void)RelativeYawControl_Update(&control, 0.0f, 0.0f, t, &angular_z);
    t += 10U;
    require_int(control.active != 0U, "settle window starts in tolerance");

    /* 99ms 稳定不应完成 */
    (void)RelativeYawControl_Update(&control, 0.0f, 0.0f, t, &angular_z);
    t += 89U;
    require_int(control.active != 0U, "99ms stable is not complete");

    /* 100ms 稳定应完成 */
    t += 1U;
    require_int(RelativeYawControl_Update(&control, 0.0f, 0.0f, t, &angular_z) == 0U, "100ms stable completes");
    require_int(control.end_reason == RELATIVE_YAW_END_COMPLETED, "completion reason recorded");
    require_close(angular_z, 0.0f, 0.001f, "completion commands zero angular speed");
}

/* 超时检测 + tick 回绕 */
static void test_timeout_and_tick_wrap(void)
{
    relative_yaw_control_t control;
    float                  angular_z = 0.0f;

    RelativeYawControl_Init(&control);
    (void)RelativeYawControl_Start(&control, -90.0f, 0.0f, 100U, 4000U);
    require_int(RelativeYawControl_Update(&control, 0.0f, 0.0f, 4099U, &angular_z) != 0U,
                "before timeout remains active");
    require_int(RelativeYawControl_Update(&control, 0.0f, 0.0f, 4100U, &angular_z) == 0U, "timeout stops relative yaw");
    require_int(control.end_reason == RELATIVE_YAW_END_TIMEOUT, "timeout reason recorded");
    require_close(angular_z, 0.0f, 0.001f, "timeout commands zero angular speed");

    /* tick wrap: start near max uint32, update after wrap */
    RelativeYawControl_Init(&control);
    (void)RelativeYawControl_Start(&control, 90.0f, 0.0f, 0xFFFFFFF0UL, 4000U);
    require_int(RelativeYawControl_Update(&control, 0.0f, 0.0f, 0x00000010UL, &angular_z) != 0U,
                "timeout arithmetic handles tick wrap");
}

/* dt 异常保护：间隔过大时用 fallback */
static void test_large_dt_uses_fallback(void)
{
    relative_yaw_control_t control;
    float                  angular_z = 0.0f;

    RelativeYawControl_Init(&control);
    (void)RelativeYawControl_Start(&control, 90.0f, 0.0f, 100U, 4000U);

    /* 501ms gap → dt clamped to 20ms fallback */
    (void)RelativeYawControl_Update(&control, 0.0f, 100.0f, 601U, &angular_z);
    require_close(control.accumulated_delta_deg, 2.0f, 0.01f, "large dt uses 20ms fallback: 100dps * 0.02s = 2.0 deg");
}

int main(void)
{
    test_gyro_rate_integration_basic();
    test_full_turn_gyro_integration();
    test_output_limit_and_settle_window();
    test_timeout_and_tick_wrap();
    test_large_dt_uses_fallback();
    (void)printf("PASS: relative yaw control host tests\n");
    return 0;
}
