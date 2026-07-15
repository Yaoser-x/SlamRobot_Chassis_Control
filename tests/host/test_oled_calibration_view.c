#include "oled_calibration_view.h"
#include "system_snapshot_service.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void require_int(int condition, const char *message)
{
    if (!condition)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    oled_calibration_view_t view;
    view = OLED_CalibrationView_Build(SYSTEM_IMU_CAL_WAIT, 0U, 100U, SYSTEM_IMU_CAL_FAIL_MOTION, 10U, 0U);
    require_int(view.visible && strcmp(view.title, "IMU CAL WAIT") == 0, "wait view");
    require_int(strcmp(view.detail, "KEEP STILL") == 0, "stationary gate reason");
    view = OLED_CalibrationView_Build(SYSTEM_IMU_CAL_RUNNING, 25U, 100U, 0U, 10U, 0U);
    require_int(view.progress_percent == 25U, "sample progress");
    view = OLED_CalibrationView_Build(SYSTEM_IMU_CAL_DONE, 100U, 100U, 0U, 3999U, 1000U);
    require_int(view.visible && !view.return_to_normal, "terminal view retained");
    view = OLED_CalibrationView_Build(SYSTEM_IMU_CAL_FAILED, 10U, 100U, 1U, 4000U, 1000U);
    require_int(!view.visible && view.return_to_normal, "terminal auto return");
    view = OLED_CalibrationView_Build(SYSTEM_IMU_CAL_FAILED, 0U, 100U, SYSTEM_IMU_CAL_FAIL_READ, 3000U, 0U);
    require_int(!view.visible && view.return_to_normal, "terminal at tick zero auto returns");
    puts("PASS: OLED calibration view model");
    return 0;
}
