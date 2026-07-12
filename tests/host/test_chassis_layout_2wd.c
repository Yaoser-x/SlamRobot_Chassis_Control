#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "chassis_layout.h"

static void require_int(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)printf("FAIL: %s\n", message);
        _Exit(1);
    }
}

int main(void)
{
    require_int(ChassisLayout_MotorEnabled(MOTOR_ID_M1) == 0U, "m1 disabled");
    require_int(ChassisLayout_MotorEnabled(MOTOR_ID_M2) != 0U, "m2 enabled");
    require_int(ChassisLayout_MotorEnabled(MOTOR_ID_M3) != 0U, "m3 enabled");
    require_int(ChassisLayout_MotorEnabled(MOTOR_ID_M4) == 0U, "m4 disabled");
    require_int(ChassisLayout_SideMotorCount(MOTOR_SIDE_LEFT) == 1U, "left count");
    require_int(ChassisLayout_SideMotorCount(MOTOR_SIDE_RIGHT) == 1U, "right count");
    require_int(ChassisLayout_HasBothSides() != 0U, "both sides present");
    (void)printf("PASS: chassis layout 2wd host test\n");
    return 0;
}
