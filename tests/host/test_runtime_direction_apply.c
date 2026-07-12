#include "direction_apply.h"

#include <stdio.h>
#include <stdlib.h>

static void check(int ok, const char *message)
{
    if (!ok)
    {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

int main(void)
{
    const int8_t motor_dir[4]   = {1, -1, 1, -1};
    const int8_t encoder_dir[4] = {-1, 1, -1, 1};
    for (uint8_t motor = 0U; motor < 4U; ++motor)
    {
        check(DirectionApply_Signed(300, motor_dir[motor]) == 300 * motor_dir[motor],
              "motor runtime direction consumed");
        check(DirectionApply_Signed(10, encoder_dir[motor]) == 10 * encoder_dir[motor],
              "encoder runtime direction consumed");
    }
    puts("PASS: four-channel runtime direction consumption");
    return 0;
}
