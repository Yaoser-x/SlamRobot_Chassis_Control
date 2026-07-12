#include "chassis_layout.h"

#include "bsp_config.h"

typedef struct
{
    /* CHASSIS_Mx_ENABLED。 */
    uint8_t enabled;

    /* CHASSIS_Mx_SIDE。 */
    motor_side_t side;

    /* CHASSIS_Mx_MOTOR_DIR。 */
    int8_t motor_direction;

    /* CHASSIS_Mx_ENCODER_DIR。 */
    int8_t encoder_direction;
} chassis_motor_layout_t;

/* 表驱动电机布局。改 CHASSIS_Mx_* 配置，不改分配逻辑。 */
static const chassis_motor_layout_t motor_layout[MOTOR_ID_COUNT] = {
    {CHASSIS_M1_ENABLED, CHASSIS_M1_SIDE, CHASSIS_M1_MOTOR_DIR, CHASSIS_M1_ENCODER_DIR},
    {CHASSIS_M2_ENABLED, CHASSIS_M2_SIDE, CHASSIS_M2_MOTOR_DIR, CHASSIS_M2_ENCODER_DIR},
    {CHASSIS_M3_ENABLED, CHASSIS_M3_SIDE, CHASSIS_M3_MOTOR_DIR, CHASSIS_M3_ENCODER_DIR},
    {CHASSIS_M4_ENABLED, CHASSIS_M4_SIDE, CHASSIS_M4_MOTOR_DIR, CHASSIS_M4_ENCODER_DIR},
};

static uint8_t ChassisLayout_IsValidMotor(motor_id_t motor)
{
    return ((uint32_t)motor < MOTOR_ID_COUNT) ? 1U : 0U;
}

uint8_t ChassisLayout_MotorEnabled(motor_id_t motor)
{
    if (ChassisLayout_IsValidMotor(motor) == 0U)
    {
        return 0U;
    }
    return (motor_layout[(uint32_t)motor].enabled != 0U) ? 1U : 0U;
}

motor_side_t ChassisLayout_MotorSide(motor_id_t motor)
{
    if (ChassisLayout_IsValidMotor(motor) == 0U)
    {
        return MOTOR_SIDE_LEFT;
    }
    return motor_layout[(uint32_t)motor].side;
}

int8_t ChassisLayout_MotorDirection(motor_id_t motor)
{
    if (ChassisLayout_IsValidMotor(motor) == 0U || motor_layout[(uint32_t)motor].motor_direction < 0)
    {
        return -1;
    }
    return 1;
}

int8_t ChassisLayout_EncoderDirection(motor_id_t motor)
{
    if (ChassisLayout_IsValidMotor(motor) == 0U || motor_layout[(uint32_t)motor].encoder_direction < 0)
    {
        return -1;
    }
    return 1;
}

uint8_t ChassisLayout_SideMotorCount(motor_side_t side)
{
    uint8_t count = 0U;

    for (uint8_t i = 0U; i < MOTOR_ID_COUNT; ++i)
    {
        if (ChassisLayout_MotorEnabled((motor_id_t)i) != 0U && ChassisLayout_MotorSide((motor_id_t)i) == side)
        {
            count++;
        }
    }
    return count;
}

uint8_t ChassisLayout_HasBothSides(void)
{
    return (ChassisLayout_SideMotorCount(MOTOR_SIDE_LEFT) != 0U && ChassisLayout_SideMotorCount(MOTOR_SIDE_RIGHT) != 0U)
               ? 1U
               : 0U;
}
