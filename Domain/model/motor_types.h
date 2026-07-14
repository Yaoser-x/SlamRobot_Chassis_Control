#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

typedef enum
{
    MOTOR_ID_M1    = 0,
    MOTOR_ID_M2    = 1,
    MOTOR_ID_M3    = 2,
    MOTOR_ID_M4    = 3,
    MOTOR_ID_COUNT = 4
} motor_id_t;

typedef enum
{
    MOTOR_SIDE_LEFT  = 0,
    MOTOR_SIDE_RIGHT = 1
} motor_side_t;

#endif
