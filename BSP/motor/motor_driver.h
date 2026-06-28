#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  MOTOR_ID_M1 = 0,
  MOTOR_ID_M2 = 1,
  MOTOR_ID_M3 = 2,
  MOTOR_ID_M4 = 3,
  MOTOR_ID_COUNT = 4
} motor_id_t;

typedef enum
{
  MOTOR_SIDE_LEFT = 0,
  MOTOR_SIDE_RIGHT = 1
} motor_side_t;

typedef enum
{
  MOTOR_STOP_LOW_SIDE_BRAKE = 0
} motor_stop_mode_t;

typedef struct
{
  uint8_t fault_active[MOTOR_ID_COUNT];
  uint8_t sleep_enabled;
  uint8_t tim1_moe_active;
  uint8_t tim1_break_flag;
  uint8_t tim8_moe_active;
  uint8_t tim8_break_flag;
  uint32_t tim1_break_count;
  uint32_t tim8_break_count;
} motor_driver_state_t;

void MotorDriver_Init(void);
void MotorDriver_SetPermille(motor_id_t motor, int16_t permille);
void MotorDriver_SetSidePermille(motor_side_t side, int16_t permille);
void MotorDriver_SetInputPermille(motor_id_t motor, int16_t forward_permille, int16_t reverse_permille);
void MotorDriver_SetSideInputPermille(motor_side_t side, int16_t forward_permille, int16_t reverse_permille);
void MotorDriver_Stop(motor_id_t motor, motor_stop_mode_t mode);
void MotorDriver_StopSide(motor_side_t side, motor_stop_mode_t mode);
void MotorDriver_StopAll(motor_stop_mode_t mode);
void MotorDriver_UpdateFaults(void);
uint8_t MotorDriver_HasFault(void);
void MotorDriver_GetState(motor_driver_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
