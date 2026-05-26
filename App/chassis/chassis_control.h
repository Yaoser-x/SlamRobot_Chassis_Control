#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdint.h>

#include "motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float motor_target_mps[MOTOR_ID_COUNT];
  float motor_requested_mps[MOTOR_ID_COUNT];
  float motor_actual_mps[MOTOR_ID_COUNT];
  float motor_error_mps[MOTOR_ID_COUNT];
  int16_t motor_output_permille[MOTOR_ID_COUNT];
  uint8_t motor_speed_valid[MOTOR_ID_COUNT];
  uint8_t motor_pid_active[MOTOR_ID_COUNT];
  uint8_t motor_feedback_lost[MOTOR_ID_COUNT];
  uint8_t motor_current_limited[MOTOR_ID_COUNT];
  float left_target_mps;
  float right_target_mps;
  float left_requested_mps;
  float right_requested_mps;
  float left_actual_mps;
  float right_actual_mps;
  float left_error_mps;
  float right_error_mps;
  int16_t left_output_permille;
  int16_t right_output_permille;
  uint8_t output_enabled;
  uint8_t left_speed_valid;
  uint8_t right_speed_valid;
  uint8_t left_pid_active;
  uint8_t right_pid_active;
  uint8_t left_feedback_lost;
  uint8_t right_feedback_lost;
  uint8_t left_current_limited;
  uint8_t right_current_limited;
} chassis_control_state_t;

void ChassisControl_Init(void);
void ChassisControl_Step(uint32_t now_ms);
void ChassisControl_EmergencyStop(void);
void ChassisControl_OpenLoopTest(int16_t left_permille, int16_t right_permille);
void ChassisControl_RawInputTest(int16_t left_forward_permille,
                                 int16_t left_reverse_permille,
                                 int16_t right_forward_permille,
                                 int16_t right_reverse_permille);
void ChassisControl_RawMotorInputTest(motor_id_t motor, int16_t forward_permille, int16_t reverse_permille);
void ChassisControl_ResolveSideTargets(float linear_x, float angular_z, float *left_mps, float *right_mps);
void ChassisControl_GetState(chassis_control_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
