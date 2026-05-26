#include "encoder_driver.h"

#include "chassis_config.h"
#include "encoder_math.h"
#include "tim.h"

#define TWO_PI_F 6.28318530718f

typedef struct
{
  TIM_HandleTypeDef *htim;
  int8_t direction;
} encoder_hw_t;

static const encoder_hw_t encoder_hw[MOTOR_ID_COUNT] = {
  { &htim2, CHASSIS_M1_ENCODER_DIR },
  { &htim3, CHASSIS_M2_ENCODER_DIR },
  { &htim4, CHASSIS_M3_ENCODER_DIR },
  { &htim5, CHASSIS_M4_ENCODER_DIR },
};

static encoder_state_t encoder_state;
static uint32_t last_count[MOTOR_ID_COUNT];
static uint32_t last_update_ms;
static uint8_t has_last_update;

int32_t EncoderDriver_DiffCount(uint32_t now, uint32_t last, uint32_t period)
{
  return EncoderMath_DiffCount(now, last, period);
}

float EncoderDriver_GetCountsPerRev(void)
{
  return CHASSIS_ENCODER_BASE_PPR * CHASSIS_ENCODER_QUADRATURE_MULT * CHASSIS_MOTOR_GEAR_RATIO;
}

void EncoderDriver_Init(void)
{
  for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    (void)HAL_TIM_Encoder_Start(encoder_hw[i].htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(encoder_hw[i].htim, 0U);
    last_count[i] = 0U;
  }
  encoder_state = (encoder_state_t){0};
  last_update_ms = 0U;
  has_last_update = 0U;
}

void EncoderDriver_Update(uint32_t now_ms)
{
  uint32_t now_count[MOTOR_ID_COUNT];
  int32_t delta[MOTOR_ID_COUNT];
  uint32_t dt_ms = now_ms - last_update_ms;
  float dt_s = (float)dt_ms / 1000.0f;
  float counts_per_rev = EncoderDriver_GetCountsPerRev();
  float meters_per_rev = TWO_PI_F * CHASSIS_WHEEL_RADIUS_M;
  uint32_t primask;
  uint8_t valid_all = 1U;

  for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(encoder_hw[i].htim);
    now_count[i] = __HAL_TIM_GET_COUNTER(encoder_hw[i].htim);
    delta[i] = EncoderDriver_DiffCount(now_count[i], last_count[i], period) * encoder_hw[i].direction;
    last_count[i] = now_count[i];
  }

  primask = __get_PRIMASK();
  __disable_irq();

  encoder_state.last_update_ms = now_ms;
  for (uint32_t i = 0U; i < MOTOR_ID_COUNT; ++i)
  {
    encoder_state.delta[i] = delta[i];
    encoder_state.count[i] += delta[i];

    if (has_last_update == 0U ||
        dt_ms <= CHASSIS_MIN_ENCODER_DT_MS ||
        dt_ms > CHASSIS_MAX_ENCODER_DT_MS ||
        counts_per_rev <= 0.0f ||
        meters_per_rev <= 0.0f)
    {
      encoder_state.speed_mps[i] = 0.0f;
      encoder_state.speed_valid[i] = 0U;
      valid_all = 0U;
    }
    else
    {
      encoder_state.speed_mps[i] = ((float)delta[i] / counts_per_rev) * meters_per_rev / dt_s;
      encoder_state.speed_valid[i] = 1U;
    }
  }

  encoder_state.left_count = (encoder_state.count[MOTOR_ID_M1] + encoder_state.count[MOTOR_ID_M2]) / 2;
  encoder_state.right_count = (encoder_state.count[MOTOR_ID_M3] + encoder_state.count[MOTOR_ID_M4]) / 2;
  encoder_state.left_delta = (encoder_state.delta[MOTOR_ID_M1] + encoder_state.delta[MOTOR_ID_M2]) / 2;
  encoder_state.right_delta = (encoder_state.delta[MOTOR_ID_M3] + encoder_state.delta[MOTOR_ID_M4]) / 2;
  encoder_state.left_speed_mps = (encoder_state.speed_mps[MOTOR_ID_M1] + encoder_state.speed_mps[MOTOR_ID_M2]) * 0.5f;
  encoder_state.right_speed_mps = (encoder_state.speed_mps[MOTOR_ID_M3] + encoder_state.speed_mps[MOTOR_ID_M4]) * 0.5f;
  encoder_state.left_speed_valid = (encoder_state.speed_valid[MOTOR_ID_M1] != 0U &&
                                    encoder_state.speed_valid[MOTOR_ID_M2] != 0U) ? 1U : 0U;
  encoder_state.right_speed_valid = (encoder_state.speed_valid[MOTOR_ID_M3] != 0U &&
                                     encoder_state.speed_valid[MOTOR_ID_M4] != 0U) ? 1U : 0U;
  if (encoder_state.left_speed_valid == 0U || encoder_state.right_speed_valid == 0U)
  {
    valid_all = 0U;
  }
  encoder_state.speed_valid_all = valid_all;

  __set_PRIMASK(primask);

  last_update_ms = now_ms;
  has_last_update = 1U;
}

void EncoderDriver_GetState(encoder_state_t *state)
{
  uint32_t primask;

  if (state == 0)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  *state = encoder_state;
  __set_PRIMASK(primask);
}
