#include "chassis_tasks.h"

#include "iwdg.h"
#include "adc_monitor.h"
#include "chassis_config.h"
#include "chassis_control.h"
#include "chassis_layout.h"
#include "chassis_task_timing.h"
#include "cmsis_os2.h"
#include "encoder_driver.h"
#include "esp12f_comm.h"
#include "esp12f_flash_bridge.h"
#include "imu_bmi270.h"
#include "imu_calibration_gate.h"
#include "led_status.h"
#include "line_control.h"
#include "line_uart.h"
#include "ps2_control.h"
#include "reset_trace.h"
#include "system_monitor.h"
#include "upper_uart.h"
#include "usart1_debug_console.h"
#include "ssd1306.h"
#include "oled_ui.h"
#include "flash_param.h"
#include "param_store.h"
#include "power_on_self_test.h"
#include "control_manager.h"
#include "main.h"

extern osThreadId_t imuTaskHandle;

#define CHASSIS_CURRENT_ZERO_MAX_SPEED_MPS 0.02f
#define CHASSIS_IMU_AUTOSAVE_RETRY_MS 1000U
#define CHASSIS_IMU_AUTOSAVE_MAX_ATTEMPTS 3U

static imu_calibration_gate_t imu_calibration_gate;
static volatile uint8_t imu_first_calibration_save_needed;
static volatile uint8_t imu_calibration_save_pending;
static uint8_t imu_calibration_save_attempts;
static uint32_t imu_calibration_save_next_ms;

static uint8_t ChassisTasks_CurrentZeroStationary(void)
{
  encoder_state_t encoder_state;
  motor_driver_state_t motor_state;

  EncoderDriver_GetState(&encoder_state);
  MotorDriver_GetState(&motor_state);
  for (uint8_t motor = 0U; motor < MOTOR_ID_COUNT; ++motor)
  {
    if (motor_state.effective_pwm[motor] != 0)
    {
      return 0U;
    }
    if (ChassisLayout_MotorEnabled((motor_id_t)motor) != 0U &&
        (encoder_state.speed_valid[motor] == 0U ||
         encoder_state.speed_mps[motor] < -CHASSIS_CURRENT_ZERO_MAX_SPEED_MPS ||
         encoder_state.speed_mps[motor] > CHASSIS_CURRENT_ZERO_MAX_SPEED_MPS))
    {
      return 0U;
    }
  }
  return 1U;
}

static void ChassisTasks_ServiceFirstImuCalibrationSave(uint32_t now_ms)
{
  adc_monitor_state_t adc_state;
  flash_param_bundle_t bundle;
  imu_bmi270_state_t imu_state;

  if (imu_calibration_save_pending == 0U ||
      ((int32_t)(now_ms - imu_calibration_save_next_ms)) < 0 ||
      ChassisTasks_CurrentZeroStationary() == 0U)
  {
    return;
  }
  ControlManager_ClearCommand();
  ChassisControl_EmergencyStop();
  MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE);
  if (ChassisTasks_CurrentZeroStationary() == 0U)
  {
    return;
  }

  ParamStore_Get(&bundle.params);
  AdcMonitor_GetState(&adc_state);
  ImuBmi270_GetState(&imu_state);
  if (adc_state.current_zero_valid != 0U)
  {
    for (uint8_t motor = 0U; motor < MOTOR_ID_COUNT; ++motor)
    {
      bundle.params.current_zero_raw[motor] = adc_state.current_zero_raw[motor];
    }
    bundle.params.current_zero_valid = 1U;
  }
  if (imu_state.gyro_calibrated != 0U)
  {
    for (uint8_t axis = 0U; axis < 3U; ++axis)
    {
      bundle.params.imu_gyro_bias_dps[axis] = imu_state.gyro_bias_dps[axis];
    }
    bundle.params.imu_gyro_bias_valid = 1U;
  }
  ImuBmi270_GetCalibration(&bundle.imu_calibration);
  imu_calibration_save_attempts++;
  if (FlashParam_SaveBundle(&bundle) == FLASH_PARAM_STATUS_OK)
  {
    (void)ParamStore_Set(&bundle.params);
    imu_calibration_save_pending = 0U;
    imu_calibration_save_attempts = 0U;
  }
  else if (imu_calibration_save_attempts >= CHASSIS_IMU_AUTOSAVE_MAX_ATTEMPTS)
  {
    imu_calibration_save_pending = 0U;
  }
  else
  {
    imu_calibration_save_next_ms = now_ms + CHASSIS_IMU_AUTOSAVE_RETRY_MS;
  }
}

void ChassisTasks_InitHardware(void)
{
  flash_param_bundle_t bundle;
  param_store_t params;
  uint8_t params_loaded;

  ChassisTaskTiming_Reset();
  ParamStore_SetDefaults();
  params_loaded = (FlashParam_LoadBundle(&bundle) == FLASH_PARAM_STATUS_OK) ? 1U : 0U;
  if (params_loaded != 0U)
  {
    params = bundle.params;
    (void)ParamStore_Set(&params);
  }
  EncoderDriver_Init();
  AdcMonitor_Init();
  ImuBmi270_Init();
  ImuCalibrationGate_Init(&imu_calibration_gate);
  imu_first_calibration_save_needed = 1U;
  imu_calibration_save_pending = 0U;
  imu_calibration_save_attempts = 0U;
  imu_calibration_save_next_ms = 0UL;
  if (params_loaded != 0U)
  {
    (void)ImuBmi270_ApplyCalibration(&bundle.imu_calibration);
    if (params.current_zero_valid != 0U)
    {
      AdcMonitor_ApplyCurrentZeroCalibration(params.current_zero_raw);
    }
    if (params.imu_gyro_bias_valid != 0U)
    {
      ImuBmi270_ApplyGyroBias(params.imu_gyro_bias_dps);
      imu_first_calibration_save_needed = 0U;
    }
  }
  LedStatus_Init();
  SystemMonitor_Init();
  ChassisControl_Init();
  UpperUart_Init();
  LineUart_Init();
  LineUart_InitSensor();
  LineControl_Init();
  Ps2Control_Init();
  Esp12fComm_Init();
  Esp12fFlashBridge_Init();
  Usart1DebugConsole_Init();
  POST_Run();
  SSD1306_Init();
  OLED_UI_Init();
}

uint32_t ChassisTasks_GetMissedPeriodCount(uint32_t task)
{
  return ChassisTaskTiming_GetMissedCount((chassis_task_timing_id_t)task);
}

void Task_Safety(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    uint32_t now_ms = osKernelGetTickCount();

    SystemMonitor_Update();
    POST_UpdateRuntime(now_ms);
    ResetTrace_UpdateControl(ControlManager_GetActiveSource(),
                             ControlManager_IsEmergencyStop(),
                             ControlManager_IsFaultStop());
    ResetTrace_TaskHeartbeat(RESET_TRACE_TASK_SAFETY, now_ms);

    /* 看门狗守卫：仅当电机任务心跳在 200ms 内更新时才喂狗。
       电机任务挂死 → 心跳过期 → IWDG 超时复位。 */
    {
      uint32_t motor_hb = ResetTrace_GetTaskHeartbeat(RESET_TRACE_TASK_MOTOR);
      if (motor_hb != 0U && (now_ms - motor_hb) <= 200U)
      {
        HAL_IWDG_Refresh(&hiwdg);
      }
    }
    ChassisTasks_ServiceFirstImuCalibrationSave(now_ms);
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_SAFETY, &next_wake, CHASSIS_ADC_PERIOD_MS);
  }
}

void Task_MotorControl(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    uint32_t now_ms = osKernelGetTickCount();

    ResetTrace_TaskHeartbeat(RESET_TRACE_TASK_MOTOR, now_ms);
    EncoderDriver_Update(now_ms);
    AdcMonitor_SetCurrentZeroStationary(ChassisTasks_CurrentZeroStationary());
    ChassisControl_Step(now_ms);
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_MOTOR, &next_wake, CHASSIS_CONTROL_PERIOD_MS);
  }
}

void Task_RpiComm(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    UpperUart_Update();
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_RPI, &next_wake, UPPER_UART_TASK_PERIOD_MS);
  }
}

void Task_Imu(void *argument)
{
  (void)argument;
  for (;;)
  {
    encoder_state_t encoder_state;
    motor_driver_state_t motor_state;
    imu_bmi270_state_t imu_state;
    uint8_t motor_enabled_mask = 0U;
    uint8_t stationary;
    uint8_t was_calibrated;
    uint32_t now_ms;

    (void)osThreadFlagsWait(CHASSIS_IMU_TASK_FLAG_DRDY,
                            osFlagsWaitAny,
                            CHASSIS_IMU_PERIOD_MS);
    (void)ImuBmi270_Update();
    now_ms = osKernelGetTickCount();
    EncoderDriver_GetState(&encoder_state);
    MotorDriver_GetState(&motor_state);
    ImuBmi270_GetState(&imu_state);
    was_calibrated = imu_state.gyro_calibrated;
    for (uint8_t motor = 0U; motor < MOTOR_ID_COUNT; ++motor)
    {
      if (ChassisLayout_MotorEnabled((motor_id_t)motor) != 0U)
      {
        motor_enabled_mask |= (uint8_t)(1U << motor);
      }
    }
    stationary = ImuCalibrationGate_Update(&imu_calibration_gate,
                                           motor_state.effective_pwm,
                                           encoder_state.speed_mps,
                                           encoder_state.speed_valid,
                                           motor_enabled_mask,
                                           imu_state.body_accel_g,
                                           imu_state.gyro_corrected_dps,
                                           imu_state.sample_count);
    ImuBmi270_ServiceCalibration(now_ms, stationary);
    ImuBmi270_GetState(&imu_state);
    if (imu_first_calibration_save_needed != 0U &&
        was_calibrated == 0U && imu_state.gyro_calibrated != 0U)
    {
      imu_first_calibration_save_needed = 0U;
      imu_calibration_save_pending = 1U;
      imu_calibration_save_attempts = 0U;
      imu_calibration_save_next_ms = now_ms;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == IMU_INT1_Pin)
  {
    ImuBmi270_OnDataReadyFromIsr();
    if (imuTaskHandle != NULL)
    {
      (void)osThreadFlagsSet(imuTaskHandle, CHASSIS_IMU_TASK_FLAG_DRDY);
    }
  }
}

void Task_Line(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    LineUart_Update();
    LineUart_RequestAnalog();
    LineControl_Update();
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_LINE, &next_wake, CHASSIS_LINE_PERIOD_MS);
  }
}

void Task_Esp12f(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    uint32_t now_ms = osKernelGetTickCount();

    ResetTrace_TaskHeartbeat(RESET_TRACE_TASK_ESP, now_ms);
    Esp12fFlashBridge_Update(now_ms);
    Esp12fComm_Update();
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_ESP, &next_wake, CHASSIS_ESP12F_PERIOD_MS);
  }
}

void Task_Ps2(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    ResetTrace_TaskHeartbeat(RESET_TRACE_TASK_PS2, osKernelGetTickCount());
    Ps2Control_Update();
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_PS2, &next_wake, CHASSIS_PS2_PERIOD_MS);
  }
}

void Task_Led(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    LedStatus_TaskStep(CHASSIS_LED_PERIOD_MS);
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_LED, &next_wake, CHASSIS_LED_PERIOD_MS);
  }
}

void Task_Oled(void *argument)
{
  uint32_t next_wake = osKernelGetTickCount();
  (void)argument;
  for (;;)
  {
    OLED_UI_Update();
    ChassisTaskTiming_DelayUntil(CHASSIS_TASK_TIMING_OLED, &next_wake, OLED_TASK_PERIOD_MS);
  }
}
