#include "usart1_debug_console.h"

#include "adc_monitor.h"
#include "chassis_config.h"
#include "chassis_control.h"
#include "chassis_task_timing.h"
#include "cmsis_os2.h"
#include "control_manager.h"
#include "encoder_driver.h"
#include "esp12f_comm.h"
#include "esp12f_flash_bridge.h"
#include "imu_bmi270.h"
#include "line_uart.h"
#include "motor_driver.h"
#include "ps2_control.h"
#include "system_monitor.h"
#include "upper_uart.h"
#include "usart.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEBUG_CONSOLE_RX_LINE_SIZE   96U
#define DEBUG_CONSOLE_RX_RING_SIZE   160U
#define DEBUG_CONSOLE_TX_LINE_SIZE   768U
#define DEBUG_CONSOLE_TASK_PERIOD_MS 10U
#define DEBUG_CONSOLE_LOG_PERIOD_MS  500U

static char rx_line[DEBUG_CONSOLE_RX_LINE_SIZE];
static uint8_t rx_len;
static uint8_t stream_enabled;
static uint8_t debug_velocity_enabled;
static chassis_cmd_t debug_velocity_cmd;
static uint8_t rx_byte;
static volatile uint8_t rx_ring[DEBUG_CONSOLE_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

extern osThreadId_t defaultTaskHandle;
extern osThreadId_t safetyTaskHandle;
extern osThreadId_t motorTaskHandle;
extern osThreadId_t rpiCommTaskHandle;
extern osThreadId_t imuTaskHandle;
extern osThreadId_t lineTaskHandle;
extern osThreadId_t espTaskHandle;
extern osThreadId_t usart1DebugTaskHandle;
extern osThreadId_t ps2TaskHandle;
extern osThreadId_t ledTaskHandle;

static void DebugConsole_Write(const char *text)
{
  if (text != 0)
  {
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 50U);
  }
}

static int32_t DebugConsole_Milli(float value)
{
  return (int32_t)(value * 1000.0f);
}

static int16_t DebugConsole_ClampPermille(int32_t value)
{
  if (value > CHASSIS_PWM_MAX_PERMILLE)
  {
    return CHASSIS_PWM_MAX_PERMILLE;
  }
  if (value < -CHASSIS_PWM_MAX_PERMILLE)
  {
    return -CHASSIS_PWM_MAX_PERMILLE;
  }
  return (int16_t)value;
}

static uint8_t DebugConsole_MotorTestAllowed(void)
{
  return (ControlManager_IsEmergencyStop() == 0U &&
          ControlManager_IsFaultStop() == 0U) ? 1U : 0U;
}

static void DebugConsole_PrintHelp(void)
{
  DebugConsole_Write(
    "\r\nF407 V2 debug console\r\n"
    "help/status/header/log 0|1\r\n"
    "rtos             heap and task stack status\r\n"
    "motor L R        side open-loop permille\r\n"
    "left P/right P   side open-loop shortcut\r\n"
    "m1 F R ... m4 F R raw IN1/IN2 permille\r\n"
    "raw LF LR RF RR  left/right pair raw inputs\r\n"
    "vel V [W]        closed-loop mm/s, optional mrad/s\r\n"
    "stop             clear tests and commands\r\n"
    "estop 0|1        clear/set emergency stop\r\n"
    "clearfault       clear latched overcurrent/driver faults\r\n"
    "imutest/imuinit/imu 0|1\r\n"
    "espreset/espboot 0|1\r\n"
    "espflash on|off|status bridge USART1 to ESP12F\r\n"
    "\r\n");
}

static void DebugConsole_PrintTaskStatus(const char *name, osThreadId_t handle, uint32_t missed)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];

  if (handle == NULL)
  {
    (void)snprintf(tx, sizeof(tx), "RTOS %-10s missing\r\n", name);
  }
  else
  {
    (void)snprintf(tx, sizeof(tx),
                   "RTOS %-10s state=%ld stack_free=%luB missed=%lu\r\n",
                   name,
                   (long)osThreadGetState(handle),
                   (unsigned long)osThreadGetStackSpace(handle),
                   (unsigned long)missed);
  }
  DebugConsole_Write(tx);
}

static void DebugConsole_PrintRtosStatus(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  upper_uart_state_t upper_state;
  esp12f_comm_state_t esp_state;

  UpperUart_GetState(&upper_state);
  Esp12fComm_GetState(&esp_state);

  (void)snprintf(tx, sizeof(tx),
                 "RTOS heap_free=%luB heap_min=%luB tick=%lu\r\n",
                 (unsigned long)xPortGetFreeHeapSize(),
                 (unsigned long)xPortGetMinimumEverFreeHeapSize(),
                 (unsigned long)osKernelGetTickCount());
  DebugConsole_Write(tx);

  DebugConsole_PrintTaskStatus("default", defaultTaskHandle, 0U);
  DebugConsole_PrintTaskStatus("safety", safetyTaskHandle, ChassisTaskTiming_GetMissedCount(CHASSIS_TASK_TIMING_SAFETY));
  DebugConsole_PrintTaskStatus("motor", motorTaskHandle, ChassisTaskTiming_GetMissedCount(CHASSIS_TASK_TIMING_MOTOR));
  DebugConsole_PrintTaskStatus("rpi", rpiCommTaskHandle, ChassisTaskTiming_GetMissedCount(CHASSIS_TASK_TIMING_RPI));
  DebugConsole_PrintTaskStatus("imu", imuTaskHandle, ChassisTaskTiming_GetMissedCount(CHASSIS_TASK_TIMING_IMU));
  DebugConsole_PrintTaskStatus("line", lineTaskHandle, ChassisTaskTiming_GetMissedCount(CHASSIS_TASK_TIMING_LINE));
  DebugConsole_PrintTaskStatus("esp", espTaskHandle, ChassisTaskTiming_GetMissedCount(CHASSIS_TASK_TIMING_ESP));
  DebugConsole_PrintTaskStatus("debug", usart1DebugTaskHandle, 0U);
  DebugConsole_PrintTaskStatus("ps2", ps2TaskHandle, ChassisTaskTiming_GetMissedCount(CHASSIS_TASK_TIMING_PS2));
  DebugConsole_PrintTaskStatus("led", ledTaskHandle, ChassisTaskTiming_GetMissedCount(CHASSIS_TASK_TIMING_LED));

  (void)snprintf(tx, sizeof(tx),
                 "RTOS comm upper_tx=%lu upper_drop=%lu esp_tx=%lu esp_drop=%lu\r\n",
                 (unsigned long)upper_state.tx_frames,
                 (unsigned long)upper_state.tx_busy_drops,
                 (unsigned long)esp_state.tx_frames,
                 (unsigned long)esp_state.tx_busy_drops);
  DebugConsole_Write(tx);
}

static void DebugConsole_PrintEspFlashStatus(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  esp12f_flash_bridge_state_t bridge_state;
  uint32_t now_ms = osKernelGetTickCount();
  uint32_t idle_ms;

  Esp12fFlashBridge_GetState(&bridge_state);
  idle_ms = now_ms - bridge_state.last_activity_ms;

  (void)snprintf(tx, sizeof(tx),
                 "ESPFLASH active=%u download=%u idle=%lums pc_rx=%lu pc_tx=%lu esp_rx=%lu esp_tx=%lu ovf=%lu/%lu uart_err=%lu auto_exit=%lu\r\n",
                 bridge_state.active,
                 bridge_state.download_mode,
                 (unsigned long)idle_ms,
                 (unsigned long)bridge_state.pc_to_esp_rx_bytes,
                 (unsigned long)bridge_state.pc_to_esp_tx_bytes,
                 (unsigned long)bridge_state.esp_to_pc_rx_bytes,
                 (unsigned long)bridge_state.esp_to_pc_tx_bytes,
                 (unsigned long)bridge_state.pc_to_esp_overflow,
                 (unsigned long)bridge_state.esp_to_pc_overflow,
                 (unsigned long)bridge_state.uart_error_count,
                 (unsigned long)bridge_state.auto_exit_count);
  DebugConsole_Write(tx);
}

static void DebugConsole_PrintHeader(void)
{
  DebugConsole_Write("t_ms,m1_mms,m2_mms,m3_mms,m4_mms,m1_pwm,m2_pwm,m3_pwm,m4_pwm,vbat_mv,m1_ma,m2_ma,m3_ma,m4_ma,imu_online,imu_chip,errors,source,ps2_ok,ps2_fail,line_bytes,line_frames,esp_rx,esp_tx\r\n");
}

static void DebugConsole_PrintStatus(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  adc_monitor_state_t adc_state;
  encoder_state_t encoder_state;
  chassis_control_state_t chassis_state;
  system_monitor_state_t monitor_state;
  imu_bmi270_state_t imu_state;
  ps2_control_state_t ps2_state;
  line_uart_state_t line_state;
  esp12f_comm_state_t esp_state;
  motor_driver_state_t motor_state;

  AdcMonitor_GetState(&adc_state);
  EncoderDriver_GetState(&encoder_state);
  ChassisControl_GetState(&chassis_state);
  SystemMonitor_GetState(&monitor_state);
  ImuBmi270_GetState(&imu_state);
  Ps2Control_GetState(&ps2_state);
  LineUart_GetState(&line_state);
  Esp12fComm_GetState(&esp_state);
  MotorDriver_GetState(&motor_state);

  (void)snprintf(tx, sizeof(tx),
                 "ENC m1=%ld d=%ld %ldmm/s v=%u m2=%ld d=%ld %ldmm/s v=%u m3=%ld d=%ld %ldmm/s v=%u m4=%ld d=%ld %ldmm/s v=%u\r\n",
                 (long)encoder_state.count[MOTOR_ID_M1], (long)encoder_state.delta[MOTOR_ID_M1], (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M1]), encoder_state.speed_valid[MOTOR_ID_M1],
                 (long)encoder_state.count[MOTOR_ID_M2], (long)encoder_state.delta[MOTOR_ID_M2], (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M2]), encoder_state.speed_valid[MOTOR_ID_M2],
                 (long)encoder_state.count[MOTOR_ID_M3], (long)encoder_state.delta[MOTOR_ID_M3], (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M3]), encoder_state.speed_valid[MOTOR_ID_M3],
                 (long)encoder_state.count[MOTOR_ID_M4], (long)encoder_state.delta[MOTOR_ID_M4], (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M4]), encoder_state.speed_valid[MOTOR_ID_M4]);
  DebugConsole_Write(tx);

  (void)snprintf(tx, sizeof(tx),
                 "CHASSIS req=%ld,%ldmm/s target=%ld,%ldmm/s actual=%ld,%ldmm/s pwm=%d,%d,%d,%d out=%u estop=%u fault=%u\r\n",
                 (long)DebugConsole_Milli(chassis_state.left_requested_mps),
                 (long)DebugConsole_Milli(chassis_state.right_requested_mps),
                 (long)DebugConsole_Milli(chassis_state.left_target_mps),
                 (long)DebugConsole_Milli(chassis_state.right_target_mps),
                 (long)DebugConsole_Milli(chassis_state.left_actual_mps),
                 (long)DebugConsole_Milli(chassis_state.right_actual_mps),
                 chassis_state.motor_output_permille[MOTOR_ID_M1],
                 chassis_state.motor_output_permille[MOTOR_ID_M2],
                 chassis_state.motor_output_permille[MOTOR_ID_M3],
                 chassis_state.motor_output_permille[MOTOR_ID_M4],
                 chassis_state.output_enabled,
                 ControlManager_IsEmergencyStop(),
                 ControlManager_IsFaultStop());
  DebugConsole_Write(tx);

  (void)snprintf(tx, sizeof(tx),
                 "ADC vbat=%ldmV m1=%ldmA raw=%u m2=%ldmA raw=%u m3=%ldmA raw=%u m4=%ldmA raw=%u valid=%u\r\n",
                 (long)DebugConsole_Milli(adc_state.battery_voltage),
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M1]), adc_state.raw_current[MOTOR_ID_M1],
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M2]), adc_state.raw_current[MOTOR_ID_M2],
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M3]), adc_state.raw_current[MOTOR_ID_M3],
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M4]), adc_state.raw_current[MOTOR_ID_M4],
                 adc_state.current_valid);
  DebugConsole_Write(tx);

  (void)snprintf(tx, sizeof(tx),
                 "BMI270 enabled=%u online=%u chip=0x%02X err=%u errcnt=%lu acc_mg=%ld,%ld,%ld gyro_mdps=%ld,%ld,%ld\r\n",
                 imu_state.enabled, imu_state.online, imu_state.chip_id, imu_state.last_error,
                 (unsigned long)imu_state.error_count,
                 (long)DebugConsole_Milli(imu_state.accel_g[0]),
                 (long)DebugConsole_Milli(imu_state.accel_g[1]),
                 (long)DebugConsole_Milli(imu_state.accel_g[2]),
                 (long)DebugConsole_Milli(imu_state.gyro_dps[0]),
                 (long)DebugConsole_Milli(imu_state.gyro_dps[1]),
                 (long)DebugConsole_Milli(imu_state.gyro_dps[2]));
  DebugConsole_Write(tx);

  (void)snprintf(tx, sizeof(tx),
                 "SYS source=%u errors=0x%08lX latched=0x%08lX drv_fault=%u,%u,%u,%u line=%lu/%lu esp=%lu/%lu ps2=%u ok=%lu fail=%lu\r\n",
                 monitor_state.control_mode,
                 (unsigned long)monitor_state.error_flags,
                 (unsigned long)monitor_state.latched_error_flags,
                 motor_state.fault_active[MOTOR_ID_M1],
                 motor_state.fault_active[MOTOR_ID_M2],
                 motor_state.fault_active[MOTOR_ID_M3],
                 motor_state.fault_active[MOTOR_ID_M4],
                 (unsigned long)line_state.rx_bytes,
                 (unsigned long)line_state.rx_frames,
                 (unsigned long)esp_state.rx_frames,
                 (unsigned long)esp_state.tx_frames,
                 ps2_state.online,
                 (unsigned long)ps2_state.rx_ok_count,
                 (unsigned long)ps2_state.rx_fail_count);
  DebugConsole_Write(tx);
}

static void DebugConsole_PrintLogFrame(uint32_t now_ms)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  adc_monitor_state_t adc_state;
  encoder_state_t encoder_state;
  chassis_control_state_t chassis_state;
  system_monitor_state_t monitor_state;
  imu_bmi270_state_t imu_state;
  ps2_control_state_t ps2_state;
  line_uart_state_t line_state;
  esp12f_comm_state_t esp_state;

  AdcMonitor_GetState(&adc_state);
  EncoderDriver_GetState(&encoder_state);
  ChassisControl_GetState(&chassis_state);
  SystemMonitor_GetState(&monitor_state);
  ImuBmi270_GetState(&imu_state);
  Ps2Control_GetState(&ps2_state);
  LineUart_GetState(&line_state);
  Esp12fComm_GetState(&esp_state);

  (void)snprintf(tx, sizeof(tx),
                 "%lu,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%ld,%u,%u,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
                 (unsigned long)now_ms,
                 (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M1]),
                 (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M2]),
                 (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M3]),
                 (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M4]),
                 chassis_state.motor_output_permille[MOTOR_ID_M1],
                 chassis_state.motor_output_permille[MOTOR_ID_M2],
                 chassis_state.motor_output_permille[MOTOR_ID_M3],
                 chassis_state.motor_output_permille[MOTOR_ID_M4],
                 (long)DebugConsole_Milli(adc_state.battery_voltage),
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M1]),
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M2]),
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M3]),
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M4]),
                 imu_state.online,
                 imu_state.chip_id,
                 (unsigned long)monitor_state.error_flags,
                 monitor_state.control_mode,
                 (unsigned long)ps2_state.rx_ok_count,
                 (unsigned long)ps2_state.rx_fail_count,
                 (unsigned long)line_state.rx_bytes,
                 (unsigned long)line_state.rx_frames,
                 (unsigned long)esp_state.rx_frames,
                 (unsigned long)esp_state.tx_frames);
  DebugConsole_Write(tx);
}

static uint8_t DebugConsole_ParseMotorId(const char *line, motor_id_t *motor)
{
  if (strncmp(line, "m1 ", 3) == 0)
  {
    *motor = MOTOR_ID_M1;
    return 1U;
  }
  if (strncmp(line, "m2 ", 3) == 0)
  {
    *motor = MOTOR_ID_M2;
    return 1U;
  }
  if (strncmp(line, "m3 ", 3) == 0)
  {
    *motor = MOTOR_ID_M3;
    return 1U;
  }
  if (strncmp(line, "m4 ", 3) == 0)
  {
    *motor = MOTOR_ID_M4;
    return 1U;
  }
  return 0U;
}

static void DebugConsole_HandleLine(char *line)
{
  int left;
  int right;
  int value;
  int lf;
  int lr;
  int rf;
  int rr;
  int linear_mm_s;
  int angular_mrad_s = 0;
  motor_id_t motor;

  if ((strcmp(line, "help") == 0) || (strcmp(line, "h") == 0))
  {
    DebugConsole_PrintHelp();
  }
  else if ((strcmp(line, "status") == 0) || (strcmp(line, "s") == 0))
  {
    DebugConsole_PrintStatus();
  }
  else if (strcmp(line, "rtos") == 0)
  {
    DebugConsole_PrintRtosStatus();
  }
  else if (strcmp(line, "header") == 0)
  {
    DebugConsole_PrintHeader();
  }
  else if (sscanf(line, "log %d", &value) == 1)
  {
    stream_enabled = (value != 0) ? 1U : 0U;
    if (stream_enabled != 0U)
    {
      DebugConsole_PrintHeader();
    }
    else
    {
      DebugConsole_Write("log off\r\n");
    }
  }
  else if (sscanf(line, "motor %d %d", &left, &right) == 2)
  {
    if (DebugConsole_MotorTestAllowed() == 0U)
    {
      DebugConsole_Write("motor test rejected: estop/fault active\r\n");
      return;
    }
    debug_velocity_enabled = 0U;
    ControlManager_ClearCommand();
    ChassisControl_OpenLoopTest(DebugConsole_ClampPermille(left), DebugConsole_ClampPermille(right));
    DebugConsole_Write("side motor test updated\r\n");
  }
  else if (sscanf(line, "left %d", &value) == 1)
  {
    if (DebugConsole_MotorTestAllowed() == 0U)
    {
      DebugConsole_Write("left test rejected: estop/fault active\r\n");
      return;
    }
    debug_velocity_enabled = 0U;
    ControlManager_ClearCommand();
    ChassisControl_OpenLoopTest(DebugConsole_ClampPermille(value), 0);
    DebugConsole_Write("left side test updated\r\n");
  }
  else if (sscanf(line, "right %d", &value) == 1)
  {
    if (DebugConsole_MotorTestAllowed() == 0U)
    {
      DebugConsole_Write("right test rejected: estop/fault active\r\n");
      return;
    }
    debug_velocity_enabled = 0U;
    ControlManager_ClearCommand();
    ChassisControl_OpenLoopTest(0, DebugConsole_ClampPermille(value));
    DebugConsole_Write("right side test updated\r\n");
  }
  else if (DebugConsole_ParseMotorId(line, &motor) != 0U && sscanf(&line[3], "%d %d", &lf, &lr) == 2)
  {
    if (DebugConsole_MotorTestAllowed() == 0U)
    {
      DebugConsole_Write("raw motor test rejected: estop/fault active\r\n");
      return;
    }
    debug_velocity_enabled = 0U;
    ControlManager_ClearCommand();
    ChassisControl_RawMotorInputTest(motor, DebugConsole_ClampPermille(lf), DebugConsole_ClampPermille(lr));
    DebugConsole_Write("single motor raw test updated\r\n");
  }
  else if (sscanf(line, "raw %d %d %d %d", &lf, &lr, &rf, &rr) == 4)
  {
    if (DebugConsole_MotorTestAllowed() == 0U)
    {
      DebugConsole_Write("raw test rejected: estop/fault active\r\n");
      return;
    }
    debug_velocity_enabled = 0U;
    ControlManager_ClearCommand();
    ChassisControl_RawInputTest(DebugConsole_ClampPermille(lf),
                                DebugConsole_ClampPermille(lr),
                                DebugConsole_ClampPermille(rf),
                                DebugConsole_ClampPermille(rr));
    DebugConsole_Write("side raw test updated\r\n");
  }
  else if (sscanf(line, "vel %d %d", &linear_mm_s, &angular_mrad_s) == 2 ||
           sscanf(line, "vel %d", &linear_mm_s) == 1)
  {
    debug_velocity_cmd = (chassis_cmd_t){
      .linear_x = (float)linear_mm_s / 1000.0f,
      .angular_z = (float)angular_mrad_s / 1000.0f,
      .enable = 1U,
      .source = CONTROL_SOURCE_DEBUG,
      .timestamp_ms = osKernelGetTickCount(),
    };
    ChassisControl_OpenLoopTest(0, 0);
    if (ControlManager_SetCommand(&debug_velocity_cmd) == CONTROL_COMMAND_ACCEPTED)
    {
      debug_velocity_enabled = 1U;
      DebugConsole_Write("velocity command accepted\r\n");
    }
    else
    {
      DebugConsole_Write("velocity command rejected\r\n");
    }
  }
  else if (strcmp(line, "stop") == 0)
  {
    debug_velocity_enabled = 0U;
    ChassisControl_OpenLoopTest(0, 0);
    ChassisControl_RawInputTest(0, 0, 0, 0);
    ControlManager_ClearCommand();
    DebugConsole_Write("chassis stopped\r\n");
  }
  else if (sscanf(line, "estop %d", &value) == 1)
  {
    ControlManager_SetEmergencyStop((value != 0) ? 1U : 0U);
    DebugConsole_Write((value != 0) ? "estop set\r\n" : "estop cleared\r\n");
  }
  else if (strcmp(line, "clearfault") == 0)
  {
    SystemMonitor_ClearLatchedFaults(0xFFFFFFFFUL);
    DebugConsole_Write("fault clear requested\r\n");
  }
  else if (strcmp(line, "imutest") == 0)
  {
    DebugConsole_Write((ImuBmi270_ProbeNow() != 0U) ? "bmi270 probe ok\r\n" : "bmi270 probe failed\r\n");
  }
  else if (strcmp(line, "imuinit") == 0)
  {
    DebugConsole_Write((ImuBmi270_ConfigNow() != 0U) ? "bmi270 init ok\r\n" : "bmi270 init failed\r\n");
  }
  else if (sscanf(line, "imu %d", &value) == 1)
  {
    (void)ImuBmi270_SetEnabled((value != 0) ? 1U : 0U);
    DebugConsole_Write((value != 0) ? "imu enabled\r\n" : "imu disabled\r\n");
  }
  else if (strcmp(line, "espreset") == 0)
  {
    Esp12fComm_ResetModule();
    DebugConsole_Write("esp12f reset\r\n");
  }
  else if (sscanf(line, "espboot %d", &value) == 1)
  {
    Esp12fComm_SetDownloadMode((value != 0) ? 1U : 0U);
    DebugConsole_Write((value != 0) ? "esp12f download mode\r\n" : "esp12f normal boot mode\r\n");
  }
  else if (strcmp(line, "espflash on") == 0)
  {
    stream_enabled = 0U;
    debug_velocity_enabled = 0U;
    DebugConsole_Write("esp12f flash bridge on: close this terminal and use esptool/Arduino at 115200\r\n");
    Esp12fFlashBridge_Enable();
  }
  else if (strcmp(line, "espflash off") == 0)
  {
    Esp12fFlashBridge_Disable();
    DebugConsole_Write("esp12f flash bridge off, normal boot requested\r\n");
  }
  else if (strcmp(line, "espflash status") == 0)
  {
    DebugConsole_PrintEspFlashStatus();
  }
  else
  {
    DebugConsole_Write("unknown command, type help\r\n");
  }
}

static void DebugConsole_PollRx(void)
{
  uint8_t ch;

  while (rx_tail != rx_head)
  {
    ch = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % DEBUG_CONSOLE_RX_RING_SIZE);

    if ((ch == '\r') || (ch == '\n'))
    {
      if (rx_len > 0U)
      {
        rx_line[rx_len] = '\0';
        DebugConsole_HandleLine(rx_line);
        rx_len = 0U;
      }
    }
    else if (rx_len < (DEBUG_CONSOLE_RX_LINE_SIZE - 1U))
    {
      rx_line[rx_len++] = (char)ch;
    }
    else
    {
      rx_len = 0U;
      DebugConsole_Write("line too long\r\n");
    }
  }
}

void Usart1DebugConsole_Init(void)
{
  rx_len = 0U;
  stream_enabled = 0U;
  debug_velocity_enabled = 0U;
  debug_velocity_cmd = (chassis_cmd_t){0};
  rx_head = 0U;
  rx_tail = 0U;
  HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  Usart1DebugConsole_RestartRx();
  DebugConsole_Write("\r\nF407 V2 chassis firmware\r\n");
  DebugConsole_Write("USART1 debug console ready, type help\r\n");
}

void Usart1DebugConsole_RestartRx(void)
{
  rx_len = 0U;
  rx_head = 0U;
  rx_tail = 0U;
  (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

void Usart1DebugConsole_OnRxCplt(void)
{
  uint16_t next_head = (uint16_t)((rx_head + 1U) % DEBUG_CONSOLE_RX_RING_SIZE);

  if (next_head != rx_tail)
  {
    rx_ring[rx_head] = rx_byte;
    rx_head = next_head;
  }
  (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

void Usart1DebugConsole_OnUartError(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
}

void Task_Usart1DebugConsole(void *argument)
{
  uint32_t last_log_ms = 0U;

  (void)argument;
  for (;;)
  {
    uint32_t now_ms = osKernelGetTickCount();

    if (Esp12fFlashBridge_IsActive() != 0U)
    {
      osDelay(DEBUG_CONSOLE_TASK_PERIOD_MS);
      continue;
    }

    DebugConsole_PollRx();

    if (debug_velocity_enabled != 0U)
    {
      debug_velocity_cmd.timestamp_ms = now_ms;
      if (ControlManager_SetCommand(&debug_velocity_cmd) != CONTROL_COMMAND_ACCEPTED)
      {
        debug_velocity_enabled = 0U;
        DebugConsole_Write("velocity command stopped\r\n");
      }
    }

    if ((stream_enabled != 0U) && ((now_ms - last_log_ms) >= DEBUG_CONSOLE_LOG_PERIOD_MS))
    {
      last_log_ms = now_ms;
      DebugConsole_PrintLogFrame(now_ms);
    }

    osDelay(DEBUG_CONSOLE_TASK_PERIOD_MS);
  }
}
