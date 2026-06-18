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
#include "i2c.h"
#include "line_control.h"
#include "line_uart.h"
#include "motor_driver.h"
#include "ps2_control.h"
#include "reset_trace.h"
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
static uint8_t stream_mode;
static uint8_t debug_velocity_enabled;
static chassis_cmd_t debug_velocity_cmd;
static uint8_t log_filter_count;
static uint8_t log_filter_order[8];
static uint8_t rx_byte;
static volatile uint8_t rx_ring[DEBUG_CONSOLE_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static uint32_t boot_reset_flags;
static uint8_t boot_reset_flags_captured;

extern osThreadId_t defaultTaskHandle;
extern osThreadId_t safetyTaskHandle;
extern osThreadId_t motorTaskHandle;
extern osThreadId_t rpiCommTaskHandle;
extern osThreadId_t imuTaskHandle;
extern osThreadId_t lineTaskHandle;
extern osThreadId_t espTaskHandle;
extern osThreadId_t usart1DebugTaskHandle;
extern osThreadId_t ps2TaskHandle;
extern osThreadId_t oledTaskHandle;
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

static void DebugConsole_CaptureResetFlags(void)
{
  if (boot_reset_flags_captured == 0U)
  {
    boot_reset_flags = RCC->CSR;
    boot_reset_flags_captured = 1U;
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }
}

static uint8_t DebugConsole_ResetFlag(uint32_t flag)
{
  return ((boot_reset_flags & flag) != 0UL) ? 1U : 0U;
}

static void DebugConsole_PrintResetFlags(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];

  DebugConsole_CaptureResetFlags();
  (void)snprintf(tx, sizeof(tx),
                 "RESET csr=0x%08lX bor=%u por=%u pin=%u sftr=%u iwdg=%u wwdg=%u lpwr=%u\r\n",
                 (unsigned long)boot_reset_flags,
                 DebugConsole_ResetFlag(RCC_CSR_BORRSTF),
                 DebugConsole_ResetFlag(RCC_CSR_PORRSTF),
                 DebugConsole_ResetFlag(RCC_CSR_PINRSTF),
                 DebugConsole_ResetFlag(RCC_CSR_SFTRSTF),
                 DebugConsole_ResetFlag(RCC_CSR_IWDGRSTF),
                 DebugConsole_ResetFlag(RCC_CSR_WWDGRSTF),
                 DebugConsole_ResetFlag(RCC_CSR_LPWRRSTF));
  DebugConsole_Write(tx);
}

static void DebugConsole_PrintResetTrace(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  reset_trace_record_t trace;
  uint8_t valid = ResetTrace_GetBootRecord(&trace);

  (void)snprintf(tx, sizeof(tx),
                 "RESETTRACE valid=%u kind=%lu reason=%lu task=%lu line=%lu cfsr=0x%08lX hfsr=0x%08lX bfar=0x%08lX mmfar=0x%08lX pc=0x%08lX lr=0x%08lX xpsr=0x%08lX exc=0x%08lX sp=0x%08lX d0=0x%08lX d1=0x%08lX d2=0x%08lX d3=0x%08lX safety=%lu motor=%lu ps2=%lu esp=%lu debug=%lu source=%lu estop=%lu fault=%lu\r\n",
                 valid,
                 (unsigned long)trace.kind,
                 (unsigned long)trace.reason,
                 (unsigned long)trace.task,
                 (unsigned long)trace.line,
                 (unsigned long)trace.cfsr,
                 (unsigned long)trace.hfsr,
                 (unsigned long)trace.bfar,
                 (unsigned long)trace.mmfar,
                 (unsigned long)trace.stacked_pc,
                 (unsigned long)trace.stacked_lr,
                 (unsigned long)trace.stacked_xpsr,
                 (unsigned long)trace.exc_return,
                 (unsigned long)trace.stack_ptr,
                 (unsigned long)trace.detail0,
                 (unsigned long)trace.detail1,
                 (unsigned long)trace.detail2,
                 (unsigned long)trace.detail3,
                 (unsigned long)trace.heartbeat_safety,
                 (unsigned long)trace.heartbeat_motor,
                 (unsigned long)trace.heartbeat_ps2,
                 (unsigned long)trace.heartbeat_esp,
                 (unsigned long)trace.heartbeat_debug,
                 (unsigned long)trace.source,
                 (unsigned long)trace.estop,
                 (unsigned long)trace.fault);
  DebugConsole_Write(tx);
}

static uint8_t DebugConsole_MotorTestAllowed(void)
{
  return (ControlManager_IsEmergencyStop() == 0U &&
          ControlManager_IsFaultStop() == 0U) ? 1U : 0U;
}

/* ────────── 日志字段分组 ────────── */

typedef enum
{
  LOG_FLD_MOTOR = 0,
  LOG_FLD_ADC,
  LOG_FLD_IMU,
  LOG_FLD_ERRORS,
  LOG_FLD_SOURCE,
  LOG_FLD_PS2,
  LOG_FLD_LINE,
  LOG_FLD_ESP,
  LOG_FLD_COUNT
} log_field_id_t;

static const char *const log_field_names[LOG_FLD_COUNT] = {
  "motor", "adc", "imu", "errors", "source", "ps2", "line", "esp"
};

static const char *const log_field_headers[LOG_FLD_COUNT] = {
  "m1_mms,m2_mms,m3_mms,m4_mms,m1_pwm,m2_pwm,m3_pwm,m4_pwm",
  "vbat_mv,m1_ma,m2_ma,m3_ma,m4_ma",
  "imu_online,imu_chip,imu_acc_x_mg,imu_acc_y_mg,imu_acc_z_mg,imu_gyro_corr_x_mdps,imu_gyro_corr_y_mdps,imu_gyro_corr_z_mdps,imu_gyro_filt_x_mdps,imu_gyro_filt_y_mdps,imu_gyro_filt_z_mdps,imu_roll_mdeg,imu_pitch_mdeg,imu_yaw_mdeg",
  "errors",
  "source",
  "ps2_ok,ps2_fail",
  "line_bytes,line_frames",
  "esp_rx,esp_tx"
};

static void DebugConsole_PrintFilteredHeader(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  size_t pos = 0U;
  uint8_t i;

  pos += (size_t)snprintf(tx + pos, sizeof(tx) - pos, "t_ms");
  for (i = 0U; i < log_filter_count; ++i)
  {
    pos += (size_t)snprintf(tx + pos, sizeof(tx) - pos, ",%s", log_field_headers[log_filter_order[i]]);
  }
  pos += (size_t)snprintf(tx + pos, sizeof(tx) - pos, "\r\n");
  DebugConsole_Write(tx);
}

static size_t DebugConsole_WriteFieldData(char *tx, size_t pos, log_field_id_t field)
{
  adc_monitor_state_t adc;
  encoder_state_t enc;
  chassis_control_state_t cs;
  system_monitor_state_t mon;
  imu_bmi270_state_t imu;
  ps2_control_state_t ps2;
  line_uart_state_t line;
  esp12f_comm_state_t esp;

  /* 惰性获取：仅需要的字段才获取状态快照 */
  switch (field)
  {
    case LOG_FLD_MOTOR:
      EncoderDriver_GetState(&enc);
      ChassisControl_GetState(&cs);
      pos += (size_t)snprintf(tx + pos, DEBUG_CONSOLE_TX_LINE_SIZE - pos,
        "%ld,%ld,%ld,%ld,%d,%d,%d,%d",
        (long)DebugConsole_Milli(enc.speed_mps[MOTOR_ID_M1]),
        (long)DebugConsole_Milli(enc.speed_mps[MOTOR_ID_M2]),
        (long)DebugConsole_Milli(enc.speed_mps[MOTOR_ID_M3]),
        (long)DebugConsole_Milli(enc.speed_mps[MOTOR_ID_M4]),
        cs.motor_output_permille[MOTOR_ID_M1],
        cs.motor_output_permille[MOTOR_ID_M2],
        cs.motor_output_permille[MOTOR_ID_M3],
        cs.motor_output_permille[MOTOR_ID_M4]);
      break;

    case LOG_FLD_ADC:
      AdcMonitor_GetState(&adc);
      pos += (size_t)snprintf(tx + pos, DEBUG_CONSOLE_TX_LINE_SIZE - pos,
        "%ld,%ld,%ld,%ld,%ld",
        (long)DebugConsole_Milli(adc.battery_voltage),
        (long)DebugConsole_Milli(adc.current_a[MOTOR_ID_M1]),
        (long)DebugConsole_Milli(adc.current_a[MOTOR_ID_M2]),
        (long)DebugConsole_Milli(adc.current_a[MOTOR_ID_M3]),
        (long)DebugConsole_Milli(adc.current_a[MOTOR_ID_M4]));
      break;

    case LOG_FLD_IMU:
      ImuBmi270_GetState(&imu);
      pos += (size_t)snprintf(tx + pos, DEBUG_CONSOLE_TX_LINE_SIZE - pos,
        "%u,%u,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld",
        imu.online, imu.chip_id,
        (long)DebugConsole_Milli(imu.accel_g[0]),
        (long)DebugConsole_Milli(imu.accel_g[1]),
        (long)DebugConsole_Milli(imu.accel_g[2]),
        (long)DebugConsole_Milli(imu.gyro_corrected_dps[0]),
        (long)DebugConsole_Milli(imu.gyro_corrected_dps[1]),
        (long)DebugConsole_Milli(imu.gyro_corrected_dps[2]),
        (long)DebugConsole_Milli(imu.gyro_filtered_dps[0]),
        (long)DebugConsole_Milli(imu.gyro_filtered_dps[1]),
        (long)DebugConsole_Milli(imu.gyro_filtered_dps[2]),
        (long)DebugConsole_Milli(imu.roll_deg),
        (long)DebugConsole_Milli(imu.pitch_deg),
        (long)DebugConsole_Milli(imu.yaw_deg));
      break;

    case LOG_FLD_ERRORS:
      SystemMonitor_GetState(&mon);
      pos += (size_t)snprintf(tx + pos, DEBUG_CONSOLE_TX_LINE_SIZE - pos,
        "%lu", (unsigned long)mon.error_flags);
      break;

    case LOG_FLD_SOURCE:
      SystemMonitor_GetState(&mon);
      pos += (size_t)snprintf(tx + pos, DEBUG_CONSOLE_TX_LINE_SIZE - pos,
        "%u", mon.control_mode);
      break;

    case LOG_FLD_PS2:
      Ps2Control_GetState(&ps2);
      pos += (size_t)snprintf(tx + pos, DEBUG_CONSOLE_TX_LINE_SIZE - pos,
        "%lu,%lu", (unsigned long)ps2.rx_ok_count, (unsigned long)ps2.rx_fail_count);
      break;

    case LOG_FLD_LINE:
      LineUart_GetState(&line);
      pos += (size_t)snprintf(tx + pos, DEBUG_CONSOLE_TX_LINE_SIZE - pos,
        "%lu,%lu", (unsigned long)line.rx_bytes, (unsigned long)line.rx_frames);
      break;

    case LOG_FLD_ESP:
      Esp12fComm_GetState(&esp);
      pos += (size_t)snprintf(tx + pos, DEBUG_CONSOLE_TX_LINE_SIZE - pos,
        "%lu,%lu", (unsigned long)esp.rx_frames, (unsigned long)esp.tx_frames);
      break;

    default:
      break;
  }
  return pos;
}

static void DebugConsole_PrintFilteredLogFrame(uint32_t now_ms)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  size_t pos = 0U;
  uint8_t i;

  pos += (size_t)snprintf(tx + pos, sizeof(tx) - pos, "%lu", (unsigned long)now_ms);
  for (i = 0U; i < log_filter_count; ++i)
  {
    tx[pos++] = ',';
    pos = DebugConsole_WriteFieldData(tx, pos, (log_field_id_t)log_filter_order[i]);
  }
  pos += (size_t)snprintf(tx + pos, sizeof(tx) - pos, "\r\n");
  DebugConsole_Write(tx);
}

static void DebugConsole_PrintLineStatus(void)
{
  char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
  line_sensor_data_t sensor;
  line_control_state_t lc_state;
  line_uart_state_t line_state;

  LineUart_GetSensorData(&sensor);
  LineControl_GetState(&lc_state);
  LineUart_GetState(&line_state);

  (void)snprintf(tx, sizeof(tx),
    "LINE enabled=%u active=%u pos=%.2f err=%.2f lx=%.3f az=%.3f det=%u\r\n"
    "LINE st=%u%u%u%u%u%u%u%u an=%u,%u,%u,%u,%u,%u,%u,%u\r\n"
    "LINE rx_bytes=%lu frames=%lu proto_err=%lu ovf=%lu\r\n",
    lc_state.globally_enabled, lc_state.tracking_active,
    (double)lc_state.line_position, (double)lc_state.error,
    (double)lc_state.linear_x, (double)lc_state.angular_z,
    lc_state.detected_count,
    sensor.state[0], sensor.state[1], sensor.state[2], sensor.state[3],
    sensor.state[4], sensor.state[5], sensor.state[6], sensor.state[7],
    sensor.analog[0], sensor.analog[1], sensor.analog[2], sensor.analog[3],
    sensor.analog[4], sensor.analog[5], sensor.analog[6], sensor.analog[7],
    (unsigned long)line_state.rx_bytes, (unsigned long)line_state.rx_frames,
    (unsigned long)line_state.rx_protocol_errors, (unsigned long)line_state.overflow_count);
  DebugConsole_Write(tx);
}

static void DebugConsole_PrintHelp(void)
{
  DebugConsole_Write(
    "\r\nF407 V2 debug console\r\n"
    "help/status/header\r\n"
    "log 0                  stop streaming\r\n"
    "log 1 [fld...]         start CSV stream, optional field filter\r\n"
    "                       fields: motor adc imu errors source ps2 line esp\r\n"
    "rtos                   heap and task stack status\r\n"
    "motor L R              side open-loop permille\r\n"
    "left P/right P         side open-loop shortcut\r\n"
    "m1 F R ... m4 F R      raw IN1/IN2 permille\r\n"
    "raw LF LR RF RR         left/right pair raw inputs\r\n"
    "vel V [W]              closed-loop mm/s, optional mrad/s\r\n"
    "stop                   clear tests and commands\r\n"
    "estop 0|1              clear/set emergency stop\r\n"
    "clearfault             clear latched overcurrent/driver faults\r\n"
    "line/line on/off       line sensor raw data / toggle control\r\n"
    "imutest/imudiag/imuinit/imucal [n]/imucalclear/imu 0|1\r\n"
    "espreset/espboot 0|1\r\n"
    "espflash on|off|status bridge USART1 to ESP12F (download mode)\r\n"
    "espat on|off          bridge USART1 to ESP12F (normal/AT mode)\r\n"
    "i2cscan               scan I2C1 bus for devices\r\n"
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
  DebugConsole_PrintTaskStatus("oled", oledTaskHandle, ChassisTaskTiming_GetMissedCount(CHASSIS_TASK_TIMING_OLED));

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
  uint32_t idle_ms;

  Esp12fFlashBridge_GetState(&bridge_state);
  idle_ms = Esp12fFlashBridge_GetIdleMs();

  (void)snprintf(tx, sizeof(tx),
                 "ESPFLASH active=%u download=%u idle=%lums pc_rx=%lu pc_tx=%lu esp_rx=%lu esp_tx=%lu ovf=%lu/%lu uart_err=%lu rx_start_err=%lu auto_exit=%lu exit_idle=%lums\r\n",
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
                 (unsigned long)bridge_state.rx_start_errors,
                 (unsigned long)bridge_state.auto_exit_count,
                 (unsigned long)bridge_state.last_auto_exit_idle_ms);
  DebugConsole_Write(tx);
}

static void DebugConsole_PrintHeader(void)
{
  DebugConsole_Write("t_ms,m1_mms,m2_mms,m3_mms,m4_mms,m1_pwm,m2_pwm,m3_pwm,m4_pwm,vbat_mv,m1_ma,m2_ma,m3_ma,m4_ma,imu_online,imu_chip,errors,source,ps2_ok,ps2_fail,line_bytes,line_frames,esp_rx,esp_tx,imu_acc_x_mg,imu_acc_y_mg,imu_acc_z_mg,imu_gyro_corr_x_mdps,imu_gyro_corr_y_mdps,imu_gyro_corr_z_mdps,imu_gyro_filt_x_mdps,imu_gyro_filt_y_mdps,imu_gyro_filt_z_mdps,imu_roll_mdeg,imu_pitch_mdeg,imu_yaw_mdeg\r\n");
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
                 "ADC vbat=%ldmV raw=%u m1=%ldmA raw=%u z=%u m2=%ldmA raw=%u z=%u m3=%ldmA raw=%u z=%u m4=%ldmA raw=%u z=%u cal=%u/%u valid=%u\r\n",
                 (long)DebugConsole_Milli(adc_state.battery_voltage),
                 adc_state.raw_battery,
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M1]), adc_state.raw_current[MOTOR_ID_M1], adc_state.current_zero_raw[MOTOR_ID_M1],
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M2]), adc_state.raw_current[MOTOR_ID_M2], adc_state.current_zero_raw[MOTOR_ID_M2],
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M3]), adc_state.raw_current[MOTOR_ID_M3], adc_state.current_zero_raw[MOTOR_ID_M3],
                 (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M4]), adc_state.raw_current[MOTOR_ID_M4], adc_state.current_zero_raw[MOTOR_ID_M4],
                 adc_state.current_zero_sample_count,
                 (uint16_t)ADC_MONITOR_CURRENT_ZERO_SAMPLES,
                 adc_state.current_valid);
  DebugConsole_Write(tx);

  (void)snprintf(tx, sizeof(tx),
                 "BMI270 enabled=%u online=%u chip=0x%02X err=%u errcnt=%lu gcal=%u gbias_mdps=%ld,%ld,%ld acc_mg=%ld,%ld,%ld corr_mdps=%ld,%ld,%ld filt_mdps=%ld,%ld,%ld euler_mdeg=%ld,%ld,%ld\r\n",
                 imu_state.enabled, imu_state.online, imu_state.chip_id, imu_state.last_error,
                 (unsigned long)imu_state.error_count,
                 imu_state.gyro_calibrated,
                 (long)DebugConsole_Milli(imu_state.gyro_bias_dps[0]),
                 (long)DebugConsole_Milli(imu_state.gyro_bias_dps[1]),
                 (long)DebugConsole_Milli(imu_state.gyro_bias_dps[2]),
                 (long)DebugConsole_Milli(imu_state.accel_g[0]),
                 (long)DebugConsole_Milli(imu_state.accel_g[1]),
                 (long)DebugConsole_Milli(imu_state.accel_g[2]),
                 (long)DebugConsole_Milli(imu_state.gyro_corrected_dps[0]),
                 (long)DebugConsole_Milli(imu_state.gyro_corrected_dps[1]),
                 (long)DebugConsole_Milli(imu_state.gyro_corrected_dps[2]),
                 (long)DebugConsole_Milli(imu_state.gyro_filtered_dps[0]),
                 (long)DebugConsole_Milli(imu_state.gyro_filtered_dps[1]),
                 (long)DebugConsole_Milli(imu_state.gyro_filtered_dps[2]),
                 (long)DebugConsole_Milli(imu_state.roll_deg),
                 (long)DebugConsole_Milli(imu_state.pitch_deg),
                 (long)DebugConsole_Milli(imu_state.yaw_deg));
  DebugConsole_Write(tx);

  (void)snprintf(tx, sizeof(tx),
                 "SYS source=%u errors=0x%08lX latched=0x%08lX reset=0x%08lX bor=%u por=%u iwdg=%u sftr=%u drv_fault=%u,%u,%u,%u line=%lu/%lu esp=%lu/%lu ps2=%u ok=%lu fail=%lu\r\n",
                 monitor_state.control_mode,
                 (unsigned long)monitor_state.error_flags,
                 (unsigned long)monitor_state.latched_error_flags,
                 (unsigned long)boot_reset_flags,
                 DebugConsole_ResetFlag(RCC_CSR_BORRSTF),
                 DebugConsole_ResetFlag(RCC_CSR_PORRSTF),
                 DebugConsole_ResetFlag(RCC_CSR_IWDGRSTF),
                 DebugConsole_ResetFlag(RCC_CSR_SFTRSTF),
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
  DebugConsole_PrintResetTrace();
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
                 "%lu,%ld,%ld,%ld,%ld,%d,%d,%d,%d,%ld,%ld,%ld,%ld,%ld,%u,%u,%lu,%u,%lu,%lu,%lu,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
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
                 (unsigned long)esp_state.tx_frames,
                 (long)DebugConsole_Milli(imu_state.accel_g[0]),
                 (long)DebugConsole_Milli(imu_state.accel_g[1]),
                 (long)DebugConsole_Milli(imu_state.accel_g[2]),
                 (long)DebugConsole_Milli(imu_state.gyro_corrected_dps[0]),
                 (long)DebugConsole_Milli(imu_state.gyro_corrected_dps[1]),
                 (long)DebugConsole_Milli(imu_state.gyro_corrected_dps[2]),
                 (long)DebugConsole_Milli(imu_state.gyro_filtered_dps[0]),
                 (long)DebugConsole_Milli(imu_state.gyro_filtered_dps[1]),
                 (long)DebugConsole_Milli(imu_state.gyro_filtered_dps[2]),
                 (long)DebugConsole_Milli(imu_state.roll_deg),
                 (long)DebugConsole_Milli(imu_state.pitch_deg),
                 (long)DebugConsole_Milli(imu_state.yaw_deg));
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
  int value = 0;
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
  else if (strncmp(line, "log ", 4) == 0)
  {
    char *token = strtok(line + 4, " \t");

    if (token == 0)
    {
      DebugConsole_Write("usage: log 0 | log 1 [field...]\r\n");
    }
    else
    {
      int log_on = atoi(token);

      if (log_on == 0)
      {
        stream_mode = 0U;
        log_filter_count = 0U;
        DebugConsole_Write("log off\r\n");
      }
      else
      {
        token = strtok(0, " \t");
        if (token == 0)
        {
          /* log 1（无过滤）：全字段，兼容旧行为 */
          stream_mode = 1U;
          log_filter_count = 0U;
          DebugConsole_PrintHeader();
        }
        else
        {
          /* log 1 field1 field2 ...：仅输出指定字段 */
          uint8_t count = 0U;
          uint8_t order[8];
          uint8_t ok = 1U;

          while (token != 0 && count < 8U)
          {
            uint8_t j;
            int8_t found = -1;
            for (j = 0U; j < LOG_FLD_COUNT; ++j)
            {
              if (strcmp(token, log_field_names[j]) == 0)
              {
                found = (int8_t)j;
                break;
              }
            }
            if (found < 0)
            {
              ok = 0U;
              break;
            }
            order[count++] = (uint8_t)found;
            token = strtok(0, " \t");
          }

          if (ok == 0U || count == 0U)
          {
            DebugConsole_Write("unknown field, valid: motor adc imu errors source ps2 line esp\r\n");
          }
          else
          {
            stream_mode = 2U;
            log_filter_count = count;
            for (uint8_t k = 0U; k < count; ++k)
            {
              log_filter_order[k] = order[k];
            }
            DebugConsole_PrintFilteredHeader();
          }
        }
      }
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
  else if (strcmp(line, "imudiag") == 0)
  {
    char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
    imu_bmi270_diag_t diag;

    if (ImuBmi270_Diagnose(&diag) == 0U)
    {
      DebugConsole_Write("bmi270 diag failed\r\n");
      return;
    }

    (void)snprintf(tx, sizeof(tx),
                   "BMI270 diag hal1 st=%u rx=%02X,%02X,%02X hal2 st=%u rx=%02X,%02X,%02X\r\n",
                   diag.hal_status[0], diag.hal_rx[0][0], diag.hal_rx[0][1], diag.hal_rx[0][2],
                   diag.hal_status[1], diag.hal_rx[1][0], diag.hal_rx[1][1], diag.hal_rx[1][2]);
    DebugConsole_Write(tx);
    (void)snprintf(tx, sizeof(tx),
                   "BMI270 diag bitbang rx=%02X,%02X,%02X miso nopull=%u pullup=%u pulldown=%u\r\n",
                   diag.bitbang_rx[0], diag.bitbang_rx[1], diag.bitbang_rx[2],
                   diag.miso_nopull, diag.miso_pullup, diag.miso_pulldown);
    DebugConsole_Write(tx);
  }
  else if (strcmp(line, "imuinit") == 0)
  {
    DebugConsole_Write((ImuBmi270_ConfigNow() != 0U) ? "bmi270 init ok\r\n" : "bmi270 init failed\r\n");
  }
  else if ((strcmp(line, "imucal") == 0) || (sscanf(line, "imucal %d", &value) == 1))
  {
    char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
    uint16_t samples = (value > 0) ? (uint16_t)value : 0U;
    imu_bmi270_state_t imu_state;

    DebugConsole_Write("bmi270 gyro calibration: keep still\r\n");
    if (ImuBmi270_CalibrateGyro(samples, 10U) == 0U)
    {
      DebugConsole_Write("bmi270 gyro calibration failed: keep still and retry\r\n");
      return;
    }

    ImuBmi270_GetState(&imu_state);
    (void)snprintf(tx, sizeof(tx),
                   "bmi270 gyro calibration ok bias_mdps=%ld,%ld,%ld\r\n",
                   (long)DebugConsole_Milli(imu_state.gyro_bias_dps[0]),
                   (long)DebugConsole_Milli(imu_state.gyro_bias_dps[1]),
                   (long)DebugConsole_Milli(imu_state.gyro_bias_dps[2]));
    DebugConsole_Write(tx);
  }
  else if (strcmp(line, "imucalclear") == 0)
  {
    ImuBmi270_ClearCalibration();
    DebugConsole_Write("bmi270 gyro calibration cleared\r\n");
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
    stream_mode = 0U;
    debug_velocity_enabled = 0U;
    if (Esp12fFlashBridge_Enable(1U) != 0U)
    {
      DebugConsole_Write("esp12f flash bridge on: close this terminal and use esptool/Arduino at 115200\r\n");
    }
    else
    {
      DebugConsole_Write("esp12f flash bridge failed: UART RX not ready\r\n");
    }
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
  else if (strcmp(line, "espat on") == 0)
  {
    stream_mode = 0U;
    debug_velocity_enabled = 0U;
    if (Esp12fFlashBridge_Enable(0U) != 0U)
    {
      DebugConsole_Write("AT passthrough active: IO0=high, USART1<->USART2 bridge open.\r\n"
                         "Type AT commands directly. Auto-exit after 30s idle.\r\n");
    }
    else
    {
      DebugConsole_Write("AT passthrough failed: UART RX not ready\r\n");
    }
  }
  else if (strcmp(line, "espat off") == 0)
  {
    Esp12fFlashBridge_Disable();
    DebugConsole_Write("AT passthrough off, normal boot requested\r\n");
  }
  else if (strcmp(line, "line") == 0)
  {
    DebugConsole_PrintLineStatus();
  }
  else if (strcmp(line, "line on") == 0)
  {
    LineControl_Enable(1U);
    DebugConsole_Write("line tracking enabled\r\n");
  }
  else if (strcmp(line, "line off") == 0)
  {
    LineControl_Enable(0U);
    DebugConsole_Write("line tracking disabled\r\n");
  }
  else if (strcmp(line, "i2cscan") == 0)
  {
    char tx[DEBUG_CONSOLE_TX_LINE_SIZE];
    int pos = 0;
    uint8_t found = 0U;
    pos += snprintf(tx + pos, sizeof(tx) - pos, "I2C1 scan:\r\n");
    for (uint8_t addr = 1U; addr < 128U; addr++)
    {
      HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(&hi2c1,
                               (uint16_t)(addr << 1), 1, 5);
      if (st == HAL_OK)
      {
        found = 1U;
        pos += snprintf(tx + pos, sizeof(tx) - pos,
                        "  0x%02X (7-bit)  ACK\r\n", addr);
      }
    }
    if (found == 0U)
    {
      pos += snprintf(tx + pos, sizeof(tx) - pos, "  no device found\r\n");
    }
    DebugConsole_Write(tx);
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
        if (Esp12fFlashBridge_IsActive() != 0U)
        {
          Usart1DebugConsole_ClearRxBuffer();
          break;
        }
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
  DebugConsole_CaptureResetFlags();
  rx_len = 0U;
  stream_mode = 0U;
  debug_velocity_enabled = 0U;
  debug_velocity_cmd = (chassis_cmd_t){0};
  rx_head = 0U;
  rx_tail = 0U;
  HAL_NVIC_SetPriority(USART1_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  Usart1DebugConsole_RestartRx();
  DebugConsole_Write("\r\nF407 V2 chassis firmware\r\n");
  DebugConsole_PrintResetFlags();
  DebugConsole_PrintResetTrace();
  DebugConsole_Write("USART1 debug console ready, type help\r\n");
}

void Usart1DebugConsole_ClearRxBuffer(void)
{
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  rx_len = 0U;
  rx_head = 0U;
  rx_tail = 0U;
  __set_PRIMASK(primask);
}

void Usart1DebugConsole_RestartRx(void)
{
  Usart1DebugConsole_ClearRxBuffer();
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

    ResetTrace_TaskHeartbeat(RESET_TRACE_TASK_DEBUG, now_ms);
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

    if ((stream_mode != 0U) && ((now_ms - last_log_ms) >= DEBUG_CONSOLE_LOG_PERIOD_MS))
    {
      last_log_ms = now_ms;
      if (stream_mode == 2U)
      {
        DebugConsole_PrintFilteredLogFrame(now_ms);
      }
      else
      {
        DebugConsole_PrintLogFrame(now_ms);
      }
    }

    osDelay(DEBUG_CONSOLE_TASK_PERIOD_MS);
  }
}
