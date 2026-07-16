#include "debug_system_status.h"

#include "power_adc_driver.h"
#include "motor_hardware_layout.h"
#include "motion_control_service.h"
#include "power_adc_driver_config.h"
#include "debug_cmd_current.h"
#include "debug_console_writer.h"
#include "wheel_encoder_driver.h"
#include "wireless_communication_service.h"
#include "bmi270_driver.h"
#include "line_sensor_driver.h"
#include "motor_driver.h"
#include "parameter_management_service.h"
#include "platform_reset.h"
#include "platform_reset_trace.h"
#include "power_on_self_test_service.h"
#include "teleoperation_service.h"
#include "safety_management_service.h"
#include "system_monitoring_service.h"
#include "host_communication_service.h"

#include <stdio.h>

#define DEBUG_SYSTEM_STATUS_TX_SIZE 1536U

static int32_t DebugConsole_Milli(float value)
{
    return (int32_t)(value * 1000.0f);
}

static uint8_t DebugSystemStatus_ResetFlag(platform_reset_reason_t reason)
{
    return PlatformReset_ReasonFlagSet(SystemMonitoring_GetResetReason(), reason);
}

void DebugSystemStatus_PrintResetFlags(void)
{
    char tx[DEBUG_SYSTEM_STATUS_TX_SIZE];

    (void)snprintf(tx,
                   sizeof(tx),
                   "RESET csr=0x%08lX bor=%u por=%u pin=%u sftr=%u iwdg=%u wwdg=%u lpwr=%u\r\n",
                   (unsigned long)SystemMonitoring_GetResetReason(),
                   DebugSystemStatus_ResetFlag(PLATFORM_RESET_REASON_BROWNOUT),
                   DebugSystemStatus_ResetFlag(PLATFORM_RESET_REASON_POWER_ON),
                   DebugSystemStatus_ResetFlag(PLATFORM_RESET_REASON_PIN),
                   DebugSystemStatus_ResetFlag(PLATFORM_RESET_REASON_SOFTWARE),
                   DebugSystemStatus_ResetFlag(PLATFORM_RESET_REASON_INDEPENDENT_WATCHDOG),
                   DebugSystemStatus_ResetFlag(PLATFORM_RESET_REASON_WINDOW_WATCHDOG),
                   DebugSystemStatus_ResetFlag(PLATFORM_RESET_REASON_LOW_POWER));
    DebugConsoleWriter_Write(tx);
}

void DebugSystemStatus_PrintResetTrace(void)
{
    char                 tx[DEBUG_SYSTEM_STATUS_TX_SIZE];
    reset_trace_record_t trace;
    uint8_t              valid = PlatformResetTrace_GetBootRecord(&trace);

    (void)snprintf(tx,
                   sizeof(tx),
                   "RESETTRACE valid=%u kind=%lu reason=%lu task=%lu line=%lu cfsr=0x%08lX hfsr=0x%08lX bfar=0x%08lX "
                   "mmfar=0x%08lX pc=0x%08lX lr=0x%08lX xpsr=0x%08lX exc=0x%08lX sp=0x%08lX msp=0x%08lX psp=0x%08lX "
                   "ctrl=0x%08lX fpccr=0x%08lX dma_lisr=0x%08lX dma_cr=0x%08lX dma_ndtr=%lu dma_fcr=0x%08lX "
                   "adc_sr=0x%08lX adc_cr2=0x%08lX d0=0x%08lX d1=0x%08lX d2=0x%08lX d3=0x%08lX safety=%lu motor=%lu "
                   "ps2=%lu esp=%lu debug=%lu source=%lu estop=%lu fault=%lu\r\n",
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
                   (unsigned long)trace.msp,
                   (unsigned long)trace.psp,
                   (unsigned long)trace.control,
                   (unsigned long)trace.fpccr,
                   (unsigned long)trace.dma2_lisr,
                   (unsigned long)trace.dma2_stream0_cr,
                   (unsigned long)trace.dma2_stream0_ndtr,
                   (unsigned long)trace.dma2_stream0_fcr,
                   (unsigned long)trace.adc1_sr,
                   (unsigned long)trace.adc1_cr2,
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
    DebugConsoleWriter_Write(tx);
}

void DebugSystemStatus_Print(void)
{
    char                           tx[DEBUG_SYSTEM_STATUS_TX_SIZE];
    power_adc_driver_state_t       adc_state;
    wheel_encoder_state_t          encoder_state;
    motion_control_status_t        chassis_state;
    safety_management_status_t     monitor_state;
    bmi270_driver_state_t          imu_state;
    teleoperation_status_t         ps2_state;
    line_sensor_driver_state_t     line_state;
    wireless_communication_state_t esp_state;
    motor_driver_state_t           motor_state;
    power_on_self_test_result_t    post_result;
    param_model_t                  params;
    uint32_t                       encoder_hw_count[MOTOR_ID_COUNT];

    PowerAdcDriver_GetState(&adc_state);
    WheelEncoderDriver_GetState(&encoder_state);
    (void)MotionControl_GetStatus(&chassis_state);
    (void)SafetyManagement_GetStatus(&monitor_state);
    Bmi270Driver_GetState(&imu_state);
    (void)Teleoperation_GetStatus(&ps2_state);
    LineSensorDriver_GetState(&line_state);
    WirelessCommunication_GetState(&esp_state);
    MotorDriver_GetState(&motor_state);
    PowerOnSelfTest_GetResult(&post_result);
    (void)ParameterManagement_GetSnapshot(&params);
    WheelEncoderDriver_GetHardwareCounts(encoder_hw_count);

    (void)snprintf(tx,
                   sizeof(tx),
                   "ENC m1=%ld d=%ld %ldmm/s v=%u m2=%ld d=%ld %ldmm/s v=%u m3=%ld d=%ld %ldmm/s v=%u m4=%ld d=%ld "
                   "%ldmm/s v=%u hw=%lu,%lu,%lu,%lu\r\n",
                   (long)encoder_state.count[MOTOR_ID_M1],
                   (long)encoder_state.delta[MOTOR_ID_M1],
                   (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M1]),
                   encoder_state.speed_valid[MOTOR_ID_M1],
                   (long)encoder_state.count[MOTOR_ID_M2],
                   (long)encoder_state.delta[MOTOR_ID_M2],
                   (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M2]),
                   encoder_state.speed_valid[MOTOR_ID_M2],
                   (long)encoder_state.count[MOTOR_ID_M3],
                   (long)encoder_state.delta[MOTOR_ID_M3],
                   (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M3]),
                   encoder_state.speed_valid[MOTOR_ID_M3],
                   (long)encoder_state.count[MOTOR_ID_M4],
                   (long)encoder_state.delta[MOTOR_ID_M4],
                   (long)DebugConsole_Milli(encoder_state.speed_mps[MOTOR_ID_M4]),
                   encoder_state.speed_valid[MOTOR_ID_M4],
                   (unsigned long)encoder_hw_count[MOTOR_ID_M1],
                   (unsigned long)encoder_hw_count[MOTOR_ID_M2],
                   (unsigned long)encoder_hw_count[MOTOR_ID_M3],
                   (unsigned long)encoder_hw_count[MOTOR_ID_M4]);
    DebugConsoleWriter_Write(tx);

    (void)snprintf(tx,
                   sizeof(tx),
                   "BMI270 profile=%u init=%u stime=%lu valid=%u samples=%lu drdy=%lu poll=%lu q=%.4f,%.4f,%.4f,%.4f "
                   "quality=0x%08lX latched=0x%08lX qcnt=%lu,%lu,%lu,%lu,%lu,%lu,%lu\r\n",
                   imu_state.profile,
                   imu_state.init_state,
                   (unsigned long)imu_state.sensor_time,
                   imu_state.sensor_time_valid,
                   (unsigned long)imu_state.sample_count,
                   (unsigned long)imu_state.drdy_count,
                   (unsigned long)imu_state.poll_fallback_count,
                   imu_state.quaternion[0],
                   imu_state.quaternion[1],
                   imu_state.quaternion[2],
                   imu_state.quaternion[3],
                   (unsigned long)imu_state.quality_flags,
                   (unsigned long)imu_state.quality_latched_flags,
                   (unsigned long)imu_state.spi_error_count,
                   (unsigned long)imu_state.init_failure_count,
                   (unsigned long)imu_state.fifo_overflow_count,
                   (unsigned long)imu_state.timestamp_error_count,
                   (unsigned long)imu_state.gyro_saturation_count,
                   (unsigned long)imu_state.accel_anomaly_count,
                   (unsigned long)imu_state.attitude_invalid_count);
    DebugConsoleWriter_Write(tx);

    (void)snprintf(
        tx,
        sizeof(tx),
        "CHASSIS req=%ld,%ldmm/s target=%ld,%ldmm/s actual=%ld,%ldmm/s pwm=%d,%d,%d,%d out=%u estop=%u fault=%u\r\n",
        (long)DebugConsole_Milli(chassis_state.left_requested_mps),
        (long)DebugConsole_Milli(chassis_state.right_requested_mps),
        (long)DebugConsole_Milli(chassis_state.left_target_mps),
        (long)DebugConsole_Milli(chassis_state.right_target_mps),
        (long)DebugConsole_Milli(chassis_state.left_actual_mps),
        (long)DebugConsole_Milli(chassis_state.right_actual_mps),
        motor_state.effective_pwm[MOTOR_ID_M1],
        motor_state.effective_pwm[MOTOR_ID_M2],
        motor_state.effective_pwm[MOTOR_ID_M3],
        motor_state.effective_pwm[MOTOR_ID_M4],
        chassis_state.output_enabled,
        SafetyManagement_IsEmergencyStop(),
        SafetyManagement_IsFaultStop());
    DebugConsoleWriter_Write(tx);

    (void)snprintf(tx,
                   sizeof(tx),
                   "BREAK tim1 moe=%u bif=%u count=%lu last=%lu origin=%u startup=%u pre_bif=%u bkin=%u nfault=0x%02X "
                   "tim8 moe=%u bif=%u count=%lu last=%lu edge=%lu,%lu,%lu,%lu low=%lu,%lu,%lu,%lu\r\n",
                   motor_state.tim1_moe_active,
                   motor_state.tim1_break_flag,
                   (unsigned long)motor_state.tim1_break_count,
                   (unsigned long)motor_state.tim1_break_last_ms,
                   (unsigned int)motor_state.break_origin,
                   motor_state.startup_qualified,
                   motor_state.startup_pre_wake_bif,
                   motor_state.startup_bkin_high,
                   motor_state.startup_nfault_high_mask,
                   motor_state.tim8_moe_active,
                   motor_state.tim8_break_flag,
                   (unsigned long)motor_state.tim8_break_count,
                   (unsigned long)motor_state.tim8_break_last_ms,
                   (unsigned long)motor_state.fault_edge_count[MOTOR_ID_M1],
                   (unsigned long)motor_state.fault_edge_count[MOTOR_ID_M2],
                   (unsigned long)motor_state.fault_edge_count[MOTOR_ID_M3],
                   (unsigned long)motor_state.fault_edge_count[MOTOR_ID_M4],
                   (unsigned long)motor_state.fault_low_since_ms[MOTOR_ID_M1],
                   (unsigned long)motor_state.fault_low_since_ms[MOTOR_ID_M2],
                   (unsigned long)motor_state.fault_low_since_ms[MOTOR_ID_M3],
                   (unsigned long)motor_state.fault_low_since_ms[MOTOR_ID_M4]);
    DebugConsoleWriter_Write(tx);

    (void)snprintf(tx,
                   sizeof(tx),
                   "ADC vbat=%ldmV raw=%u m1=%ldmA raw=%u z=%u m2=%ldmA raw=%u z=%u m3=%ldmA raw=%u z=%u m4=%ldmA "
                   "raw=%u z=%u cal=%u/%u valid=%u flags=0x%08lX invalid=0x%08lX raw_n=%u miss=%u rate_mHz=%lu\r\n",
                   (long)DebugConsole_Milli(adc_state.battery_voltage),
                   adc_state.raw_battery,
                   (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M1]),
                   adc_state.raw_current[MOTOR_ID_M1],
                   adc_state.current_zero_raw[MOTOR_ID_M1],
                   (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M2]),
                   adc_state.raw_current[MOTOR_ID_M2],
                   adc_state.current_zero_raw[MOTOR_ID_M2],
                   (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M3]),
                   adc_state.raw_current[MOTOR_ID_M3],
                   adc_state.current_zero_raw[MOTOR_ID_M3],
                   (long)DebugConsole_Milli(adc_state.current_a[MOTOR_ID_M4]),
                   adc_state.raw_current[MOTOR_ID_M4],
                   adc_state.current_zero_raw[MOTOR_ID_M4],
                   adc_state.current_zero_sample_count,
                   (uint16_t)POWER_ADC_DRIVER_CURRENT_ZERO_SAMPLES,
                   adc_state.current_valid,
                   (unsigned long)adc_state.valid_flags,
                   (unsigned long)adc_state.invalid_reason_flags,
                   adc_state.raw_sample_count,
                   adc_state.missed_window_count,
                   (unsigned long)adc_state.sample_rate_hz_milli);
    DebugConsoleWriter_Write(tx);
    DebugCmdCurrent_PrintCalibrationStatus();

    (void)snprintf(tx,
                   sizeof(tx),
                   "POST done=%u errors=0x%08lX drv=%s(mask=0x%02X) adc=%s imu=%s(chip=0x%02X) enc=%s\r\n",
                   post_result.done,
                   (unsigned long)post_result.error_flags,
                   PowerOnSelfTest_ItemStatusString(post_result.drv_status),
                   post_result.drv_fault_mask,
                   PowerOnSelfTest_ItemStatusString(post_result.adc_status),
                   PowerOnSelfTest_ItemStatusString(post_result.imu_status),
                   post_result.imu_chip_id,
                   PowerOnSelfTest_ItemStatusString(post_result.encoder_status));
    DebugConsoleWriter_Write(tx);

    (void)snprintf(tx,
                   sizeof(tx),
                   "PARAM vmax=%ldmm/s wmax=%ldmrad/s ramp=%ldmm/s2 wr=%ldum track=%ldum gcal_valid=%u\r\n",
                   (long)DebugConsole_Milli(params.max_linear_mps),
                   (long)DebugConsole_Milli(params.max_angular_rps),
                   (long)DebugConsole_Milli(params.speed_ramp_mps2),
                   (long)(params.wheel_radius_m * 1000000.0f),
                   (long)(params.track_width_m * 1000000.0f),
                   params.imu_gyro_bias_valid);
    DebugConsoleWriter_Write(tx);

    (void)snprintf(tx,
                   sizeof(tx),
                   "ADCWIN m1 mean=%ld rms=%ld pk=%ld n=%u m2 mean=%ld rms=%ld pk=%ld n=%u m3 mean=%ld rms=%ld pk=%ld "
                   "n=%u m4 mean=%ld rms=%ld pk=%ld n=%u\r\n",
                   (long)DebugConsole_Milli(adc_state.current_mean_a[MOTOR_ID_M1]),
                   (long)DebugConsole_Milli(adc_state.current_rms_a[MOTOR_ID_M1]),
                   (long)DebugConsole_Milli(adc_state.current_peak_a[MOTOR_ID_M1]),
                   adc_state.current_sample_count[MOTOR_ID_M1],
                   (long)DebugConsole_Milli(adc_state.current_mean_a[MOTOR_ID_M2]),
                   (long)DebugConsole_Milli(adc_state.current_rms_a[MOTOR_ID_M2]),
                   (long)DebugConsole_Milli(adc_state.current_peak_a[MOTOR_ID_M2]),
                   adc_state.current_sample_count[MOTOR_ID_M2],
                   (long)DebugConsole_Milli(adc_state.current_mean_a[MOTOR_ID_M3]),
                   (long)DebugConsole_Milli(adc_state.current_rms_a[MOTOR_ID_M3]),
                   (long)DebugConsole_Milli(adc_state.current_peak_a[MOTOR_ID_M3]),
                   adc_state.current_sample_count[MOTOR_ID_M3],
                   (long)DebugConsole_Milli(adc_state.current_mean_a[MOTOR_ID_M4]),
                   (long)DebugConsole_Milli(adc_state.current_rms_a[MOTOR_ID_M4]),
                   (long)DebugConsole_Milli(adc_state.current_peak_a[MOTOR_ID_M4]),
                   adc_state.current_sample_count[MOTOR_ID_M4]);
    DebugConsoleWriter_Write(tx);

    (void)snprintf(tx,
                   sizeof(tx),
                   "BMI270 enabled=%u online=%u chip=0x%02X err=%u errcnt=%lu gcal=%u acal=%u,%u,%u temp=%.1fC "
                   "temp_ok=%u gbias_dps=%.3f,%.3f,%.3f acc_g=%.3f,%.3f,%.3f corr_dps=%.2f,%.2f,%.2f "
                   "filt_dps=%.2f,%.2f,%.2f euler_deg=%.1f,%.1f,%.1f\r\n",
                   imu_state.enabled,
                   imu_state.online,
                   imu_state.chip_id,
                   imu_state.last_error,
                   (unsigned long)imu_state.error_count,
                   imu_state.gyro_calibrated,
                   imu_state.gyro_auto_cal_state,
                   imu_state.gyro_auto_cal_attempts,
                   imu_state.gyro_auto_cal_last_result,
                   imu_state.temperature_c,
                   imu_state.temperature_valid,
                   imu_state.gyro_bias_dps[0],
                   imu_state.gyro_bias_dps[1],
                   imu_state.gyro_bias_dps[2],
                   imu_state.accel_g[0],
                   imu_state.accel_g[1],
                   imu_state.accel_g[2],
                   imu_state.gyro_corrected_dps[0],
                   imu_state.gyro_corrected_dps[1],
                   imu_state.gyro_corrected_dps[2],
                   imu_state.gyro_filtered_dps[0],
                   imu_state.gyro_filtered_dps[1],
                   imu_state.gyro_filtered_dps[2],
                   imu_state.roll_deg,
                   imu_state.pitch_deg,
                   imu_state.yaw_deg);
    DebugConsoleWriter_Write(tx);

    (void)snprintf(tx,
                   sizeof(tx),
                   "SYS source=%u errors=0x%08lX latched=0x%08lX reset=0x%08lX bor=%u por=%u iwdg=%u sftr=%u "
                   "drv_fault=%u,%u,%u,%u line=%lu/%lu esp=%lu/%lu ps2=%u ok=%lu fail=%lu\r\n",
                   monitor_state.control_mode,
                   (unsigned long)monitor_state.error_flags,
                   (unsigned long)monitor_state.latched_error_flags,
                   (unsigned long)SystemMonitoring_GetResetReason(),
                   DebugSystemStatus_ResetFlag(PLATFORM_RESET_REASON_BROWNOUT),
                   DebugSystemStatus_ResetFlag(PLATFORM_RESET_REASON_POWER_ON),
                   DebugSystemStatus_ResetFlag(PLATFORM_RESET_REASON_INDEPENDENT_WATCHDOG),
                   DebugSystemStatus_ResetFlag(PLATFORM_RESET_REASON_SOFTWARE),
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
    DebugConsoleWriter_Write(tx);
    (void)snprintf(tx,
                   sizeof(tx),
                   "PS2 online=%u btn=%02X/%02X edge=%02X axis=%u,%u,%u,%u heading=%u button=%02X target=%.1f "
                   "accum=%.1f end=%u imu_age=%lu gate=0x%08lX\r\n",
                   ps2_state.online,
                   ps2_state.btn1,
                   ps2_state.btn2,
                   ps2_state.pressed_btn2,
                   ps2_state.left_x,
                   ps2_state.left_y,
                   ps2_state.right_x,
                   ps2_state.right_y,
                   ps2_state.heading_active,
                   ps2_state.macro_button,
                   ps2_state.heading_target_deg,
                   ps2_state.heading_accumulated_deg,
                   ps2_state.heading_end_reason,
                   (unsigned long)ps2_state.imu_age_ms,
                   (unsigned long)ps2_state.heading_gate_flags);
    DebugConsoleWriter_Write(tx);
    DebugSystemStatus_PrintResetTrace();
}
