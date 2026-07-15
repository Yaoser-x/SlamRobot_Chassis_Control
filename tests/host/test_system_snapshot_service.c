#include "system_snapshot_service.h"

#include "adc_monitor.h"
#include "chassis_service.h"
#include "control_service.h"
#include "current_sensor_service.h"
#include "encoder_service.h"
#include "esp12f_service.h"
#include "imu_service.h"
#include "line_uart.h"
#include "motor_driver.h"
#include "platform_critical.h"
#include "ps2_control_service.h"
#include "safety_service.h"
#include "upper_uart_service.h"

#include <assert.h>
#include <stdio.h>

static chassis_service_snapshot_t  fake_chassis;
static encoder_service_snapshot_t  fake_encoder;
static current_sensor_snapshot_t   fake_current;
static imu_service_snapshot_t      fake_imu;
static motor_driver_state_t        fake_motor;
static safety_service_snapshot_t   fake_safety;
static chassis_task_health_t       fake_task_health;
static post_result_t               fake_post;
static upper_uart_service_state_t  fake_upper;
static esp12f_service_state_t      fake_esp;
static ps2_control_service_state_t fake_ps2;
static line_sensor_data_t          fake_line;
static uint32_t                    fake_upper_last_rx_timestamp_ms;

platform_critical_state_t PlatformCritical_Enter(void)
{
    return 0U;
}
void PlatformCritical_Exit(platform_critical_state_t state)
{
    (void)state;
}
void ChassisService_GetState(chassis_service_snapshot_t *state)
{
    *state = fake_chassis;
}
void EncoderService_GetSnapshot(encoder_service_snapshot_t *state)
{
    *state = fake_encoder;
}
void CurrentSensorService_GetSnapshot(current_sensor_snapshot_t *state)
{
    *state = fake_current;
}
void ImuService_GetSnapshot(imu_service_snapshot_t *state)
{
    *state = fake_imu;
}
void MotorDriver_GetState(motor_driver_state_t *state)
{
    *state = fake_motor;
}
void SafetyService_GetState(safety_service_snapshot_t *state)
{
    *state = fake_safety;
}
void TaskHealthService_GetHealth(chassis_task_health_t *health)
{
    *health = fake_task_health;
}
void POST_GetResult(post_result_t *result)
{
    *result = fake_post;
}
void UpperUartService_GetState(upper_uart_service_state_t *state)
{
    *state = fake_upper;
}
uint32_t UpperUartService_GetLastRxTimestamp(void)
{
    return fake_upper_last_rx_timestamp_ms;
}
void Esp12fService_GetState(esp12f_service_state_t *state)
{
    *state = fake_esp;
}
void Ps2ControlService_GetState(ps2_control_service_state_t *state)
{
    *state = fake_ps2;
}
uint8_t LineUart_GetSensorData(line_sensor_data_t *data)
{
    *data = fake_line;
    return fake_line.valid;
}
uint8_t ChassisLayout_MotorEnabled(motor_id_t motor)
{
    return (motor < MOTOR_ID_COUNT) ? 1U : 0U;
}
uint8_t ControlService_IsEmergencyStop(void)
{
    return 1U;
}
uint8_t ControlService_IsFaultStop(void)
{
    return 0U;
}
uint8_t ControlService_GetActiveSource(void)
{
    return CONTROL_SOURCE_UPPER;
}
uint8_t LineControlService_IsEnabled(void)
{
    return 1U;
}
uint32_t ResetReasonService_GetFlags(void)
{
    return 0x1234UL;
}

static void SeedInputs(void)
{
    fake_chassis                      = (chassis_service_snapshot_t){0};
    fake_encoder                      = (encoder_service_snapshot_t){0};
    fake_current                      = (current_sensor_snapshot_t){0};
    fake_imu                          = (imu_service_snapshot_t){0};
    fake_motor                        = (motor_driver_state_t){0};
    fake_safety                       = (safety_service_snapshot_t){0};
    fake_task_health                  = (chassis_task_health_t){0};
    fake_post                         = (post_result_t){0};
    fake_upper                        = (upper_uart_service_state_t){0};
    fake_esp                          = (esp12f_service_state_t){0};
    fake_ps2                          = (ps2_control_service_state_t){0};
    fake_line                         = (line_sensor_data_t){0};
    fake_upper_last_rx_timestamp_ms   = 800U;
    fake_chassis.motor_target_mps[0]  = 0.5f;
    fake_chassis.motor_actual_mps[0]  = 0.4f;
    fake_encoder.count[0]             = 42;
    fake_encoder.speed_valid[0]       = 1U;
    fake_encoder.speed_valid_all      = 1U;
    fake_encoder.anomaly_count[1]     = 1U;
    fake_current.current_valid        = 1U;
    fake_current.invalid_reason_flags = 0x55UL;
    fake_imu.online                   = 1U;
    fake_imu.chip_id                  = 0x24U;
    fake_imu.gyro_calibrated          = 1U;
    fake_imu.last_update_ms           = 990U;
    fake_imu.sample_count             = 7U;
    fake_motor.effective_pwm[0]       = 123;
    fake_motor.fault_active[1]        = 1U;
    fake_safety.battery_voltage       = 12.3f;
    fake_safety.motor_current_a[0]    = 0.7f;
    fake_safety.control_mode          = CONTROL_SOURCE_UPPER;
    fake_post.done                    = 1U;
    fake_upper.last_valid_frame_ms    = 800U;
    fake_esp.last_rx_timestamp_ms     = 850U;
    fake_ps2.online                   = 1U;
    fake_line.valid                   = 1U;
    fake_line.timestamp_ms            = 980U;
}

int main(void)
{
    system_snapshot_t snapshot;

    SeedInputs();
    SystemSnapshotService_Init();
    assert(SystemSnapshotService_Get(0) == 0U);
    assert(SystemSnapshotService_Get(&snapshot) == 0U);
    SystemSnapshotService_Update(1000U);
    assert(SystemSnapshotService_Get(&snapshot) == 1U);
    assert(snapshot.generation == 1U);
    assert(snapshot.chassis.motor_target_mps[0] == 0.5f);
    assert(snapshot.chassis.motor_output_permille[0] == 123);
    assert(snapshot.encoder.count[0] == 42);
    assert(snapshot.encoder.anomaly_mask == (1U << 1));
    assert(snapshot.safety.battery_voltage == 12.3f);
    assert(snapshot.safety.motor_fault_mask == (1U << 1));
    assert(snapshot.imu.chip_id == 0x24U);
    assert(snapshot.imu.sample_count == 7U);
    assert(snapshot.control.emergency_stop == 1U);
    assert(snapshot.control.line_enabled == 1U);
    assert(snapshot.control.reset_reason_flags == 0x1234UL);
    assert(snapshot.modules.upper_online == 1U);
    assert(snapshot.modules.esp12f_online == 1U);
    assert(snapshot.modules.line_online == 1U);
    assert(snapshot.modules.ps2_online == 1U);

    fake_upper.last_valid_frame_ms  = 1U;
    fake_upper_last_rx_timestamp_ms = 1U;
    fake_esp.boot_mode_download     = 1U;
    fake_line.timestamp_ms          = 1U;
    SystemSnapshotService_Update(2000U);
    assert(SystemSnapshotService_Get(&snapshot) == 2U);
    assert(snapshot.modules.upper_online == 0U);
    assert(snapshot.modules.esp12f_online == 0U);
    assert(snapshot.modules.line_online == 0U);
    puts("system snapshot service tests passed");
    return 0;
}
