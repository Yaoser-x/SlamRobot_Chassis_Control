#ifndef CURRENT_SENSOR_SERVICE_H
#define CURRENT_SENSOR_SERVICE_H

#include "adc_monitor.h"

typedef adc_monitor_state_t current_sensor_snapshot_t;

void CurrentSensorService_Init(void);
void CurrentSensorService_UpdateStationary(void);
void CurrentSensorService_ApplyZeroCalibration(const uint16_t zero_raw[MOTOR_ID_COUNT]);
void CurrentSensorService_GetSnapshot(current_sensor_snapshot_t *snapshot);

#endif
