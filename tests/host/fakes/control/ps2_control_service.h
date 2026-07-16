#ifndef PS2_CONTROL_SERVICE_H
#define PS2_CONTROL_SERVICE_H

#include "teleoperation_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define PS2_HEADING_GATE_IMU_OFFLINE      TELEOPERATION_HEADING_GATE_IMU_OFFLINE
#define PS2_HEADING_GATE_IMU_UNCALIBRATED TELEOPERATION_HEADING_GATE_IMU_UNCALIBRATED
#define PS2_HEADING_GATE_IMU_STALE        TELEOPERATION_HEADING_GATE_IMU_STALE
#define PS2_HEADING_GATE_IMU_QUALITY      TELEOPERATION_HEADING_GATE_IMU_QUALITY

    typedef teleoperation_status_t ps2_control_service_state_t;

    void Ps2ControlService_Init(void);
    void Ps2ControlService_Update(void);
    void Ps2ControlService_GetState(ps2_control_service_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
