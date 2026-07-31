#ifndef CONTROL_MODE_COORDINATOR_H
#define CONTROL_MODE_COORDINATOR_H

#include <stdint.h>

#include "control_mode_config.h"
#include "control_mode_types.h"
#include "teleoperation_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

    uint8_t ControlModeCoordinator_ValidateConfig(const control_mode_config_t *config);
    uint8_t ControlModeCoordinator_Init(const control_mode_config_t *config);

    /** Intersect the current mode with the latest Safety-owned source capabilities. */
    uint8_t ControlModeCoordinator_ApplyCapabilityMask(uint8_t capability_mask);

    /** Request an App-owned product mode. MANUAL remains PS2-takeover-only. */
    uint8_t ControlModeCoordinator_Request(control_mode_t mode);

    /** Consume one PS2 observation and perform debounce, takeover, release, or disconnect transitions. */
    control_mode_event_t ControlModeCoordinator_UpdatePs2(const teleoperation_status_t *status, uint32_t now_ms);

    uint32_t ControlModeCoordinator_GetSnapshot(control_mode_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_MODE_COORDINATOR_H */
