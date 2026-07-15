#ifndef TELEMETRY_FRAME_BUILDER_H
#define TELEMETRY_FRAME_BUILDER_H

#include <stdint.h>

#include "communication_types.h"
#include "system_snapshot_service.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** Build one STATUS frame from a coherent system snapshot. */
    uint16_t TelemetryFrameBuilder_BuildStatus(const system_snapshot_t *snapshot,
                                               communication_link_t     link,
                                               uint8_t                 *out,
                                               uint16_t                 out_len);

    /** Build one DIAGNOSTIC frame from a coherent system snapshot. */
    uint16_t TelemetryFrameBuilder_BuildDiagnostic(const system_snapshot_t *snapshot,
                                                   uint32_t                 now_ms,
                                                   uint8_t                 *out,
                                                   uint16_t                 out_len);

    /** Build one IMU frame, or return zero while the IMU is offline. */
    uint16_t
    TelemetryFrameBuilder_BuildImu(const system_snapshot_t *snapshot, uint32_t now_ms, uint8_t *out, uint16_t out_len);

#ifdef __cplusplus
}
#endif

#endif
