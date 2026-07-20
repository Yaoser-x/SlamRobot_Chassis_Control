#ifndef TELEMETRY_FRAME_BUILDER_H
#define TELEMETRY_FRAME_BUILDER_H

#include <stdint.h>

#include "communication_types.h"
#include "communication_publish_model_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /** Build one STATUS frame from a coherent system snapshot. */
    uint16_t TelemetryEncoder_BuildStatus(const communication_publish_model_t *snapshot,
                                          communication_link_t                 link,
                                          uint8_t                             *out,
                                          uint16_t                             out_len);

    /** Build one HELLO frame from injected immutable identity and snapshot parameter CRC. */
    uint16_t TelemetryEncoder_BuildHello(const communication_firmware_identity_t *identity,
                                         uint32_t                                 parameter_crc32,
                                         uint8_t                                 *out,
                                         uint16_t                                 out_len);

    /** Build one DIAGNOSTIC frame from a coherent system snapshot. */
    uint16_t
    TelemetryEncoder_BuildDiagnostic(const communication_publish_model_t *snapshot, uint8_t *out, uint16_t out_len);

    /** Build one IMU frame, or return zero while the IMU is offline. */
    uint16_t TelemetryEncoder_BuildImu(const communication_publish_model_t *snapshot, uint8_t *out, uint16_t out_len);

#ifdef __cplusplus
}
#endif

#endif
