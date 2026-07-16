#ifndef COMMUNICATION_PROTOCOL_INFO_H
#define COMMUNICATION_PROTOCOL_INFO_H

/** Upper-link protocol version exposed for diagnostics. */
#define COMMUNICATION_PROTOCOL_VERSION          2U
/** Diagnostic payload schema version exposed for diagnostics. */
#define COMMUNICATION_DIAGNOSTIC_SCHEMA_VERSION 1U
/** Fixed status payload length in bytes. */
#define COMMUNICATION_STATUS_PAYLOAD_LENGTH     65U
/** Fixed diagnostic payload length in bytes. */
#define COMMUNICATION_DIAGNOSTIC_PAYLOAD_LENGTH 28U
/** Fixed IMU status payload length and maximum payload length in bytes. */
#define COMMUNICATION_IMU_STATUS_PAYLOAD_LENGTH 99U
#define COMMUNICATION_MAX_PAYLOAD_LENGTH        99U

#endif
