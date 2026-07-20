#ifndef FIRMWARE_IDENTITY_PROVIDER_H
#define FIRMWARE_IDENTITY_PROVIDER_H

#include "communication_types.h"

/** Build the immutable communication identity owned by App composition. */
communication_firmware_identity_t FirmwareIdentityProvider_Build(void);

#endif
