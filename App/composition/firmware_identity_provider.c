#include "firmware_identity_provider.h"

#include "build_identity.h"
#include "communication_protocol_types.h"

#define F407_HARDWARE_REVISION 0x00020000UL

communication_firmware_identity_t FirmwareIdentityProvider_Build(void)
{
    static const uint8_t              git_commit[COMMUNICATION_GIT_COMMIT_LENGTH] = F407_GIT_COMMIT_BYTES;
    communication_firmware_identity_t identity                                    = {
                                           .hardware_revision = F407_HARDWARE_REVISION,
                                           .capabilities      = COMMUNICATION_REQUIRED_CAPABILITIES,
    };

    for (uint8_t i = 0U; i < COMMUNICATION_GIT_COMMIT_LENGTH; ++i)
    {
        identity.git_commit[i] = git_commit[i];
    }
    if (F407_BUILD_IDENTITY_VALID == 0)
    {
        identity.capabilities &= ~COMMUNICATION_CAPABILITY_BUILD_IDENTITY;
    }
    return identity;
}
