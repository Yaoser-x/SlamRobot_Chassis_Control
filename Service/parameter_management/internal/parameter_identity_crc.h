#ifndef PARAMETER_IDENTITY_CRC_H
#define PARAMETER_IDENTITY_CRC_H

#include "parameter_management_types.h"

#include <stdint.h>

/** Calculate CRC32/ISO-HDLC over the canonical little-endian parameter model. */
uint32_t ParameterIdentityCrc_Calculate(const param_model_t *params);

#endif
