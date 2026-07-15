#ifndef I2C_BUS_DIAGNOSTIC_H
#define I2C_BUS_DIAGNOSTIC_H

#include <stdint.h>

/** Return nonzero when an I2C1 device acknowledges a 7-bit address. */
uint8_t I2cBusDiagnostic_Probe(uint8_t address_7bit);

#endif
