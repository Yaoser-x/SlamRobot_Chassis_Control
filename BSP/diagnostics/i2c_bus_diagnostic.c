#include "i2c_bus_diagnostic.h"

#include "i2c.h"

uint8_t I2cBusDiagnostic_Probe(uint8_t address_7bit)
{
    return (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(address_7bit << 1), 1U, 5U) == HAL_OK) ? 1U : 0U;
}
