#ifndef BMI270_BUS_H
#define BMI270_BUS_H

#include <stdint.h>

typedef struct
{
    uint8_t hal_status[2];
    uint8_t hal_rx[2][3];
    uint8_t bitbang_rx[3];
    uint8_t miso_nopull;
    uint8_t miso_pullup;
    uint8_t miso_pulldown;
} imu_bmi270_diag_t;

/** Drive BMI270 chip select inactive before initialization or recovery. */
void Bmi270Bus_Deselect(void);

/** Read one BMI270 register through the normal SPI bus. */
uint8_t Bmi270Bus_ReadReg(uint8_t reg, uint8_t *value);

/** Read a bounded BMI270 register burst through the normal SPI bus. */
uint8_t Bmi270Bus_ReadBytes(uint8_t reg, uint8_t *data, uint8_t len);

/** Write one BMI270 register through the normal SPI bus. */
uint8_t Bmi270Bus_WriteReg(uint8_t reg, uint8_t value);

/** Write a bounded BMI270 register burst through the normal SPI bus. */
uint8_t Bmi270Bus_WriteBytes(uint8_t reg, const uint8_t *data, uint8_t len);

/** Run the destructive SPI/bit-bang recovery probe used by maintenance diagnostics. */
uint8_t Bmi270Bus_RunRecoveryProbe(uint8_t chip_id_reg, imu_bmi270_diag_t *diag);

#endif
