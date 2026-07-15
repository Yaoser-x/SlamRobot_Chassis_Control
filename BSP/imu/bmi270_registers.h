#ifndef BMI270_REGISTERS_H
#define BMI270_REGISTERS_H

#define BMI270_REG_CHIP_ID              0x00U
#define BMI270_REG_ERR_REG              0x02U
#define BMI270_REG_DATA_8               0x0CU
#define BMI270_REG_SENSORTIME_0         0x18U
#define BMI270_REG_INTERNAL_STATUS      0x21U
#define BMI270_REG_TEMP_0               0x22U
#define BMI270_REG_FIFO_LENGTH_0        0x24U
#define BMI270_REG_FIFO_DATA            0x26U
#define BMI270_REG_ACC_CONF             0x40U
#define BMI270_REG_ACC_RANGE            0x41U
#define BMI270_REG_GYR_CONF             0x42U
#define BMI270_REG_GYR_RANGE            0x43U
#define BMI270_REG_FIFO_DOWNS           0x45U
#define BMI270_REG_FIFO_WTM_0           0x46U
#define BMI270_REG_FIFO_WTM_1           0x47U
#define BMI270_REG_FIFO_CONFIG_0        0x48U
#define BMI270_REG_FIFO_CONFIG_1        0x49U
#define BMI270_REG_INT1_IO_CTRL         0x53U
#define BMI270_REG_INT_MAP_DATA         0x58U
#define BMI270_REG_INIT_CTRL            0x59U
#define BMI270_REG_INIT_ADDR_0          0x5BU
#define BMI270_REG_INIT_ADDR_1          0x5CU
#define BMI270_REG_INIT_DATA            0x5EU
#define BMI270_REG_PWR_CONF             0x7CU
#define BMI270_REG_PWR_CTRL             0x7DU
#define BMI270_REG_CMD                  0x7EU
#define BMI270_READ_BIT                 0x80U
#define BMI270_CHIP_ID                  0x24U
#define BMI270_CMD_SOFT_RESET           0xB6U
#define BMI270_INTERNAL_STATUS_MSG_MASK 0x0FU
#define BMI270_INTERNAL_STATUS_INIT_OK  0x01U
#define BMI270_INIT_CTRL_PREPARE        0x00U
#define BMI270_INIT_CTRL_COMPLETE       0x01U

#endif
