#include "bmi270_profile.h"

#define BMI270_ACC_CONF_100HZ_PERF            0xA8U
#define BMI270_ACC_RANGE_2G                   0x00U
#define BMI270_GYR_CONF_100HZ_PERF            0xE8U
#define BMI270_GYR_RANGE_500DPS               0x02U
#define BMI270_PWR_CONF_APS_OFF               0x00U
#define BMI270_PWR_CTRL_ACC_GYR_TEMP_ON       0x0EU
#define BMI270_FIFO_DOWNS_FILTERED            0x88U
#define BMI270_FIFO_CONFIG_TIME_EN            0x02U
#define BMI270_FIFO_CONFIG_HEADER_ACC_GYR     0xD0U
#define BMI270_INT1_ACTIVE_HIGH_PUSH_PULL     0x0AU
#define BMI270_INT_MAP_DATA_INT1_DRDY_WTM_ERR 0x0EU

static const imu_bmi270_profile_t profiles[] = {{BMI270_ACC_CONF_100HZ_PERF,
                                                 BMI270_ACC_RANGE_2G,
                                                 BMI270_GYR_CONF_100HZ_PERF,
                                                 BMI270_GYR_RANGE_500DPS,
                                                 BMI270_PWR_CONF_APS_OFF,
                                                 BMI270_PWR_CTRL_ACC_GYR_TEMP_ON,
                                                 BMI270_FIFO_DOWNS_FILTERED,
                                                 64U,
                                                 0U,
                                                 BMI270_FIFO_CONFIG_TIME_EN,
                                                 BMI270_FIFO_CONFIG_HEADER_ACC_GYR,
                                                 BMI270_INT1_ACTIVE_HIGH_PUSH_PULL,
                                                 BMI270_INT_MAP_DATA_INT1_DRDY_WTM_ERR,
                                                 100U,
                                                 64U},
                                                {BMI270_ACC_CONF_100HZ_PERF,
                                                 BMI270_ACC_RANGE_2G,
                                                 BMI270_GYR_CONF_100HZ_PERF,
                                                 BMI270_GYR_RANGE_500DPS,
                                                 BMI270_PWR_CONF_APS_OFF,
                                                 BMI270_PWR_CTRL_ACC_GYR_TEMP_ON,
                                                 BMI270_FIFO_DOWNS_FILTERED,
                                                 96U,
                                                 0U,
                                                 BMI270_FIFO_CONFIG_TIME_EN,
                                                 BMI270_FIFO_CONFIG_HEADER_ACC_GYR,
                                                 BMI270_INT1_ACTIVE_HIGH_PUSH_PULL,
                                                 BMI270_INT_MAP_DATA_INT1_DRDY_WTM_ERR,
                                                 100U,
                                                 96U},
                                                {BMI270_ACC_CONF_100HZ_PERF,
                                                 BMI270_ACC_RANGE_2G,
                                                 BMI270_GYR_CONF_100HZ_PERF,
                                                 BMI270_GYR_RANGE_500DPS,
                                                 BMI270_PWR_CONF_APS_OFF,
                                                 BMI270_PWR_CTRL_ACC_GYR_TEMP_ON,
                                                 0x00U,
                                                 32U,
                                                 0U,
                                                 BMI270_FIFO_CONFIG_TIME_EN,
                                                 BMI270_FIFO_CONFIG_HEADER_ACC_GYR,
                                                 BMI270_INT1_ACTIVE_HIGH_PUSH_PULL,
                                                 BMI270_INT_MAP_DATA_INT1_DRDY_WTM_ERR,
                                                 100U,
                                                 32U}};

const imu_bmi270_profile_t *ImuBmi270Profile_Get(imu_bmi270_profile_id_t profile)
{
    if ((uint32_t)profile >= (sizeof(profiles) / sizeof(profiles[0])))
    {
        profile = IMU_BMI270_PROFILE_PERFORMANCE;
    }
    return &profiles[(uint32_t)profile];
}

uint8_t ImuBmi270Profile_Check(const imu_bmi270_profile_t *profile, const imu_bmi270_profile_check_t *check)
{
    if (profile == 0 || check == 0)
    {
        return 0U;
    }

    return (profile->acc_conf == check->acc_conf && profile->acc_range == check->acc_range
            && profile->gyr_conf == check->gyr_conf && profile->gyr_range == check->gyr_range
            && profile->pwr_conf == check->pwr_conf && profile->pwr_ctrl == check->pwr_ctrl
            && profile->fifo_config_0 == check->fifo_config_0 && profile->fifo_config_1 == check->fifo_config_1
            && profile->int1_io_ctrl == check->int1_io_ctrl && profile->int_map_data == check->int_map_data)
               ? 1U
               : 0U;
}
