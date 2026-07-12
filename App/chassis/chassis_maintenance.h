#ifndef CHASSIS_MAINTENANCE_H
#define CHASSIS_MAINTENANCE_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        CHASSIS_MAINTENANCE_OK = 0,
        CHASSIS_MAINTENANCE_BUSY,
        CHASSIS_MAINTENANCE_NOT_STATIONARY
    } chassis_maintenance_result_t;

    chassis_maintenance_result_t ChassisMaintenance_Begin(void);
    void                         ChassisMaintenance_End(void);

#ifdef __cplusplus
}
#endif

#endif
