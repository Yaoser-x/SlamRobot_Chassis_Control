#ifndef CHASSIS_MAINTENANCE_SERVICE_H
#define CHASSIS_MAINTENANCE_SERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        CHASSIS_MAINTENANCE_SERVICE_OK = 0,
        CHASSIS_MAINTENANCE_SERVICE_BUSY,
        CHASSIS_MAINTENANCE_SERVICE_NOT_STATIONARY
    } chassis_maintenance_service_result_t;

    chassis_maintenance_service_result_t ChassisMaintenanceService_Begin(void);
    void                                 ChassisMaintenanceService_End(void);

#ifdef __cplusplus
}
#endif

#endif
