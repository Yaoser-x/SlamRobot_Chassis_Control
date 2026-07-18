#include "chassis_maintenance_service.h"

#include "motion_control_service.h"
#include "motion_control_maintenance.h"

chassis_maintenance_service_result_t ChassisMaintenanceService_Begin(void)
{
    return (chassis_maintenance_service_result_t)MotionControl_BeginMaintenance();
}

void ChassisMaintenanceService_End(void)
{
    MotionControl_EndMaintenance();
}
