#ifndef MOTION_MAINTENANCE_ORCHESTRATOR_H
#define MOTION_MAINTENANCE_ORCHESTRATOR_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        APP_MOTION_MAINTENANCE_OK = 0,
        APP_MOTION_MAINTENANCE_BUSY,
        APP_MOTION_MAINTENANCE_NOT_STATIONARY
    } app_motion_maintenance_result_t;

    /** Acquire the App-owned maintenance workflow and prove every enabled wheel is stationary. */
    app_motion_maintenance_result_t AppMotionMaintenance_Begin(void);

    /** Restore the previous product mode while the Safety gate is still closed, then release maintenance. */
    void AppMotionMaintenance_End(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_MAINTENANCE_ORCHESTRATOR_H */
