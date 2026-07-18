#include "system_publish_snapshot_service.h"

#include "platform_critical.h"
#include "system_monitoring_service.h"

static communication_publish_model_t publish_models[2];
static uint8_t                       active_model;
static uint32_t                      publish_generation;

void SystemPublishSnapshot_Init(void)
{
    platform_critical_state_t state;

    state              = PlatformCritical_Enter();
    publish_models[0]  = (communication_publish_model_t){0};
    publish_models[1]  = (communication_publish_model_t){0};
    active_model       = 0U;
    publish_generation = 0U;
    PlatformCritical_Exit(state);
}

void SystemPublishSnapshot_Publish(const communication_publish_model_t *snapshot)
{
    system_monitoring_module_health_t modules;
    platform_critical_state_t         critical;
    uint8_t                           inactive;

    if (snapshot == 0)
    {
        return;
    }
    modules = (system_monitoring_module_health_t){
        .imu_online     = snapshot->modules.imu_online,
        .encoder_online = snapshot->modules.encoder_online,
        .motor_online   = snapshot->modules.motor_online,
        .adc_online     = snapshot->modules.adc_online,
        .host_online    = snapshot->modules.upper_online,
        .esp12f_online  = snapshot->modules.esp12f_online,
        .line_online    = snapshot->modules.line_online,
        .ps2_online     = snapshot->modules.ps2_online,
    };
    SystemMonitoring_SetModuleHealth(&modules);

    inactive                 = (uint8_t)(active_model ^ 1U);
    publish_models[inactive] = *snapshot;
    critical                 = PlatformCritical_Enter();
    publish_generation++;
    publish_models[inactive].generation = publish_generation;
    active_model                        = inactive;
    PlatformCritical_Exit(critical);
}

uint32_t SystemPublishSnapshot_Get(communication_publish_model_t *out)
{
    platform_critical_state_t state;
    uint32_t                  generation;

    if (out == 0)
    {
        return 0U;
    }
    state      = PlatformCritical_Enter();
    *out       = publish_models[active_model];
    generation = out->generation;
    PlatformCritical_Exit(state);
    return generation;
}
