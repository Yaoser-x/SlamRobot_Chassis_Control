#include "system_publish_snapshot_service.h"

#include "platform_critical.h"
#include "system_monitoring_service.h"

static communication_publish_model_t        publish_models[2];
static uint8_t                              active_model;
static uint32_t                             publish_generation;
static communication_publish_model_config_t publish_config;
static system_publish_snapshot_provider_t   snapshot_provider;

uint8_t SystemPublishSnapshot_Init(const communication_publish_model_config_t *config,
                                       system_publish_snapshot_provider_t          provider)
{
    platform_critical_state_t state;

    if (config == 0 || provider == 0 || config->host_timeout_ms == 0U || config->esp12f_timeout_ms == 0U
        || config->line_timeout_ms == 0U)
    {
        return 0U;
    }
    state              = PlatformCritical_Enter();
    publish_config     = *config;
    snapshot_provider  = provider;
    publish_models[0]  = (communication_publish_model_t){0};
    publish_models[1]  = (communication_publish_model_t){0};
    active_model       = 0U;
    publish_generation = 0U;
    PlatformCritical_Exit(state);
    return 1U;
}

void SystemPublishSnapshot_Update(uint32_t now_ms)
{
    communication_publish_model_t     next = {0};
    system_monitoring_module_health_t modules;
    platform_critical_state_t         critical;
    uint8_t                           inactive;

    if (snapshot_provider == 0)
    {
        return;
    }
    snapshot_provider(now_ms, &publish_config, &next);
    modules = (system_monitoring_module_health_t){
        .imu_online     = next.modules.imu_online,
        .encoder_online = next.modules.encoder_online,
        .motor_online   = next.modules.motor_online,
        .adc_online     = next.modules.adc_online,
        .host_online    = next.modules.upper_online,
        .esp12f_online  = next.modules.esp12f_online,
        .line_online    = next.modules.line_online,
        .ps2_online     = next.modules.ps2_online,
    };
    SystemMonitoring_SetModuleHealth(&modules);

    inactive                 = (uint8_t)(active_model ^ 1U);
    publish_models[inactive] = next;
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
