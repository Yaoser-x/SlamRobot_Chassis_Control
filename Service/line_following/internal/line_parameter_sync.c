#include "line_parameter_sync.h"

#include "line_sensor_driver.h"
#include "parameter_management_service.h"

void LineParameterSync_Init(line_parameter_sync_t *sync)
{
    if (sync != 0)
    {
        sync->applied_generation = 0UL;
    }
}

uint8_t LineParameterSync_Update(line_parameter_sync_t *sync)
{
    param_model_t params;
    uint32_t      generation;

    if (sync == 0)
    {
        return 0U;
    }
    generation = ParameterManagement_GetSnapshot(&params);
    if (generation == 0UL || generation == sync->applied_generation)
    {
        return 0U;
    }
    LineSensorDriver_SetThresholdConfig(params.line_threshold_raw, params.line_active_low);
    sync->applied_generation = generation;
    return 1U;
}
