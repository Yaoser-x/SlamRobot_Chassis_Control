#ifndef CHASSIS_SNAPSHOT_H
#define CHASSIS_SNAPSHOT_H

#include "chassis_service.h"
#include "chassis_target_planner.h"

/** Expand left/right targets to enabled motor snapshot fields. */
void ChassisSnapshot_SetSideTargets(chassis_service_snapshot_t *snapshot,
                                    float                       left_mps,
                                    float                       right_mps,
                                    uint8_t                     requested);

/** Aggregate per-motor snapshot fields into left/right side fields. */
void ChassisSnapshot_SyncSides(chassis_service_snapshot_t *snapshot);

/** Publish target-planner diagnostics into the public chassis snapshot. */
void ChassisSnapshot_ApplyPlannerResult(chassis_service_snapshot_t            *snapshot,
                                        const chassis_target_planner_result_t *result,
                                        uint8_t                                control_source);

#endif
