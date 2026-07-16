#ifndef CHASSIS_SNAPSHOT_H
#define CHASSIS_SNAPSHOT_H

#include "wheel_target_planner.h"
#include "motion_control_status.h"

/** Expand left/right targets to enabled motor snapshot fields. */
void MotionStatusBuilder_SetSideTargets(motion_control_status_t *snapshot,
                                        float                    left_mps,
                                        float                    right_mps,
                                        uint8_t                  requested);

/** Aggregate per-motor snapshot fields into left/right side fields. */
void MotionStatusBuilder_SyncSides(motion_control_status_t *snapshot);

/** Publish target-planner diagnostics into the public chassis snapshot. */
void MotionStatusBuilder_ApplyPlannerResult(motion_control_status_t             *snapshot,
                                            const wheel_target_planner_result_t *result,
                                            uint8_t                              control_source);

#endif
