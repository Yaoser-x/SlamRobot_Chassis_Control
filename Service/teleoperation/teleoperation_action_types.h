#ifndef TELEOPERATION_ACTION_TYPES_H
#define TELEOPERATION_ACTION_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        TELEOPERATION_ACTION_NONE = 0,
        TELEOPERATION_ACTION_TOGGLE_LINE,
        TELEOPERATION_ACTION_CALIBRATE_LINE_FLOOR,
        TELEOPERATION_ACTION_CALIBRATE_LINE_SURFACE
    } teleoperation_action_type_t;

    typedef struct
    {
        teleoperation_action_type_t type;
        uint32_t                    timestamp_ms;
        uint32_t                    input_generation;
        uint8_t                     line_tracking_enabled;
    } teleoperation_action_t;

#ifdef __cplusplus
}
#endif

#endif /* TELEOPERATION_ACTION_TYPES_H */