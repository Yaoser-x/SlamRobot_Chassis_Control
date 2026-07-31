#ifndef CONTROL_MODE_TYPES_H
#define CONTROL_MODE_TYPES_H

#include <stdint.h>

typedef enum
{
    CONTROL_MODE_DISABLED = 0,
    CONTROL_MODE_MANUAL,
    CONTROL_MODE_AUTO,
    CONTROL_MODE_LINE,
    CONTROL_MODE_MAINTENANCE,
    CONTROL_MODE_COUNT
} control_mode_t;

typedef struct
{
    control_mode_t mode;
    control_mode_t recovery_mode;
    uint32_t       generation;
    uint32_t       manual_neutral_since_ms;
    uint8_t        takeover_active;
} control_mode_snapshot_t;

typedef enum
{
    CONTROL_MODE_EVENT_NONE = 0,
    CONTROL_MODE_EVENT_ENTERED_MANUAL,
    CONTROL_MODE_EVENT_RESTORED_AUTO,
    CONTROL_MODE_EVENT_RESTORED_LINE,
    CONTROL_MODE_EVENT_PS2_DISCONNECTED
} control_mode_event_t;

#endif /* CONTROL_MODE_TYPES_H */
