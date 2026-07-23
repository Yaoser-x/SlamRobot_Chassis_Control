#ifndef LINE_FOLLOWING_SERVICE_H
#define LINE_FOLLOWING_SERVICE_H

#include <stdint.h>

#include "line_following_config.h"
#include "line_following_status.h"

#ifdef __cplusplus
extern "C"
{
#endif

    uint8_t                 LineFollowing_Init(const line_following_config_t *config);
    void                    LineFollowing_Update(void);
    line_following_result_t LineFollowing_Enable(uint8_t enabled);
    uint8_t                 LineFollowing_IsEnabled(void);
    uint32_t                LineFollowing_GetStatus(line_following_status_t *status);

#ifdef __cplusplus
}
#endif

#endif /* LINE_FOLLOWING_SERVICE_H */
