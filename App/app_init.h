#ifndef APP_INIT_H
#define APP_INIT_H

#include <stdint.h>

#include "robot_config.h"

/** @brief Initialize the product with the frozen default aggregate. */
void App_Init(void);

/** @brief Initialize the product from an App-owned validated aggregate. */
uint8_t App_InitWithConfig(const robot_config_t *config);

#endif
