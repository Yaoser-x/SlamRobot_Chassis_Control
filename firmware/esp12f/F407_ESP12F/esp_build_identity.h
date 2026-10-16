#ifndef ESP_BUILD_IDENTITY_H
#define ESP_BUILD_IDENTITY_H

#if __has_include("esp_build_identity_generated.h")
#include "esp_build_identity_generated.h"
#else
#define ESP_BUILD_GIT_SHA "unknown"
#define ESP_BUILD_DIRTY 1
#define ESP_BUILD_IDENTITY_VALID 0
#endif

#endif
