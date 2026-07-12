#ifndef ESP_LINK_POLICY_H
#define ESP_LINK_POLICY_H

#include <stdint.h>

#define ESP_LINK_STATUS_TIMEOUT_MS 500UL

typedef struct
{
  uint8_t status_valid;
  uint8_t online;
  uint8_t neutral_sent;
  uint32_t last_status_ms;
} esp_link_policy_t;

static inline void EspLinkPolicy_OnStatus(esp_link_policy_t *policy, uint32_t now_ms)
{
  policy->status_valid = 1U;
  policy->online = 1U;
  policy->neutral_sent = 0U;
  policy->last_status_ms = now_ms;
}

static inline uint32_t EspLinkPolicy_StatusAge(const esp_link_policy_t *policy, uint32_t now_ms)
{
  return policy->status_valid ? (uint32_t)(now_ms - policy->last_status_ms) : UINT32_MAX;
}

static inline uint8_t EspLinkPolicy_Update(esp_link_policy_t *policy, uint32_t now_ms)
{
  if (policy->status_valid != 0U &&
      EspLinkPolicy_StatusAge(policy, now_ms) > ESP_LINK_STATUS_TIMEOUT_MS)
  {
    policy->online = 0U;
    if (policy->neutral_sent == 0U)
    {
      policy->neutral_sent = 1U;
      return 1U;
    }
  }
  return 0U;
}

#endif
