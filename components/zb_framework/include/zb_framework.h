#pragma once

#include "esp_err.h"
#include "zb_types.h"

/**
 * Initialize the Zigbee framework with the given hardware and network config.
 * Must be called before zb_framework_start(). NVS must already be initialized
 * by the caller (nvs_flash_init()).
 */
esp_err_t zb_framework_init(const zb_config_t *cfg);

/**
 * Start the framework task. The state machine will begin transitioning from
 * UNINITIALIZED toward READY asynchronously.
 */
esp_err_t zb_framework_start(void);

/**
 * Query the current framework state. Thread-safe.
 */
zb_state_t zb_framework_get_state(void);

/**
 * Retrieve firmware version strings for both the ESP32 and CC2652P7.
 * cc26xx_fw is only valid after the state machine has passed ZNP_INIT.
 */
esp_err_t zb_framework_get_versions(zb_versions_t *out);
