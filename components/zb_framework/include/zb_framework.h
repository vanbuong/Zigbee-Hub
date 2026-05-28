#pragma once

#include "esp_err.h"
#include "zb_types.h"
#include "zb_storage.h"   /* zb_net_config_t — needed by zb_network_param_set */

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

/**
 * Return a snapshot of the current network state (FR-11.1).
 * Safe to call from any task at any framework state — no ZNP round-trip.
 * Fields that are not yet meaningful are zeroed.
 */
esp_err_t zb_network_info_get(zb_network_info_t *out);

/**
 * Persist updated network parameters to /zb/config.json (FR-11.4).
 * If the framework is currently in READY state and PAN ID or channel differs
 * from the active network, a RECONFIGURING transition is triggered.
 */
esp_err_t zb_network_param_set(const zb_net_config_t *cfg);
