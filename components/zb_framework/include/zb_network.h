#pragma once

/*
 * Internal network management helpers — not part of the public framework API.
 * Used exclusively by zb_framework.c (state machine handlers).
 */

#include <stdint.h>
#include "esp_err.h"

/**
 * Send SYS_PING SREQ and verify a valid SRSP is received.
 */
esp_err_t zb_net_ping(void);

/**
 * Open or close the network for joining via ZDO_MGMT_PERMIT_JOIN_REQ (broadcast).
 * @param duration_s  0 = close, 0xFF = open indefinitely, 1-254 = seconds.
 */
esp_err_t zb_net_permit_join(uint8_t duration_s);

/**
 * Register an AF endpoint on the coordinator (AF_REGISTER SREQ).
 * Must be called during ZNP_INIT so the coordinator can receive
 * AF_INCOMING_MSG AREQs from joined devices.
 * @param ep  Endpoint number (use 1 for the default coordinator endpoint).
 */
esp_err_t zb_net_af_register(uint8_t ep);

/**
 * Send SYS_VERSION SREQ and copy the version string into out[len].
 */
esp_err_t zb_net_get_version(char *out, size_t len);

/**
 * Read a Z-Stack NV item from the CC2652P7 via OsalNvRead SREQ.
 * @param id    ZCD_NV_* item ID.
 * @param buf   Output buffer.
 * @param len   Requested byte count.
 */
esp_err_t zb_net_read_nv(uint16_t id, uint8_t *buf, uint8_t len);

/**
 * Write a Z-Stack NV item to the CC2652P7 via OsalNvWrite SREQ.
 * @param id    ZCD_NV_* item ID.
 * @param buf   Data to write.
 * @param len   Byte count.
 */
esp_err_t zb_net_write_nv(uint16_t id, const uint8_t *buf, uint8_t len);

/**
 * Configure coordinator parameters (logical type, PAN ID, channel, key) in
 * CC2652P7 NV, then call ZDO_STARTUP_FROM_APP.
 * @param pan_id   16-bit PAN ID.
 * @param channel  Zigbee channel (11-26).
 * @param nwk_key  16-byte AES-128 network key.
 */
esp_err_t zb_net_form_network(uint16_t pan_id, uint8_t channel,
                               const uint8_t nwk_key[16]);

/**
 * Send ZDO_STARTUP_FROM_APP SREQ and await a ZDO_STATE_CHANGE_IND AREQ
 * with DEV_ZB_COORD (0x09) within timeout_ms.
 */
esp_err_t zb_net_startup_and_wait(uint32_t timeout_ms);

/**
 * Read PAN ID and channel from CC2652P7 NV and check against desired values.
 * @return ESP_OK    if NV is empty (first boot).
 *         ESP_ERR_NOT_FOUND  if NV params differ from desired (needs reconfigure).
 *         ESP_ERR_NOT_FINISHED  if NV params match (can resume directly).
 */
esp_err_t zb_net_check_nv_state(uint16_t desired_pan_id, uint8_t desired_channel);
