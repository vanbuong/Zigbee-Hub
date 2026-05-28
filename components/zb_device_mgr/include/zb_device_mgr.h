#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "zb_cap.h"
#include "znp_types.h"

#define ZB_DEV_MAX_DEVICES    100
#define ZB_DEV_MAX_CAPS       64

/* Runtime device record (in-memory, not the protobuf schema) */
typedef struct {
    bool        active;
    uint64_t    ieee_addr;
    uint16_t    nwk_addr;
    uint8_t     num_caps;
    zb_cap_id_t caps[ZB_DEV_MAX_CAPS];
} zb_dev_t;

/* Initialize device manager; registers built-in cluster schemas */
esp_err_t zb_dev_mgr_init(void);

/* Called by framework on device join (TC_DEV_IND / END_DEVICE_ANNCE) */
esp_err_t zb_dev_mgr_join(uint64_t ieee_addr, uint16_t nwk_addr);

/* Called by framework on device leave */
void zb_dev_mgr_leave(uint64_t ieee_addr);

/* Lookup device by IEEE address; returns NULL if not found */
const zb_dev_t *zb_dev_mgr_get(uint64_t ieee_addr);

/* Lookup NWK address by IEEE (0xFFFF = not found) */
uint16_t zb_dev_mgr_get_nwk(uint64_t ieee_addr);

/* Add a capability (cap_id) to a device; called on first attr report from that cluster */
esp_err_t zb_dev_mgr_learn_cap(uint64_t ieee_addr, zb_cap_id_t cap);

/* Enumerate capabilities for a device */
esp_err_t zb_dev_get_caps(uint64_t ieee_addr, zb_cap_id_t *list, uint8_t *count);

/* Main AREQ dispatcher — called from framework handle_ready() for every AREQ */
void zb_dev_mgr_on_areq(const znp_frame_t *areq);

/* Send a ZCL command frame via AF_DATA_REQUEST */
esp_err_t zb_zcl_send(uint64_t ieee, zb_cap_id_t cap,
                       const uint8_t *zcl_frame, uint8_t zcl_len);
