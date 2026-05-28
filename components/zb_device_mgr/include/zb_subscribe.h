#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "zb_cap.h"   /* for zb_cap_event_t */

typedef enum {
    ZB_EVENT_NETWORK_READY = 0,  /* coordinator started, network operational */
    ZB_EVENT_NETWORK_LOST,       /* coordinator reset or network lost */
    ZB_EVENT_DEVICE_JOINED,      /* a device joined the network */
    ZB_EVENT_DEVICE_LEFT,        /* a device left the network */
    ZB_EVENT_CAP_REPORT,         /* ZCL attribute report translated to typed cap event */
    ZB_EVENT_ZNP_ERROR,          /* ZNP communication failure */
    ZB_EVENT_TYPE_MAX,
} zb_event_type_t;

typedef struct {
    zb_event_type_t type;
    union {
        struct {
            uint64_t ieee_addr;
            uint16_t nwk_addr;
        } device;
        zb_cap_event_t cap;   /* ZB_EVENT_CAP_REPORT — typed attribute value */
        struct {
            int code;
        } error;
    } data;
} zb_event_t;

typedef void (*zb_event_cb_t)(const zb_event_t *event, void *ctx);

/**
 * Register a callback for an event type.
 * For ZB_EVENT_CAP_REPORT, the callback receives e->data.cap with the typed value.
 * Filter by ieee_addr / cap_id inside the callback if needed.
 */
esp_err_t zb_subscribe(zb_event_type_t event, zb_event_cb_t cb, void *ctx);

/**
 * Deregister a previously registered callback.
 */
esp_err_t zb_unsubscribe(zb_event_type_t event, zb_event_cb_t cb);

/**
 * Internal: post an event to the async delivery queue.
 * Called by the framework and device manager — not for upper-layer use.
 */
void zb_event_emit(const zb_event_t *event);
