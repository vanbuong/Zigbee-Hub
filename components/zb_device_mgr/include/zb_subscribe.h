#pragma once

#include <stdint.h>
#include "esp_err.h"

/* Network-level event types emitted by the framework and device manager.
 * Device-capability attribute events use zb_cap_subscribe() instead. */
typedef enum {
    ZB_EVENT_NETWORK_READY = 0,  /* coordinator started, network operational */
    ZB_EVENT_NETWORK_LOST,       /* coordinator reset or network lost */
    ZB_EVENT_DEVICE_JOINED,      /* a device joined the network */
    ZB_EVENT_DEVICE_LEFT,        /* a device left the network */
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
        struct {
            int code;
        } error;
    } data;
} zb_event_t;

typedef void (*zb_event_cb_t)(const zb_event_t *event, void *ctx);

/**
 * Register a callback for a specific network-level event type.
 * Multiple callbacks per event type are supported.
 */
esp_err_t zb_subscribe(zb_event_type_t event, zb_event_cb_t cb, void *ctx);

/**
 * Deregister a previously registered callback.
 */
esp_err_t zb_unsubscribe(zb_event_type_t event, zb_event_cb_t cb);

/**
 * Internal: emit an event to all registered subscribers.
 * Called by the framework and device manager — not for upper-layer use.
 */
void zb_event_emit(const zb_event_t *event);
