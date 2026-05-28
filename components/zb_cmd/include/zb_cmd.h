#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    ZB_CMD_PERMIT_JOIN = 0,
    ZB_CMD_CHANGE_CHANNEL,
    ZB_CMD_RECONFIGURE,   /* trigger RECONFIGURING (no params); used by zb_network_param_set */
} zb_cmd_type_t;

typedef struct {
    zb_cmd_type_t type;
    union {
        uint8_t duration_s;   /* PERMIT_JOIN: seconds; 0=close, 0xFF=indefinite */
        uint8_t channel;      /* CHANGE_CHANNEL: new 2.4 GHz channel (11-26) */
    } params;
} zb_cmd_t;

/**
 * Initialize the command queue.  Must be called before any zb_cmd_* function.
 */
esp_err_t zb_cmd_init(void);

/**
 * Open (or close) the network for device joins.
 * Enqueued and executed when the framework reaches READY state.
 * @param duration_s  0 = close, 0xFF = open indefinitely, 1-254 = seconds.
 */
esp_err_t zb_cmd_permit_join(uint8_t duration_s);

/**
 * Request a coordinator channel change.
 * Triggers RECONFIGURING: saves new channel to config.json and
 * re-forms the network on the new channel.
 * @param channel  2.4 GHz channel number (11-26).
 */
esp_err_t zb_cmd_change_channel(uint8_t channel);

/**
 * Trigger a full network reconfigure (clears NV, re-forms).
 * Enqueues ZB_CMD_RECONFIGURE; the framework processes it on next READY iteration.
 * Intended for use by zb_network_param_set() when params change while READY.
 */
esp_err_t zb_cmd_reconfigure(void);

/**
 * Internal: dequeue the next pending command (used by framework_task).
 * Returns pdTRUE if a command was dequeued, pdFALSE on timeout.
 */
BaseType_t zb_cmd_dequeue(zb_cmd_t *cmd, TickType_t ticks_to_wait);
