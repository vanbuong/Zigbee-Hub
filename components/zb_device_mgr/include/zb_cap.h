#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* ---- Capability handle ---- */

typedef uint32_t zb_cap_id_t;

#define ZB_CAP_ID(ep, cluster)   ((uint32_t)(ep) << 16 | (uint16_t)(cluster))
#define ZB_CAP_EP(cap)           ((uint8_t)((cap) >> 16))
#define ZB_CAP_CLUSTER(cap)      ((uint16_t)((cap) & 0xFFFF))
#define ZB_CAP_ANY               ((zb_cap_id_t)0xFFFFFFFF)

/* ---- Well-known ZCL cluster IDs ---- */

#define ZCL_CLUSTER_ONOFF        0x0006
#define ZCL_CLUSTER_LEVEL        0x0008
#define ZCL_CLUSTER_COLOR        0x0300
#define ZCL_CLUSTER_TEMPERATURE  0x0402
#define ZCL_CLUSTER_HUMIDITY     0x0405
#define ZCL_CLUSTER_ILLUMINANCE  0x0400

/* ---- Typed event delivered to upper-layer subscribers ---- */

typedef struct {
    uint64_t    ieee_addr;
    zb_cap_id_t cap_id;
    union {
        bool     on_off;                         /* ZCL_CLUSTER_ONOFF  */
        uint8_t  level;                          /* ZCL_CLUSTER_LEVEL  (0-254) */
        struct { uint16_t x; uint16_t y; } color_xy; /* ZCL_CLUSTER_COLOR */
        int16_t  temperature_hundredths;         /* ZCL_CLUSTER_TEMPERATURE (°C×100) */
        uint16_t humidity_hundredths;            /* ZCL_CLUSTER_HUMIDITY   (%×100) */
        uint32_t illuminance_lux;                /* ZCL_CLUSTER_ILLUMINANCE */
        struct {                                 /* unknown cluster fallback */
            uint16_t attr_id;
            uint8_t  data_type;
            uint8_t  raw[8];
        } raw;
    } value;
} zb_cap_event_t;

/* ---- Cluster schema entry ---- */

typedef esp_err_t (*zb_cap_set_fn)(uint64_t ieee, zb_cap_id_t cap,
                                    const void *value, size_t len);
typedef esp_err_t (*zb_cap_get_fn)(uint64_t ieee, zb_cap_id_t cap);

typedef struct {
    uint16_t       cluster_id;
    const char    *name;
    zb_cap_set_fn  set;   /* NULL if read-only */
    zb_cap_get_fn  get;   /* NULL if write-only */
} zb_cluster_schema_t;

/* ---- Init ---- */

esp_err_t zb_cap_init(void);

/* ---- Schema registry ---- */

esp_err_t zb_schema_register(const zb_cluster_schema_t *schema);
const zb_cluster_schema_t *zb_schema_find(uint16_t cluster_id);

/* ---- Generic set / get (dispatches through schema table) ---- */

esp_err_t zb_cap_set(uint64_t ieee, zb_cap_id_t cap,
                     const void *value, size_t len);
esp_err_t zb_cap_get(uint64_t ieee, zb_cap_id_t cap);

/* ---- Typed convenience functions ---- */

esp_err_t zb_cap_onoff_set(uint64_t ieee, zb_cap_id_t cap, bool on);
esp_err_t zb_cap_onoff_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_level_set(uint64_t ieee, zb_cap_id_t cap, uint8_t level);
esp_err_t zb_cap_level_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_color_xy_set(uint64_t ieee, zb_cap_id_t cap,
                               uint16_t x, uint16_t y);
esp_err_t zb_cap_temp_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_humidity_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_illuminance_get(uint64_t ieee, zb_cap_id_t cap);

/* ---- Internal: called by device manager on inbound ZCL attr report ---- */

esp_err_t zb_cap_dispatch_attr(uint64_t ieee, uint8_t ep, uint16_t cluster,
                                uint16_t attr_id, uint8_t dtype,
                                const uint8_t *raw, uint8_t raw_len);
