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

/* Initial set (FR-9.2) */
#define ZCL_CLUSTER_ONOFF         0x0006
#define ZCL_CLUSTER_LEVEL         0x0008
#define ZCL_CLUSTER_COLOR         0x0300
#define ZCL_CLUSTER_TEMPERATURE   0x0402
#define ZCL_CLUSTER_HUMIDITY      0x0405
#define ZCL_CLUSTER_ILLUMINANCE   0x0400

/* Extended ZHA set (FR-12.1) */
#define ZCL_CLUSTER_ANALOG_INPUT  0x000C
#define ZCL_CLUSTER_BINARY_INPUT  0x000F
#define ZCL_CLUSTER_BINARY_OUTPUT 0x0010
#define ZCL_CLUSTER_PRESSURE      0x0403
#define ZCL_CLUSTER_OCCUPANCY     0x0406
#define ZCL_CLUSTER_IAS_ZONE      0x0500
#define ZCL_CLUSTER_METERING      0x0702
#define ZCL_CLUSTER_ELECTRICAL    0x0B04

/* ---- Extended ZHA value types (FR-12.2, 12.4, 12.5) ---- */

typedef struct {
    uint8_t  zone_state;   /* 0 = not enrolled, 1 = enrolled */
    uint16_t zone_type;    /* 0x000D motion, 0x0015 contact, 0x0028 fire/smoke,
                            *   0x002A water, 0x002B CO, 0x002C personal,
                            *   0x0225 vibration */
    uint16_t zone_status;  /* bitmap: bit0=Alarm1, bit1=Alarm2, bit2=Tamper,
                            *         bit3=LowBattery, bit4=SupervisionReports,
                            *         bit5=RestoreReports, bit6=Trouble,
                            *         bit7=AC/mains */
} zb_ias_zone_t;

typedef struct {
    uint64_t summation_delivered;  /* raw uint48 — apply Multiplier/Divisor */
    uint64_t summation_received;   /* raw uint48 */
    int32_t  instantaneous_demand; /* raw int24 */
} zb_metering_t;

typedef struct {
    uint16_t rms_voltage;  /* 0.1 V units */
    uint16_t rms_current;  /* mA */
    int16_t  active_power; /* W */
    int8_t   power_factor; /* signed percent, −100..+100 */
} zb_electrical_t;

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

        int16_t  pressure_hpa;                   /* ZCL_CLUSTER_PRESSURE (hPa) */
        bool     occupancy;                      /* ZCL_CLUSTER_OCCUPANCY */
        zb_ias_zone_t   ias_zone;                /* ZCL_CLUSTER_IAS_ZONE */
        zb_metering_t   metering;                /* ZCL_CLUSTER_METERING */
        zb_electrical_t electrical;              /* ZCL_CLUSTER_ELECTRICAL */
        float    analog_input;                   /* ZCL_CLUSTER_ANALOG_INPUT */
        bool     binary_input;                   /* ZCL_CLUSTER_BINARY_INPUT */
        bool     binary_output;                  /* ZCL_CLUSTER_BINARY_OUTPUT */

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

/* ---- Extended ZHA cluster functions (FR-12) ---- */

esp_err_t zb_cap_pressure_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_occupancy_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_ias_zone_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_metering_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_electrical_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_analog_input_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_binary_input_get(uint64_t ieee, zb_cap_id_t cap);
esp_err_t zb_cap_binary_output_set(uint64_t ieee, zb_cap_id_t cap, bool value);
esp_err_t zb_cap_binary_output_get(uint64_t ieee, zb_cap_id_t cap);

/* ---- Internal: called by device manager on inbound ZCL attr report ---- */

esp_err_t zb_cap_dispatch_attr(uint64_t ieee, uint8_t ep, uint16_t cluster,
                                uint16_t attr_id, uint8_t dtype,
                                const uint8_t *raw, uint8_t raw_len);

/* IAS Zone status-change notification (cluster-specific cmd 0x00, FR-12.3).
 * Called by the device manager when an IAS Zone cluster-specific message
 * arrives on the AF channel. Decodes the bitmap and emits ZB_EVENT_CAP_REPORT
 * with value.ias_zone.zone_status populated.
 */
esp_err_t zb_cap_dispatch_ias_status_change(uint64_t ieee, uint8_t ep,
                                              const uint8_t *cmd_payload,
                                              uint8_t cmd_len);
