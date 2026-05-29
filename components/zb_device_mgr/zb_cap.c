#include "zb_cap.h"
#include "zb_subscribe.h"
#include "zb_device_mgr.h"

#include <string.h>
#include "esp_log.h"

static const char *TAG = "zb_cap";

/* ---- Schema registry ---- */

#define MAX_SCHEMAS 32

static const zb_cluster_schema_t *s_schemas[MAX_SCHEMAS];
static uint8_t s_schema_count = 0;

/* Initial set (FR-9.2) */
extern const zb_cluster_schema_t zb_schema_onoff;
extern const zb_cluster_schema_t zb_schema_level;
extern const zb_cluster_schema_t zb_schema_color;
extern const zb_cluster_schema_t zb_schema_temperature;
extern const zb_cluster_schema_t zb_schema_humidity;
extern const zb_cluster_schema_t zb_schema_illuminance;
/* Extended ZHA set (FR-12.1) */
extern const zb_cluster_schema_t zb_schema_pressure;
extern const zb_cluster_schema_t zb_schema_occupancy;
extern const zb_cluster_schema_t zb_schema_ias_zone;
extern const zb_cluster_schema_t zb_schema_metering;
extern const zb_cluster_schema_t zb_schema_electrical;
extern const zb_cluster_schema_t zb_schema_analog_input;
extern const zb_cluster_schema_t zb_schema_binary_input;
extern const zb_cluster_schema_t zb_schema_binary_output;

/* ---- ZCL translation helpers ---- */

static void translate_to_typed(uint16_t cluster, uint16_t attr_id,
                                uint8_t dtype, const uint8_t *v, uint8_t vlen,
                                zb_cap_event_t *evt)
{
    switch (cluster) {
    case ZCL_CLUSTER_ONOFF:
        evt->value.on_off = (v[0] != 0);
        break;
    case ZCL_CLUSTER_LEVEL:
        evt->value.level = v[0];
        break;
    case ZCL_CLUSTER_COLOR:
        if (attr_id == 0x0003 && vlen >= 2)
            evt->value.color_xy.x = (uint16_t)(v[0] | (v[1] << 8));
        else if (attr_id == 0x0004 && vlen >= 2)
            evt->value.color_xy.y = (uint16_t)(v[0] | (v[1] << 8));
        break;
    case ZCL_CLUSTER_TEMPERATURE:
        if (vlen >= 2)
            evt->value.temperature_hundredths =
                (int16_t)((uint16_t)(v[0] | (v[1] << 8)));
        break;
    case ZCL_CLUSTER_HUMIDITY:
        if (vlen >= 2)
            evt->value.humidity_hundredths = (uint16_t)(v[0] | (v[1] << 8));
        break;
    case ZCL_CLUSTER_ILLUMINANCE:
        if (vlen >= 2)
            evt->value.illuminance_lux = (uint16_t)(v[0] | (v[1] << 8));
        break;

    case ZCL_CLUSTER_PRESSURE:
        /* MeasuredValue (0x0000) is int16 in hPa */
        if (attr_id == 0x0000 && vlen >= 2)
            evt->value.pressure_hpa = (int16_t)(v[0] | (v[1] << 8));
        break;

    case ZCL_CLUSTER_OCCUPANCY:
        /* Occupancy (0x0000) is a single-byte bitmap; bit 0 = occupied */
        if (attr_id == 0x0000 && vlen >= 1)
            evt->value.occupancy = (v[0] & 0x01) != 0;
        break;

    case ZCL_CLUSTER_IAS_ZONE:
        if (vlen < 1) break;
        if (attr_id == 0x0000) {           /* ZoneState   uint8 */
            evt->value.ias_zone.zone_state = v[0];
        } else if (attr_id == 0x0001 && vlen >= 2) {   /* ZoneType    uint16 */
            evt->value.ias_zone.zone_type = (uint16_t)(v[0] | (v[1] << 8));
        } else if (attr_id == 0x0002 && vlen >= 2) {   /* ZoneStatus  bitmap16 */
            evt->value.ias_zone.zone_status = (uint16_t)(v[0] | (v[1] << 8));
        }
        break;

    case ZCL_CLUSTER_METERING:
        if (attr_id == 0x0000 && vlen >= 6) {          /* CurrentSummationDelivered uint48 */
            uint64_t s = 0;
            for (int i = 0; i < 6; i++) s |= (uint64_t)v[i] << (8 * i);
            evt->value.metering.summation_delivered = s;
        } else if (attr_id == 0x0001 && vlen >= 6) {   /* CurrentSummationReceived uint48 */
            uint64_t s = 0;
            for (int i = 0; i < 6; i++) s |= (uint64_t)v[i] << (8 * i);
            evt->value.metering.summation_received = s;
        } else if (attr_id == 0x0400 && vlen >= 3) {   /* InstantaneousDemand int24 */
            int32_t s = (int32_t)((uint32_t)v[0]
                                  | ((uint32_t)v[1] << 8)
                                  | ((uint32_t)v[2] << 16));
            if (s & 0x00800000) s |= 0xFF000000;        /* sign-extend */
            evt->value.metering.instantaneous_demand = s;
        }
        break;

    case ZCL_CLUSTER_ELECTRICAL:
        if (vlen < 2) break;
        if (attr_id == 0x0505) {                       /* RMSVoltage  uint16 (0.1 V) */
            evt->value.electrical.rms_voltage = (uint16_t)(v[0] | (v[1] << 8));
        } else if (attr_id == 0x0508) {                /* RMSCurrent  uint16 (mA)   */
            evt->value.electrical.rms_current = (uint16_t)(v[0] | (v[1] << 8));
        } else if (attr_id == 0x050B) {                /* ActivePower int16  (W)   */
            evt->value.electrical.active_power = (int16_t)(v[0] | (v[1] << 8));
        } else if (attr_id == 0x0510 && vlen >= 1) {   /* PowerFactor int8 */
            evt->value.electrical.power_factor = (int8_t)v[0];
        }
        break;

    case ZCL_CLUSTER_ANALOG_INPUT:
        /* PresentValue (0x0055) is float32 (single-precision IEEE-754) */
        if (attr_id == 0x0055 && vlen >= 4) {
            uint32_t bits = (uint32_t)v[0]
                          | ((uint32_t)v[1] << 8)
                          | ((uint32_t)v[2] << 16)
                          | ((uint32_t)v[3] << 24);
            memcpy(&evt->value.analog_input, &bits, sizeof(float));
        }
        break;

    case ZCL_CLUSTER_BINARY_INPUT:
        if (attr_id == 0x0055 && vlen >= 1)
            evt->value.binary_input = (v[0] != 0);
        break;

    case ZCL_CLUSTER_BINARY_OUTPUT:
        if (attr_id == 0x0055 && vlen >= 1)
            evt->value.binary_output = (v[0] != 0);
        break;

    default:
        evt->value.raw.attr_id   = attr_id;
        evt->value.raw.data_type = dtype;
        memcpy(evt->value.raw.raw, v, vlen < 8 ? vlen : 8);
        break;
    }
}

/* ---- Public API ---- */

esp_err_t zb_cap_init(void)
{
    zb_schema_register(&zb_schema_onoff);
    zb_schema_register(&zb_schema_level);
    zb_schema_register(&zb_schema_color);
    zb_schema_register(&zb_schema_temperature);
    zb_schema_register(&zb_schema_humidity);
    zb_schema_register(&zb_schema_illuminance);

    /* Extended ZHA set (FR-12.1, 12.8) — registered via the same mechanism */
    zb_schema_register(&zb_schema_pressure);
    zb_schema_register(&zb_schema_occupancy);
    zb_schema_register(&zb_schema_ias_zone);
    zb_schema_register(&zb_schema_metering);
    zb_schema_register(&zb_schema_electrical);
    zb_schema_register(&zb_schema_analog_input);
    zb_schema_register(&zb_schema_binary_input);
    zb_schema_register(&zb_schema_binary_output);

    ESP_LOGI(TAG, "initialized with %d built-in schemas", s_schema_count);
    return ESP_OK;
}

esp_err_t zb_cap_dispatch_ias_status_change(uint64_t ieee, uint8_t ep,
                                              const uint8_t *cmd_payload,
                                              uint8_t cmd_len)
{
    /* IAS Zone Status Change Notification payload (ZCL):
     *   ZoneStatus[2], ExtendedStatus[1], ZoneID[1], Delay[2]
     * The first 2 bytes carry the same bitmap as attribute 0x0002. */
    if (cmd_len < 2) return ESP_ERR_INVALID_SIZE;

    zb_cap_event_t evt = {
        .ieee_addr = ieee,
        .cap_id    = ZB_CAP_ID(ep, ZCL_CLUSTER_IAS_ZONE),
    };
    evt.value.ias_zone.zone_status =
        (uint16_t)(cmd_payload[0] | (cmd_payload[1] << 8));

    ESP_LOGD(TAG, "IAS status change ieee=%016llx ep=%d status=0x%04x",
             (unsigned long long)ieee, ep, evt.value.ias_zone.zone_status);

    zb_event_emit(&(zb_event_t){
        .type     = ZB_EVENT_CAP_REPORT,
        .data.cap = evt,
    });
    return ESP_OK;
}

esp_err_t zb_schema_register(const zb_cluster_schema_t *schema)
{
    if (s_schema_count >= MAX_SCHEMAS) {
        ESP_LOGE(TAG, "schema table full");
        return ESP_ERR_NO_MEM;
    }
    s_schemas[s_schema_count++] = schema;
    ESP_LOGD(TAG, "registered schema: %s (cluster=0x%04x)",
             schema->name, schema->cluster_id);
    return ESP_OK;
}

const zb_cluster_schema_t *zb_schema_find(uint16_t cluster_id)
{
    for (int i = 0; i < s_schema_count; i++) {
        if (s_schemas[i]->cluster_id == cluster_id) return s_schemas[i];
    }
    return NULL;
}

esp_err_t zb_cap_set(uint64_t ieee, zb_cap_id_t cap,
                     const void *value, size_t len)
{
    const zb_cluster_schema_t *s = zb_schema_find(ZB_CAP_CLUSTER(cap));
    if (!s || !s->set) return ESP_ERR_NOT_SUPPORTED;
    return s->set(ieee, cap, value, len);
}

esp_err_t zb_cap_get(uint64_t ieee, zb_cap_id_t cap)
{
    const zb_cluster_schema_t *s = zb_schema_find(ZB_CAP_CLUSTER(cap));
    if (!s || !s->get) return ESP_ERR_NOT_SUPPORTED;
    return s->get(ieee, cap);
}

esp_err_t zb_cap_dispatch_attr(uint64_t ieee, uint8_t ep, uint16_t cluster,
                                uint16_t attr_id, uint8_t dtype,
                                const uint8_t *raw, uint8_t raw_len)
{
    zb_cap_event_t cap_evt = {
        .ieee_addr = ieee,
        .cap_id    = ZB_CAP_ID(ep, cluster),
    };
    translate_to_typed(cluster, attr_id, dtype, raw, raw_len, &cap_evt);

    if (zb_schema_find(cluster) == NULL) {
        ESP_LOGD(TAG, "no schema for cluster 0x%04x — raw event", cluster);
    }

    ESP_LOGD(TAG, "dispatch ieee=%016llx cluster=0x%04x attr=0x%04x",
             (unsigned long long)ieee, cluster, attr_id);

    /* Deliver through the unified event bus (async, notify_task in zb_subscribe.c) */
    zb_event_emit(&(zb_event_t){
        .type     = ZB_EVENT_CAP_REPORT,
        .data.cap = cap_evt,
    });
    return ESP_OK;
}
