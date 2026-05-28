#include "zb_cap.h"
#include "zb_subscribe.h"
#include "zb_device_mgr.h"

#include <string.h>
#include "esp_log.h"

static const char *TAG = "zb_cap";

/* ---- Schema registry ---- */

#define MAX_SCHEMAS 16

static const zb_cluster_schema_t *s_schemas[MAX_SCHEMAS];
static uint8_t s_schema_count = 0;

extern const zb_cluster_schema_t zb_schema_onoff;
extern const zb_cluster_schema_t zb_schema_level;
extern const zb_cluster_schema_t zb_schema_color;
extern const zb_cluster_schema_t zb_schema_temperature;
extern const zb_cluster_schema_t zb_schema_humidity;
extern const zb_cluster_schema_t zb_schema_illuminance;

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

    ESP_LOGI(TAG, "initialized with %d built-in schemas", s_schema_count);
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
