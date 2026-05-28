#include "zb_cap.h"
#include "zb_device_mgr.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "zb_cap";

/* ---- Schema registry ---- */

#define MAX_SCHEMAS 16

static const zb_cluster_schema_t *s_schemas[MAX_SCHEMAS];
static uint8_t s_schema_count = 0;

/* External built-in schemas defined in zb_cap_clusters.c */
extern const zb_cluster_schema_t zb_schema_onoff;
extern const zb_cluster_schema_t zb_schema_level;
extern const zb_cluster_schema_t zb_schema_color;
extern const zb_cluster_schema_t zb_schema_temperature;
extern const zb_cluster_schema_t zb_schema_humidity;
extern const zb_cluster_schema_t zb_schema_illuminance;

/* ---- Subscription table ---- */

#define MAX_SUBSCRIBERS 32

typedef struct {
    bool              active;
    uint64_t          ieee;
    zb_cap_id_t       cap;
    zb_cap_event_cb_t cb;
    void             *ctx;
} sub_entry_t;

static sub_entry_t     s_subs[MAX_SUBSCRIBERS];
static SemaphoreHandle_t s_sub_mutex;

/* ---- Helpers ---- */

/* Return byte size of a ZCL scalar data type; 0 = unknown/variable */
static uint8_t zcl_type_size(uint8_t dtype)
{
    switch (dtype) {
    case 0x10: return 1;   /* boolean */
    case 0x20: return 1;   /* uint8   */
    case 0x21: return 2;   /* uint16  */
    case 0x22: return 3;   /* uint24  */
    case 0x23: return 4;   /* uint32  */
    case 0x28: return 1;   /* int8    */
    case 0x29: return 2;   /* int16   */
    case 0x2A: return 3;   /* int24   */
    case 0x2B: return 4;   /* int32   */
    default:   return 0;
    }
}

/* Translate raw ZCL attribute bytes to the typed value union */
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
            evt->value.humidity_hundredths =
                (uint16_t)(v[0] | (v[1] << 8));
        break;

    case ZCL_CLUSTER_ILLUMINANCE:
        if (vlen >= 2) {
            uint16_t raw16 = (uint16_t)(v[0] | (v[1] << 8));
            /* ZCL illuminance: MeasuredValue = 10000 * log10(lux) + 1 */
            evt->value.illuminance_lux = raw16;
        }
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
    s_sub_mutex = xSemaphoreCreateMutex();
    if (!s_sub_mutex) return ESP_ERR_NO_MEM;

    memset(s_subs, 0, sizeof(s_subs));

    /* Register built-in cluster schemas */
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
        if (s_schemas[i]->cluster_id == cluster_id) {
            return s_schemas[i];
        }
    }
    return NULL;
}

esp_err_t zb_cap_subscribe(uint64_t ieee, zb_cap_id_t cap,
                            zb_cap_event_cb_t cb, void *ctx)
{
    xSemaphoreTake(s_sub_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (!s_subs[i].active) {
            s_subs[i] = (sub_entry_t){
                .active = true,
                .ieee   = ieee,
                .cap    = cap,
                .cb     = cb,
                .ctx    = ctx,
            };
            xSemaphoreGive(s_sub_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_sub_mutex);
    ESP_LOGE(TAG, "subscriber table full");
    return ESP_ERR_NO_MEM;
}

esp_err_t zb_cap_unsubscribe(uint64_t ieee, zb_cap_id_t cap,
                              zb_cap_event_cb_t cb)
{
    xSemaphoreTake(s_sub_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (s_subs[i].active &&
            s_subs[i].ieee == ieee &&
            s_subs[i].cap  == cap  &&
            s_subs[i].cb   == cb) {
            s_subs[i].active = false;
            break;
        }
    }
    xSemaphoreGive(s_sub_mutex);
    return ESP_OK;
}

esp_err_t zb_cap_set(uint64_t ieee, zb_cap_id_t cap,
                     const void *value, size_t len)
{
    const zb_cluster_schema_t *s = zb_schema_find(ZB_CAP_CLUSTER(cap));
    if (!s || !s->set) {
        ESP_LOGW(TAG, "no set function for cluster 0x%04x", ZB_CAP_CLUSTER(cap));
        return ESP_ERR_NOT_SUPPORTED;
    }
    return s->set(ieee, cap, value, len);
}

esp_err_t zb_cap_get(uint64_t ieee, zb_cap_id_t cap)
{
    const zb_cluster_schema_t *s = zb_schema_find(ZB_CAP_CLUSTER(cap));
    if (!s || !s->get) {
        ESP_LOGW(TAG, "no get function for cluster 0x%04x", ZB_CAP_CLUSTER(cap));
        return ESP_ERR_NOT_SUPPORTED;
    }
    return s->get(ieee, cap);
}

esp_err_t zb_cap_dispatch_attr(uint64_t ieee, uint8_t ep, uint16_t cluster,
                                uint16_t attr_id, uint8_t dtype,
                                const uint8_t *raw, uint8_t raw_len)
{
    zb_cap_id_t cap = ZB_CAP_ID(ep, cluster);

    zb_cap_event_t evt = { .ieee_addr = ieee, .cap_id = cap };
    translate_to_typed(cluster, attr_id, dtype, raw, raw_len, &evt);

    ESP_LOGD(TAG, "dispatch ieee=%016llx cap=0x%08"PRIx32" cluster=0x%04x attr=0x%04x",
             (unsigned long long)ieee, (uint32_t)cap, cluster, attr_id);

    /* Notify matching subscribers */
    xSemaphoreTake(s_sub_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (!s_subs[i].active) continue;
        if (s_subs[i].ieee != ieee) continue;
        if (s_subs[i].cap != ZB_CAP_ANY && s_subs[i].cap != cap) continue;
        s_subs[i].cb(&evt, s_subs[i].ctx);
    }
    xSemaphoreGive(s_sub_mutex);

    /* Emit raw event for unknown clusters (FR-9.8) */
    if (zb_schema_find(cluster) == NULL) {
        ESP_LOGD(TAG, "no schema for cluster 0x%04x — raw event only", cluster);
    }

    return ESP_OK;
}
