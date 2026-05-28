#include "zb_cap.h"
#include "zb_device_mgr.h"
#include "znp_mt_protocol.h"

#include <string.h>
#include "esp_log.h"

static const char *TAG = "zb_cap_clusters";

/* Auto-incrementing ZCL sequence number */
static uint8_t s_zcl_seq = 0;

/* ZCL frame types */
#define ZCL_FRAME_CLUSTER_SPECIFIC  0x01   /* cluster-specific, client→server, dis. default rsp */
#define ZCL_FRAME_PROFILE_WIDE      0x00   /* profile-wide */
#define ZCL_DISABLE_DEFAULT_RSP     0x10   /* bit 4 = disable default response */

/* ZCL profile-wide command IDs */
#define ZCL_CMD_READ_ATTRIBUTES     0x00
/* ZCL cluster-specific command IDs */
#define ZCL_ONOFF_CMD_OFF           0x00
#define ZCL_ONOFF_CMD_ON            0x01
#define ZCL_LEVEL_CMD_MOVE_TO_LEVEL 0x04
#define ZCL_COLOR_CMD_MOVE_TO_COLOR 0x07

/* ---- Read attribute helper ---- */

static esp_err_t read_attr(uint64_t ieee, zb_cap_id_t cap, uint16_t attr_id)
{
    uint8_t zcl[5];
    zcl[0] = ZCL_FRAME_PROFILE_WIDE | ZCL_DISABLE_DEFAULT_RSP;
    zcl[1] = s_zcl_seq++;
    zcl[2] = ZCL_CMD_READ_ATTRIBUTES;
    zcl[3] = (uint8_t)(attr_id & 0xFF);
    zcl[4] = (uint8_t)(attr_id >> 8);
    return zb_zcl_send(ieee, cap, zcl, sizeof(zcl));
}

/* ---- OnOff (cluster 0x0006) ---- */

static esp_err_t onoff_set(uint64_t ieee, zb_cap_id_t cap,
                            const void *value, size_t len)
{
    if (!value || len < 1) return ESP_ERR_INVALID_ARG;
    bool on = *(const bool *)value;

    uint8_t zcl[3];
    zcl[0] = ZCL_FRAME_CLUSTER_SPECIFIC | ZCL_DISABLE_DEFAULT_RSP;
    zcl[1] = s_zcl_seq++;
    zcl[2] = on ? ZCL_ONOFF_CMD_ON : ZCL_ONOFF_CMD_OFF;
    return zb_zcl_send(ieee, cap, zcl, sizeof(zcl));
}

static esp_err_t onoff_get(uint64_t ieee, zb_cap_id_t cap)
{
    return read_attr(ieee, cap, 0x0000);  /* OnOff attribute */
}

esp_err_t zb_cap_onoff_set(uint64_t ieee, zb_cap_id_t cap, bool on)
{
    return onoff_set(ieee, cap, &on, sizeof(on));
}

esp_err_t zb_cap_onoff_get(uint64_t ieee, zb_cap_id_t cap)
{
    return onoff_get(ieee, cap);
}

/* ---- LevelControl (cluster 0x0008) ---- */

static esp_err_t level_set(uint64_t ieee, zb_cap_id_t cap,
                            const void *value, size_t len)
{
    if (!value || len < 1) return ESP_ERR_INVALID_ARG;
    uint8_t level = *(const uint8_t *)value;

    uint8_t zcl[6];
    zcl[0] = ZCL_FRAME_CLUSTER_SPECIFIC | ZCL_DISABLE_DEFAULT_RSP;
    zcl[1] = s_zcl_seq++;
    zcl[2] = ZCL_LEVEL_CMD_MOVE_TO_LEVEL;
    zcl[3] = level;
    zcl[4] = 0x00;   /* transition time low */
    zcl[5] = 0x00;   /* transition time high (immediate) */
    return zb_zcl_send(ieee, cap, zcl, sizeof(zcl));
}

static esp_err_t level_get(uint64_t ieee, zb_cap_id_t cap)
{
    return read_attr(ieee, cap, 0x0000);  /* CurrentLevel attribute */
}

esp_err_t zb_cap_level_set(uint64_t ieee, zb_cap_id_t cap, uint8_t level)
{
    return level_set(ieee, cap, &level, sizeof(level));
}

esp_err_t zb_cap_level_get(uint64_t ieee, zb_cap_id_t cap)
{
    return level_get(ieee, cap);
}

/* ---- ColorControl (cluster 0x0300) ---- */

typedef struct { uint16_t x; uint16_t y; } color_xy_t;

static esp_err_t color_set(uint64_t ieee, zb_cap_id_t cap,
                            const void *value, size_t len)
{
    if (!value || len < sizeof(color_xy_t)) return ESP_ERR_INVALID_ARG;
    const color_xy_t *c = (const color_xy_t *)value;

    uint8_t zcl[9];
    zcl[0] = ZCL_FRAME_CLUSTER_SPECIFIC | ZCL_DISABLE_DEFAULT_RSP;
    zcl[1] = s_zcl_seq++;
    zcl[2] = ZCL_COLOR_CMD_MOVE_TO_COLOR;
    zcl[3] = (uint8_t)(c->x & 0xFF);
    zcl[4] = (uint8_t)(c->x >> 8);
    zcl[5] = (uint8_t)(c->y & 0xFF);
    zcl[6] = (uint8_t)(c->y >> 8);
    zcl[7] = 0x00;   /* transition time immediate */
    zcl[8] = 0x00;
    return zb_zcl_send(ieee, cap, zcl, sizeof(zcl));
}

esp_err_t zb_cap_color_xy_set(uint64_t ieee, zb_cap_id_t cap,
                               uint16_t x, uint16_t y)
{
    color_xy_t c = { x, y };
    return color_set(ieee, cap, &c, sizeof(c));
}

/* ---- Read-only clusters ---- */

esp_err_t zb_cap_temp_get(uint64_t ieee, zb_cap_id_t cap)
{
    return read_attr(ieee, cap, 0x0000);  /* MeasuredValue */
}

esp_err_t zb_cap_humidity_get(uint64_t ieee, zb_cap_id_t cap)
{
    return read_attr(ieee, cap, 0x0000);  /* MeasuredValue */
}

esp_err_t zb_cap_illuminance_get(uint64_t ieee, zb_cap_id_t cap)
{
    return read_attr(ieee, cap, 0x0000);  /* MeasuredValue */
}

static esp_err_t sensor_get(uint64_t ieee, zb_cap_id_t cap)
{
    return read_attr(ieee, cap, 0x0000);
}

/* ---- Built-in schema table entries ---- */

const zb_cluster_schema_t zb_schema_onoff = {
    .cluster_id = ZCL_CLUSTER_ONOFF,
    .name       = "OnOff",
    .set        = onoff_set,
    .get        = onoff_get,
};

const zb_cluster_schema_t zb_schema_level = {
    .cluster_id = ZCL_CLUSTER_LEVEL,
    .name       = "LevelControl",
    .set        = level_set,
    .get        = level_get,
};

const zb_cluster_schema_t zb_schema_color = {
    .cluster_id = ZCL_CLUSTER_COLOR,
    .name       = "ColorControl",
    .set        = color_set,
    .get        = NULL,
};

const zb_cluster_schema_t zb_schema_temperature = {
    .cluster_id = ZCL_CLUSTER_TEMPERATURE,
    .name       = "TemperatureMeasurement",
    .set        = NULL,
    .get        = sensor_get,
};

const zb_cluster_schema_t zb_schema_humidity = {
    .cluster_id = ZCL_CLUSTER_HUMIDITY,
    .name       = "RelativeHumidity",
    .set        = NULL,
    .get        = sensor_get,
};

const zb_cluster_schema_t zb_schema_illuminance = {
    .cluster_id = ZCL_CLUSTER_ILLUMINANCE,
    .name       = "IlluminanceMeasurement",
    .set        = NULL,
    .get        = sensor_get,
};
