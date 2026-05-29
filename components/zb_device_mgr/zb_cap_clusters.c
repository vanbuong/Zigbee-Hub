#include "zb_cap.h"
#include "zb_device_mgr.h"
#include "znp_mt_protocol.h"

#include <string.h>
#include "esp_log.h"

__attribute__((unused))
static const char *TAG = "zb_cap_clusters";

/* Auto-incrementing ZCL sequence number */
static uint8_t s_zcl_seq = 0;

/* ZCL frame types */
#define ZCL_FRAME_CLUSTER_SPECIFIC  0x01   /* cluster-specific, client→server, dis. default rsp */
#define ZCL_FRAME_PROFILE_WIDE      0x00   /* profile-wide */
#define ZCL_DISABLE_DEFAULT_RSP     0x10   /* bit 4 = disable default response */

/* ZCL profile-wide command IDs */
#define ZCL_CMD_READ_ATTRIBUTES     0x00
#define ZCL_CMD_WRITE_ATTRIBUTES    0x02
/* ZCL cluster-specific command IDs */
#define ZCL_ONOFF_CMD_OFF           0x00
#define ZCL_ONOFF_CMD_ON            0x01
#define ZCL_LEVEL_CMD_MOVE_TO_LEVEL 0x04
#define ZCL_COLOR_CMD_MOVE_TO_COLOR 0x07

/* ZCL scalar data types used by FR-12 writes */
#define ZCL_TYPE_BOOLEAN            0x10

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

/* Read multiple attributes in one ZCL frame: attr_id[2] * n */
static esp_err_t read_attrs(uint64_t ieee, zb_cap_id_t cap,
                             const uint16_t *attr_ids, uint8_t n)
{
    if (!attr_ids || n == 0 || n > 8) return ESP_ERR_INVALID_ARG;
    uint8_t zcl[3 + 8 * 2];
    zcl[0] = ZCL_FRAME_PROFILE_WIDE | ZCL_DISABLE_DEFAULT_RSP;
    zcl[1] = s_zcl_seq++;
    zcl[2] = ZCL_CMD_READ_ATTRIBUTES;
    for (uint8_t i = 0; i < n; i++) {
        zcl[3 + i * 2]     = (uint8_t)(attr_ids[i] & 0xFF);
        zcl[3 + i * 2 + 1] = (uint8_t)(attr_ids[i] >> 8);
    }
    return zb_zcl_send(ieee, cap, zcl, 3 + n * 2);
}

/* Write a single scalar attribute: attr_id[2] dtype[1] value[n] */
static esp_err_t write_attr(uint64_t ieee, zb_cap_id_t cap,
                             uint16_t attr_id, uint8_t dtype,
                             const uint8_t *val, uint8_t vlen)
{
    if (!val || vlen == 0 || vlen > 8) return ESP_ERR_INVALID_ARG;
    uint8_t zcl[3 + 3 + 8];
    zcl[0] = ZCL_FRAME_PROFILE_WIDE | ZCL_DISABLE_DEFAULT_RSP;
    zcl[1] = s_zcl_seq++;
    zcl[2] = ZCL_CMD_WRITE_ATTRIBUTES;
    zcl[3] = (uint8_t)(attr_id & 0xFF);
    zcl[4] = (uint8_t)(attr_id >> 8);
    zcl[5] = dtype;
    memcpy(&zcl[6], val, vlen);
    return zb_zcl_send(ieee, cap, zcl, 6 + vlen);
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

/* =========================================================================
 *  FR-12 extended ZHA clusters
 * ========================================================================= */

/* ---- PressureMeasurement (cluster 0x0403) — read-only ---- */

static esp_err_t pressure_get(uint64_t ieee, zb_cap_id_t cap)
{
    return read_attr(ieee, cap, 0x0000);  /* MeasuredValue */
}

esp_err_t zb_cap_pressure_get(uint64_t ieee, zb_cap_id_t cap)
{
    return pressure_get(ieee, cap);
}

const zb_cluster_schema_t zb_schema_pressure = {
    .cluster_id = ZCL_CLUSTER_PRESSURE,
    .name       = "PressureMeasurement",
    .set        = NULL,
    .get        = pressure_get,
};

/* ---- OccupancySensing (cluster 0x0406) — read-only ---- */

static esp_err_t occupancy_get(uint64_t ieee, zb_cap_id_t cap)
{
    return read_attr(ieee, cap, 0x0000);  /* Occupancy bitmap */
}

esp_err_t zb_cap_occupancy_get(uint64_t ieee, zb_cap_id_t cap)
{
    return occupancy_get(ieee, cap);
}

const zb_cluster_schema_t zb_schema_occupancy = {
    .cluster_id = ZCL_CLUSTER_OCCUPANCY,
    .name       = "OccupancySensing",
    .set        = NULL,
    .get        = occupancy_get,
};

/* ---- IASZone (cluster 0x0500) — read-only attrs ----
 *
 * Cluster-specific command 0x00 (ZONE_STATUS_CHANGE_NOTIFICATION) arrives
 * unsolicited and is handled in zb_device_mgr.c via
 * zb_cap_dispatch_ias_status_change(). The generic get() reads ZoneState,
 * ZoneType, and ZoneStatus in one round-trip. */

static esp_err_t ias_zone_get(uint64_t ieee, zb_cap_id_t cap)
{
    static const uint16_t attrs[3] = {0x0000, 0x0001, 0x0002};
    return read_attrs(ieee, cap, attrs, 3);
}

esp_err_t zb_cap_ias_zone_get(uint64_t ieee, zb_cap_id_t cap)
{
    return ias_zone_get(ieee, cap);
}

const zb_cluster_schema_t zb_schema_ias_zone = {
    .cluster_id = ZCL_CLUSTER_IAS_ZONE,
    .name       = "IASZone",
    .set        = NULL,
    .get        = ias_zone_get,
};

/* ---- Metering (cluster 0x0702) — FR-12.4 / 12.7 ---- */

static esp_err_t metering_get(uint64_t ieee, zb_cap_id_t cap)
{
    /* Read the three primary metering attrs + Multiplier/Divisor in one frame.
     * FR-12.7: the scaling attrs come back together so upper layers don't
     * have to manage two-step reads. */
    static const uint16_t attrs[5] = {
        0x0000, /* CurrentSummationDelivered  uint48 */
        0x0001, /* CurrentSummationReceived   uint48 */
        0x0400, /* InstantaneousDemand        int24  */
        0x0301, /* Multiplier                 uint24 */
        0x0302, /* Divisor                    uint24 */
    };
    return read_attrs(ieee, cap, attrs, 5);
}

esp_err_t zb_cap_metering_get(uint64_t ieee, zb_cap_id_t cap)
{
    return metering_get(ieee, cap);
}

const zb_cluster_schema_t zb_schema_metering = {
    .cluster_id = ZCL_CLUSTER_METERING,
    .name       = "Metering",
    .set        = NULL,
    .get        = metering_get,
};

/* ---- ElectricalMeasurement (cluster 0x0B04) — FR-12.5 ---- */

static esp_err_t electrical_get(uint64_t ieee, zb_cap_id_t cap)
{
    static const uint16_t attrs[4] = {
        0x0505, /* RMSVoltage  uint16 (0.1 V) */
        0x0508, /* RMSCurrent  uint16 (mA)    */
        0x050B, /* ActivePower int16  (W)     */
        0x0510, /* PowerFactor int8           */
    };
    return read_attrs(ieee, cap, attrs, 4);
}

esp_err_t zb_cap_electrical_get(uint64_t ieee, zb_cap_id_t cap)
{
    return electrical_get(ieee, cap);
}

const zb_cluster_schema_t zb_schema_electrical = {
    .cluster_id = ZCL_CLUSTER_ELECTRICAL,
    .name       = "ElectricalMeasurement",
    .set        = NULL,
    .get        = electrical_get,
};

/* ---- AnalogInput (cluster 0x000C) — read-only ---- */

static esp_err_t analog_input_get(uint64_t ieee, zb_cap_id_t cap)
{
    return read_attr(ieee, cap, 0x0055);  /* PresentValue (float32) */
}

esp_err_t zb_cap_analog_input_get(uint64_t ieee, zb_cap_id_t cap)
{
    return analog_input_get(ieee, cap);
}

const zb_cluster_schema_t zb_schema_analog_input = {
    .cluster_id = ZCL_CLUSTER_ANALOG_INPUT,
    .name       = "AnalogInput",
    .set        = NULL,
    .get        = analog_input_get,
};

/* ---- BinaryInput (cluster 0x000F) — read-only ---- */

static esp_err_t binary_input_get(uint64_t ieee, zb_cap_id_t cap)
{
    return read_attr(ieee, cap, 0x0055);  /* PresentValue (bool) */
}

esp_err_t zb_cap_binary_input_get(uint64_t ieee, zb_cap_id_t cap)
{
    return binary_input_get(ieee, cap);
}

const zb_cluster_schema_t zb_schema_binary_input = {
    .cluster_id = ZCL_CLUSTER_BINARY_INPUT,
    .name       = "BinaryInput",
    .set        = NULL,
    .get        = binary_input_get,
};

/* ---- BinaryOutput (cluster 0x0010) — FR-12.6, settable via Write Attributes ---- */

static esp_err_t binary_output_set(uint64_t ieee, zb_cap_id_t cap,
                                    const void *value, size_t len)
{
    if (!value || len < 1) return ESP_ERR_INVALID_ARG;
    uint8_t v = (*(const bool *)value) ? 0x01 : 0x00;
    return write_attr(ieee, cap, 0x0055, ZCL_TYPE_BOOLEAN, &v, 1);
}

static esp_err_t binary_output_get(uint64_t ieee, zb_cap_id_t cap)
{
    return read_attr(ieee, cap, 0x0055);
}

esp_err_t zb_cap_binary_output_set(uint64_t ieee, zb_cap_id_t cap, bool value)
{
    return binary_output_set(ieee, cap, &value, sizeof(value));
}

esp_err_t zb_cap_binary_output_get(uint64_t ieee, zb_cap_id_t cap)
{
    return binary_output_get(ieee, cap);
}

const zb_cluster_schema_t zb_schema_binary_output = {
    .cluster_id = ZCL_CLUSTER_BINARY_OUTPUT,
    .name       = "BinaryOutput",
    .set        = binary_output_set,
    .get        = binary_output_get,
};
