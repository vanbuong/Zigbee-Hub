#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "zb_device.pb.h"   /* nanopb-generated ZbDeviceRecord */

/* ---- Network configuration stored as JSON ---- */

typedef struct {
    uint16_t pan_id;
    uint8_t  channel;
    uint8_t  nwk_key[16];
    uint32_t uart_baud;
} zb_net_config_t;

/* Callback invoked by zb_storage_device_load_all() for each loaded device. */
typedef void (*zb_storage_device_cb_t)(const ZbDeviceRecord *dev, void *ctx);

/**
 * Mount the LittleFS partition and create required directories.
 * Must be called once, before any other zb_storage_* function.
 * Partition label and mount point are set by Kconfig (ZB_STORAGE_PARTITION_LABEL,
 * ZB_STORAGE_MOUNT_POINT).
 */
esp_err_t zb_storage_init(void);

/* ---- Config (JSON) ---- */

/**
 * Load network configuration from <mount>/config.json.
 * @return ESP_ERR_NOT_FOUND if the file does not exist (first boot — caller uses defaults).
 */
esp_err_t zb_storage_config_load(zb_net_config_t *cfg);

/**
 * Persist network configuration to <mount>/config.json.
 * Overwrites any existing file.
 */
esp_err_t zb_storage_config_save(const zb_net_config_t *cfg);

/* ---- Device registry (protobuf, one file per device) ---- */

/**
 * Encode a ZbDeviceRecord to protobuf and write it to
 * <mount>/devices/<ieee_hex>.pb. Creates or overwrites the file.
 */
esp_err_t zb_storage_device_save(const ZbDeviceRecord *dev);

/**
 * Read and decode a device record from <mount>/devices/<ieee_hex>.pb.
 * @return ESP_ERR_NOT_FOUND if the device file does not exist.
 */
esp_err_t zb_storage_device_load(uint64_t ieee_addr, ZbDeviceRecord *dev);

/**
 * Delete the device file for ieee_addr.
 * @return ESP_ERR_NOT_FOUND if the file does not exist (treated as success by callers).
 */
esp_err_t zb_storage_device_delete(uint64_t ieee_addr);

/**
 * Iterate all *.pb files in <mount>/devices/ and invoke cb for each successfully
 * decoded record. Used at bootup to pre-populate the device manager.
 * @return ESP_OK even if no files exist.
 */
esp_err_t zb_storage_device_load_all(zb_storage_device_cb_t cb, void *ctx);
