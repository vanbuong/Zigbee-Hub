#include "zb_storage.h"

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "pb_encode.h"
#include "pb_decode.h"
#include "zb_device.pb.h"

static const char *TAG = "zb_storage_dev";

#define DEVICES_DIR_FMT   "%s/devices"
#define DEVICE_PATH_FMT   "%s/devices/%016llx.pb"
#define PB_BUF_SIZE       CONFIG_ZB_DEV_PB_MAX_SIZE

/* Build path string for a device file. */
static void device_path(uint64_t ieee_addr, char *buf, size_t len)
{
    snprintf(buf, len, DEVICE_PATH_FMT,
             CONFIG_ZB_STORAGE_MOUNT_POINT, (unsigned long long)ieee_addr);
}

/* ---- Save ---- */

esp_err_t zb_storage_device_save(const ZbDeviceRecord *dev)
{
    char path[80];
    device_path(dev->ieee_addr, path, sizeof(path));

    uint8_t *pb_buf = malloc(PB_BUF_SIZE);
    if (!pb_buf) {
        return ESP_ERR_NO_MEM;
    }

    pb_ostream_t stream = pb_ostream_from_buffer(pb_buf, PB_BUF_SIZE);
    if (!pb_encode(&stream, ZbDeviceRecord_fields, dev)) {
        ESP_LOGE(TAG, "pb_encode failed for %016llx: %s",
                 (unsigned long long)dev->ieee_addr, PB_GET_ERROR(&stream));
        free(pb_buf);
        return ESP_FAIL;
    }

    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "cannot open %s for writing", path);
        free(pb_buf);
        return ESP_FAIL;
    }

    size_t written = fwrite(pb_buf, 1, stream.bytes_written, f);
    fclose(f);
    free(pb_buf);

    if (written != stream.bytes_written) {
        ESP_LOGE(TAG, "short write to %s (%u of %u bytes)",
                 path, (unsigned)written, (unsigned)stream.bytes_written);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "saved device %016llx (%u bytes)",
             (unsigned long long)dev->ieee_addr, (unsigned)stream.bytes_written);
    return ESP_OK;
}

/* ---- Load single ---- */

esp_err_t zb_storage_device_load(uint64_t ieee_addr, ZbDeviceRecord *dev)
{
    char path[80];
    device_path(ieee_addr, path, sizeof(path));

    FILE *f = fopen(path, "rb");
    if (!f) {
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t *pb_buf = malloc(PB_BUF_SIZE);
    if (!pb_buf) {
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    size_t len = fread(pb_buf, 1, PB_BUF_SIZE, f);
    fclose(f);

    *dev = ZbDeviceRecord_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(pb_buf, len);
    bool ok = pb_decode(&stream, ZbDeviceRecord_fields, dev);
    free(pb_buf);

    if (!ok) {
        ESP_LOGE(TAG, "pb_decode failed for %016llx: %s",
                 (unsigned long long)ieee_addr, PB_GET_ERROR(&stream));
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "loaded device %016llx (%u bytes, %d endpoints)",
             (unsigned long long)dev->ieee_addr,
             (unsigned)len, (int)dev->endpoints_count);
    return ESP_OK;
}

/* ---- Delete ---- */

esp_err_t zb_storage_device_delete(uint64_t ieee_addr)
{
    char path[80];
    device_path(ieee_addr, path, sizeof(path));

    if (remove(path) != 0) {
        if (errno == ENOENT) {
            return ESP_ERR_NOT_FOUND;
        }
        ESP_LOGE(TAG, "remove %s failed: %d", path, errno);
        return ESP_FAIL;
    }

    ESP_LOGD(TAG, "deleted device %016llx", (unsigned long long)ieee_addr);
    return ESP_OK;
}

/* ---- Load all (bootup) ---- */

esp_err_t zb_storage_device_load_all(zb_storage_device_cb_t cb, void *ctx)
{
    char dir_path[64];
    snprintf(dir_path, sizeof(dir_path), DEVICES_DIR_FMT,
             CONFIG_ZB_STORAGE_MOUNT_POINT);

    DIR *dir = opendir(dir_path);
    if (!dir) {
        /* Directory missing or empty — not an error on first boot */
        ESP_LOGD(TAG, "devices dir not found or empty: %s", dir_path);
        return ESP_OK;
    }

    int count = 0;
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        /* Only process files with .pb extension */
        const char *dot = strrchr(entry->d_name, '.');
        if (!dot || strcmp(dot, ".pb") != 0) {
            continue;
        }

        /* Parse IEEE address from filename (16 hex chars) */
        unsigned long long ieee_raw = 0;
        if (sscanf(entry->d_name, "%016llx.pb", &ieee_raw) != 1) {
            ESP_LOGW(TAG, "skipping unexpected file: %s", entry->d_name);
            continue;
        }

        ZbDeviceRecord dev = ZbDeviceRecord_init_zero;
        esp_err_t err = zb_storage_device_load((uint64_t)ieee_raw, &dev);
        if (err == ESP_OK) {
            cb(&dev, ctx);
            count++;
        } else {
            ESP_LOGW(TAG, "failed to load %s: %s", entry->d_name, esp_err_to_name(err));
        }
    }

    closedir(dir);
    ESP_LOGI(TAG, "loaded %d device(s) from storage", count);
    return ESP_OK;
}
