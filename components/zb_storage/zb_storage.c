#include "zb_storage.h"

#include <sys/stat.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_littlefs.h"

static const char *TAG = "zb_storage";

#define DEVICES_SUBDIR  "/devices"

esp_err_t zb_storage_init(void)
{
    const char *partition = CONFIG_ZB_STORAGE_PARTITION_LABEL;
    const char *mount     = CONFIG_ZB_STORAGE_MOUNT_POINT;

    esp_vfs_littlefs_conf_t conf = {
        .base_path             = mount,
        .partition_label       = partition,
        .format_if_mount_failed = CONFIG_ZB_STORAGE_FORMAT_ON_FAIL,
        .dont_mount            = false,
    };

    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to mount LittleFS (partition=%s): %s",
                 partition, esp_err_to_name(err));
        return err;
    }

    size_t total = 0, used = 0;
    esp_littlefs_info(partition, &total, &used);
    ESP_LOGI(TAG, "LittleFS mounted: %s  total=%u used=%u",
             mount, (unsigned)total, (unsigned)used);

    /* Ensure devices directory exists */
    char devices_path[64];
    snprintf(devices_path, sizeof(devices_path), "%s%s", mount, DEVICES_SUBDIR);

    struct stat st;
    if (stat(devices_path, &st) != 0) {
        if (mkdir(devices_path, 0777) != 0 && errno != EEXIST) {
            ESP_LOGE(TAG, "mkdir %s failed: %d", devices_path, errno);
            return ESP_FAIL;
        }
        ESP_LOGD(TAG, "created %s", devices_path);
    }

    return ESP_OK;
}
