#include "zb_storage.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include "esp_log.h"
#include "cJSON.h"

static const char *TAG = "zb_storage_cfg";

#define CONFIG_FILENAME  "/config.json"
#define NWK_KEY_HEX_LEN  32   /* 16 bytes × 2 hex chars */

/* Build path: <mount>/config.json */
static void config_path(char *buf, size_t len)
{
    snprintf(buf, len, "%s%s", CONFIG_ZB_STORAGE_MOUNT_POINT, CONFIG_FILENAME);
}

/* Convert 16-byte key to 32-char lowercase hex string (no NUL written). */
static void key_to_hex(const uint8_t key[16], char hex[NWK_KEY_HEX_LEN + 1])
{
    for (int i = 0; i < 16; i++) {
        snprintf(&hex[i * 2], 3, "%02x", key[i]);
    }
}

/* Parse 32-char hex string into 16-byte key. Returns false on bad input. */
static bool hex_to_key(const char *hex, uint8_t key[16])
{
    if (!hex || strlen(hex) < NWK_KEY_HEX_LEN) {
        return false;
    }
    for (int i = 0; i < 16; i++) {
        unsigned byte;
        if (sscanf(&hex[i * 2], "%02x", &byte) != 1) {
            return false;
        }
        key[i] = (uint8_t)byte;
    }
    return true;
}

esp_err_t zb_storage_config_load(zb_net_config_t *cfg)
{
    char path[80];
    config_path(path, sizeof(path));

    FILE *f = fopen(path, "r");
    if (!f) {
        ESP_LOGD(TAG, "config file not found (%s) — first boot", path);
        return ESP_ERR_NOT_FOUND;
    }

    /* Read entire file into buffer */
    fseek(f, 0, SEEK_END);
    long fsize = ftell(f);
    rewind(f);

    char *buf = malloc((size_t)fsize + 1);
    if (!buf) {
        fclose(f);
        return ESP_ERR_NO_MEM;
    }
    fread(buf, 1, (size_t)fsize, f);
    buf[fsize] = '\0';
    fclose(f);

    cJSON *root = cJSON_Parse(buf);
    free(buf);

    if (!root) {
        ESP_LOGE(TAG, "JSON parse error in %s", path);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t result = ESP_OK;

    cJSON *pan_id_j = cJSON_GetObjectItemCaseSensitive(root, "pan_id");
    cJSON *channel_j = cJSON_GetObjectItemCaseSensitive(root, "channel");
    cJSON *nwk_key_j = cJSON_GetObjectItemCaseSensitive(root, "nwk_key");
    cJSON *baud_j    = cJSON_GetObjectItemCaseSensitive(root, "uart_baud");

    if (!cJSON_IsNumber(pan_id_j) || !cJSON_IsNumber(channel_j) ||
        !cJSON_IsString(nwk_key_j) || !cJSON_IsNumber(baud_j)) {
        ESP_LOGE(TAG, "missing or wrong-type fields in config.json");
        result = ESP_ERR_INVALID_STATE;
        goto cleanup;
    }

    cfg->pan_id    = (uint16_t)pan_id_j->valuedouble;
    cfg->channel   = (uint8_t)channel_j->valuedouble;
    cfg->uart_baud = (uint32_t)baud_j->valuedouble;

    if (!hex_to_key(nwk_key_j->valuestring, cfg->nwk_key)) {
        ESP_LOGE(TAG, "nwk_key field is not a valid 32-char hex string");
        result = ESP_ERR_INVALID_STATE;
        goto cleanup;
    }

    ESP_LOGI(TAG, "config loaded: pan_id=0x%04x channel=%d baud=%"PRIu32,
             cfg->pan_id, cfg->channel, cfg->uart_baud);

cleanup:
    cJSON_Delete(root);
    return result;
}

esp_err_t zb_storage_config_save(const zb_net_config_t *cfg)
{
    char path[80];
    config_path(path, sizeof(path));

    char hex_key[NWK_KEY_HEX_LEN + 1];
    key_to_hex(cfg->nwk_key, hex_key);

    cJSON *root = cJSON_CreateObject();
    if (!root) {
        return ESP_ERR_NO_MEM;
    }

    cJSON_AddNumberToObject(root, "pan_id",    cfg->pan_id);
    cJSON_AddNumberToObject(root, "channel",   cfg->channel);
    cJSON_AddStringToObject(root, "nwk_key",   hex_key);
    cJSON_AddNumberToObject(root, "uart_baud", cfg->uart_baud);

    char *json_str = cJSON_Print(root);
    cJSON_Delete(root);

    if (!json_str) {
        return ESP_ERR_NO_MEM;
    }

    FILE *f = fopen(path, "w");
    if (!f) {
        ESP_LOGE(TAG, "cannot open %s for writing", path);
        free(json_str);
        return ESP_FAIL;
    }

    fputs(json_str, f);
    fclose(f);
    free(json_str);

    ESP_LOGI(TAG, "config saved: pan_id=0x%04x channel=%d", cfg->pan_id, cfg->channel);
    return ESP_OK;
}
