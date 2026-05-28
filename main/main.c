#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "zb_framework.h"
#include "zb_types.h"

static const char *TAG = "main";

/* Default network key — replace with your own 16-byte key */
static const uint8_t DEFAULT_NWK_KEY[16] = {
    0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F,
    0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0D
};

void app_main(void)
{
    /* Initialize NVS — required before framework init */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition problem (%s) — erasing and reinitializing",
                 esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized");

    /* Configure the Zigbee framework */
    zb_config_t cfg = {
        .pan_id     = CONFIG_ZB_DEFAULT_PAN_ID,
        .channel    = CONFIG_ZB_DEFAULT_CHANNEL,
        .uart_port  = CONFIG_ZNP_UART_PORT,
        .uart_baud  = CONFIG_ZNP_UART_BAUD,
        .gpio_tx    = CONFIG_ZNP_GPIO_TX,
        .gpio_rx    = CONFIG_ZNP_GPIO_RX,
        .gpio_reset = CONFIG_ZNP_GPIO_RESET,
        .gpio_bsl   = CONFIG_ZNP_GPIO_BSL,
    };
    memcpy(cfg.nwk_key, DEFAULT_NWK_KEY, sizeof(DEFAULT_NWK_KEY));

    ESP_ERROR_CHECK(zb_framework_init(&cfg));
    ESP_ERROR_CHECK(zb_framework_start());

    ESP_LOGI(TAG, "framework started — waiting for READY state");

    /* Poll state until READY (smoke-test; Phase 3 will use the Subscribe API) */
    for (;;) {
        zb_state_t state = zb_framework_get_state();
        if (state == ZB_STATE_READY) {
            ESP_LOGI(TAG, "hub is READY");

            zb_versions_t ver;
            if (zb_framework_get_versions(&ver) == ESP_OK) {
                ESP_LOGI(TAG, "ESP32 fw: %s", ver.esp32_fw);
                ESP_LOGI(TAG, "CC2652P7 fw: %s", ver.cc26xx_fw);
            }
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    ESP_LOGI(TAG, "init complete — hub running");

    /* Keep app_main alive; the framework task handles everything from here */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
