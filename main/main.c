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
#include "zb_cap.h"
#include "zb_device_mgr.h"
#include "zb_subscribe.h"
#include "zb_cmd.h"

static const char *TAG = "main";

/* Network-level event callback */
static void on_network_event(const zb_event_t *e, void *ctx)
{
    switch (e->type) {
    case ZB_EVENT_NETWORK_READY:
        ESP_LOGI(TAG, "[net] NETWORK_READY");
        /* Open for joining now that we're confirmed READY */
        zb_cmd_permit_join(60);
        break;
    case ZB_EVENT_NETWORK_LOST:
        ESP_LOGW(TAG, "[net] NETWORK_LOST");
        break;
    case ZB_EVENT_DEVICE_JOINED:
        ESP_LOGI(TAG, "[net] DEVICE_JOINED %016llx nwk=0x%04x",
                 (unsigned long long)e->data.device.ieee_addr,
                 e->data.device.nwk_addr);
        break;
    case ZB_EVENT_DEVICE_LEFT:
        ESP_LOGI(TAG, "[net] DEVICE_LEFT %016llx",
                 (unsigned long long)e->data.device.ieee_addr);
        break;
    default:
        break;
    }
}

/* Capability event callback — upper layer sees no raw ZCL details */
static void on_cap_event(const zb_cap_event_t *e, void *ctx)
{
    uint16_t cluster = ZB_CAP_CLUSTER(e->cap_id);
    switch (cluster) {
    case ZCL_CLUSTER_ONOFF:
        ESP_LOGI(TAG, "[cap] %016llx OnOff=%s",
                 (unsigned long long)e->ieee_addr,
                 e->value.on_off ? "ON" : "OFF");
        break;
    case ZCL_CLUSTER_LEVEL:
        ESP_LOGI(TAG, "[cap] %016llx Level=%d",
                 (unsigned long long)e->ieee_addr, e->value.level);
        break;
    case ZCL_CLUSTER_TEMPERATURE:
        ESP_LOGI(TAG, "[cap] %016llx Temp=%.2f C",
                 (unsigned long long)e->ieee_addr,
                 e->value.temperature_hundredths / 100.0f);
        break;
    case ZCL_CLUSTER_HUMIDITY:
        ESP_LOGI(TAG, "[cap] %016llx Humidity=%.2f%%",
                 (unsigned long long)e->ieee_addr,
                 e->value.humidity_hundredths / 100.0f);
        break;
    default:
        ESP_LOGI(TAG, "[cap] %016llx cluster=0x%04x attr=0x%04x",
                 (unsigned long long)e->ieee_addr,
                 cluster, e->value.raw.attr_id);
        break;
    }
}

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

    /* Subscribe to network-level events (permit-join opens automatically on READY) */
    zb_subscribe(ZB_EVENT_NETWORK_READY, on_network_event, NULL);
    zb_subscribe(ZB_EVENT_NETWORK_LOST,  on_network_event, NULL);
    zb_subscribe(ZB_EVENT_DEVICE_JOINED, on_network_event, NULL);
    zb_subscribe(ZB_EVENT_DEVICE_LEFT,   on_network_event, NULL);

    /* Subscribe to capability events for a known device.
     * Replace DEMO_IEEE with the actual IEEE address of a joined device. */
    static const uint64_t DEMO_IEEE = 0x00124B001234ABCDULL;
    zb_cap_subscribe(DEMO_IEEE, ZB_CAP_ANY, on_cap_event, NULL);

    /* Keep app_main alive; the framework task handles everything from here */
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
