#include "zb_cmd.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "zb_cmd";

#define CMD_QUEUE_DEPTH  CONFIG_ZB_CMD_QUEUE_DEPTH

static QueueHandle_t s_queue;

esp_err_t zb_cmd_init(void)
{
    s_queue = xQueueCreate(CMD_QUEUE_DEPTH, sizeof(zb_cmd_t));
    if (!s_queue) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "command queue initialized (depth=%d)", CMD_QUEUE_DEPTH);
    return ESP_OK;
}

esp_err_t zb_cmd_permit_join(uint8_t duration_s)
{
    if (!s_queue) return ESP_ERR_INVALID_STATE;
    zb_cmd_t cmd = { .type = ZB_CMD_PERMIT_JOIN, .params.duration_s = duration_s };
    if (xQueueSend(s_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "command queue full — permit_join dropped");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGD(TAG, "permit_join(%d s) enqueued", duration_s);
    return ESP_OK;
}

esp_err_t zb_cmd_change_channel(uint8_t channel)
{
    if (!s_queue) return ESP_ERR_INVALID_STATE;
    if (channel < 11 || channel > 26) return ESP_ERR_INVALID_ARG;
    zb_cmd_t cmd = { .type = ZB_CMD_CHANGE_CHANNEL, .params.channel = channel };
    if (xQueueSend(s_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "command queue full — change_channel dropped");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGD(TAG, "change_channel(%d) enqueued", channel);
    return ESP_OK;
}

BaseType_t zb_cmd_dequeue(zb_cmd_t *cmd, TickType_t ticks_to_wait)
{
    if (!s_queue) return pdFALSE;
    return xQueueReceive(s_queue, cmd, ticks_to_wait);
}
