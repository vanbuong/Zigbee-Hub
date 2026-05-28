#include "zb_subscribe.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "zb_sub";

#define MAX_EVENT_SUBSCRIBERS 16

typedef struct {
    bool         active;
    zb_event_cb_t cb;
    void         *ctx;
} sub_slot_t;

/* Per-event-type subscriber tables */
static sub_slot_t      s_table[ZB_EVENT_TYPE_MAX][MAX_EVENT_SUBSCRIBERS];
static SemaphoreHandle_t s_mutex;

/* Called once from zb_dev_mgr_init() */
void zb_subscribe_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    memset(s_table, 0, sizeof(s_table));
}

esp_err_t zb_subscribe(zb_event_type_t event, zb_event_cb_t cb, void *ctx)
{
    if (event >= ZB_EVENT_TYPE_MAX || !cb) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_EVENT_SUBSCRIBERS; i++) {
        if (!s_table[event][i].active) {
            s_table[event][i] = (sub_slot_t){ .active = true, .cb = cb, .ctx = ctx };
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_mutex);
    ESP_LOGE(TAG, "subscriber table full for event %d", event);
    return ESP_ERR_NO_MEM;
}

esp_err_t zb_unsubscribe(zb_event_type_t event, zb_event_cb_t cb)
{
    if (event >= ZB_EVENT_TYPE_MAX) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_EVENT_SUBSCRIBERS; i++) {
        if (s_table[event][i].active && s_table[event][i].cb == cb) {
            s_table[event][i].active = false;
            break;
        }
    }
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

void zb_event_emit(const zb_event_t *event)
{
    if (!event || event->type >= ZB_EVENT_TYPE_MAX) return;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < MAX_EVENT_SUBSCRIBERS; i++) {
        if (s_table[event->type][i].active) {
            s_table[event->type][i].cb(event, s_table[event->type][i].ctx);
        }
    }
    xSemaphoreGive(s_mutex);
}
