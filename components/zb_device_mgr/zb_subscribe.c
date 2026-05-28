#include "zb_subscribe.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "zb_sub";

#define MAX_EVENT_SUBSCRIBERS  16
#define NOTIFY_QUEUE_DEPTH     32

typedef struct {
    bool          active;
    zb_event_cb_t cb;
    void         *ctx;
} sub_slot_t;

/* Per-event-type subscriber table */
static sub_slot_t        s_table[ZB_EVENT_TYPE_MAX][MAX_EVENT_SUBSCRIBERS];
static SemaphoreHandle_t s_mutex;

/* Async delivery queue + task (FR-5.8: callbacks from dedicated task) */
static QueueHandle_t     s_notify_queue;

static void notify_task(void *arg)
{
    zb_event_t evt;
    for (;;) {
        if (xQueueReceive(s_notify_queue, &evt, portMAX_DELAY) != pdTRUE) continue;
        if (evt.type >= ZB_EVENT_TYPE_MAX) continue;

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        for (int i = 0; i < MAX_EVENT_SUBSCRIBERS; i++) {
            if (s_table[evt.type][i].active) {
                s_table[evt.type][i].cb(&evt, s_table[evt.type][i].ctx);
            }
        }
        xSemaphoreGive(s_mutex);
    }
}

/* Called once from zb_dev_mgr_init() */
void zb_subscribe_init(void)
{
    s_mutex        = xSemaphoreCreateMutex();
    s_notify_queue = xQueueCreate(NOTIFY_QUEUE_DEPTH, sizeof(zb_event_t));
    memset(s_table, 0, sizeof(s_table));

    xTaskCreate(notify_task, "zb_notify",
                CONFIG_ZB_NOTIFY_TASK_STACK, NULL,
                CONFIG_ZB_FRAMEWORK_TASK_PRIORITY - 1, NULL);
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

    if (xQueueSend(s_notify_queue, event, 0) != pdTRUE) {
        ESP_LOGW(TAG, "notify queue full — event type %d dropped", event->type);
    }
}
