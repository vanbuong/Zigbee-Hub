#include "zb_button.h"
#include "zb_subscribe.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "zb_btn";

/* ---- Internal FSM ---- */

typedef enum {
    BTN_IDLE = 0,
    BTN_DEBOUNCE_PRESS,    /* pin went active; waiting for stable window */
    BTN_HELD,              /* press confirmed; watching for long-press and release */
    BTN_DEBOUNCE_RELEASE,  /* pin went inactive; waiting for stable window */
    BTN_INTER_PRESS,       /* released; waiting for next press or multi-press timeout */
} btn_fsm_t;

static bool read_pressed(void)
{
    int level = gpio_get_level(CONFIG_ZB_BUTTON_GPIO);
#if CONFIG_ZB_BUTTON_ACTIVE_LOW
    return level == 0;
#else
    return level == 1;
#endif
}

static void emit_button(zb_button_event_kind_t kind)
{
    static const char *names[] = { "SINGLE", "DOUBLE", "LONG", "TRIPLE", "QUAD", "VERY_LONG" };
    ESP_LOGI(TAG, "button %s", (kind < 6) ? names[kind] : "?");

    zb_event_t ev = {
        .type              = ZB_EVENT_BUTTON,
        .data.button.event = kind,
    };
    zb_event_emit(&ev);
}

static void button_task(void *arg)
{
    btn_fsm_t  state       = BTN_IDLE;
    TickType_t state_enter = 0;
    int        press_count = 0;
    bool       long_fired  = false;

    const TickType_t debounce_ticks    = pdMS_TO_TICKS(CONFIG_ZB_BUTTON_DEBOUNCE_MS);
    const TickType_t long_press_ticks  = pdMS_TO_TICKS(CONFIG_ZB_BUTTON_LONG_PRESS_MS);
    const TickType_t multi_press_ticks = pdMS_TO_TICKS(CONFIG_ZB_BUTTON_MULTI_PRESS_TIMEOUT_MS);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(CONFIG_ZB_BUTTON_POLL_INTERVAL_MS));

        TickType_t now      = xTaskGetTickCount();
        bool       pressed  = read_pressed();
        TickType_t in_state = now - state_enter;

        switch (state) {
        case BTN_IDLE:
            if (pressed) {
                state       = BTN_DEBOUNCE_PRESS;
                state_enter = now;
            }
            break;

        case BTN_DEBOUNCE_PRESS:
            if (!pressed) {
                /* bounced back before debounce window — treat as noise */
                state = BTN_IDLE;
            } else if (in_state >= debounce_ticks) {
                state       = BTN_HELD;
                state_enter = now;
                long_fired  = false;
            }
            break;

        case BTN_HELD:
            if (!long_fired && in_state >= long_press_ticks) {
                emit_button(ZB_BTN_LONG);
                long_fired = true;
            }
            if (!pressed) {
                state       = BTN_DEBOUNCE_RELEASE;
                state_enter = now;
            }
            break;

        case BTN_DEBOUNCE_RELEASE:
            if (pressed) {
                /* pin bounced active again — return to held */
                state       = BTN_HELD;
                state_enter = now;
            } else if (in_state >= debounce_ticks) {
                if (long_fired) {
                    /* long-press already emitted; discard any partial short-press count */
                    press_count = 0;
                    long_fired  = false;
                    state       = BTN_IDLE;
                } else {
                    press_count++;
                    state       = BTN_INTER_PRESS;
                    state_enter = now;
                }
            }
            break;

        case BTN_INTER_PRESS:
            if (pressed) {
                /* another press started within the multi-press window */
                state       = BTN_DEBOUNCE_PRESS;
                state_enter = now;
            } else if (in_state >= multi_press_ticks) {
                /* timeout expired — emit based on press_count */
                zb_button_event_kind_t kind;
                switch (press_count) {
                case 1:  kind = ZB_BTN_SINGLE; break;
                case 2:  kind = ZB_BTN_DOUBLE; break;
                default: kind = ZB_BTN_SINGLE; break;
                }
                emit_button(kind);
                press_count = 0;
                state       = BTN_IDLE;
            }
            break;
        }
    }
}

/* ---- Public API ---- */

esp_err_t zb_button_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CONFIG_ZB_BUTTON_GPIO,
        .mode         = GPIO_MODE_INPUT,
        /* External pull-up/pull-down assumed on PCB — disable internal resistors */
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config GPIO%d failed: %s",
                 CONFIG_ZB_BUTTON_GPIO, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "GPIO%d configured (active-%s, debounce=%d ms, long=%d ms, mp=%d ms)",
                 CONFIG_ZB_BUTTON_GPIO,
                 CONFIG_ZB_BUTTON_ACTIVE_LOW ? "low" : "high",
                 CONFIG_ZB_BUTTON_DEBOUNCE_MS,
                 CONFIG_ZB_BUTTON_LONG_PRESS_MS,
                 CONFIG_ZB_BUTTON_MULTI_PRESS_TIMEOUT_MS);
    }
    return err;
}

esp_err_t zb_button_start(void)
{
    BaseType_t ret = xTaskCreate(button_task, "zb_btn",
                                  CONFIG_ZB_BUTTON_TASK_STACK, NULL,
                                  CONFIG_ZB_BUTTON_TASK_PRIORITY, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create button task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
