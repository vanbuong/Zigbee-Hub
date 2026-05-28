#include "zb_framework.h"
#include "zb_network.h"
#include "zb_types.h"
#include "zb_storage.h"
#include "zb_device_mgr.h"
#include "zb_subscribe.h"
#include "zb_cmd.h"
#include "zb_button.h"
#include "znp_transport.h"
#include "znp_mt_protocol.h"
#include "znp_types.h"

#include <string.h>
#include "esp_log.h"
#include "esp_app_desc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "zb_framework";

#define MAX_INIT_RETRIES   5
#define STARTUP_TIMEOUT_MS 10000

/* ---- Module state ---- */

static zb_config_t   s_cfg;
static zb_state_t    s_state = ZB_STATE_UNINITIALIZED;
static zb_versions_t s_versions;
static bool          s_initialized = false;

/* Network info state (FR-11) — written by framework task, read-only elsewhere */
static volatile uint16_t   s_active_pan_id          = 0;
static volatile uint8_t    s_active_channel          = 0;
/* permit_join deadline in FreeRTOS ticks: 0=closed, UINT32_MAX=indefinite */
static volatile TickType_t s_permit_join_deadline    = 0;

/* ---- State helpers ---- */

static void set_state(zb_state_t new_state)
{
    static const char *names[] = {
        "UNINITIALIZED", "ZNP_INIT", "NETWORK_CHECK",
        "FORMING", "RECONFIGURING", "READY", "FW_UPDATE",
    };
    ESP_LOGD(TAG, "state: %s → %s",
             names[s_state < 7 ? s_state : 0],
             names[new_state < 7 ? new_state : 0]);
    s_state = new_state;
}

/* ---- Storage helpers ---- */

static void load_net_config(uint16_t *pan_id, uint8_t *channel, uint8_t nwk_key[16])
{
    /* Start from compile-time / runtime defaults */
    *pan_id  = s_cfg.pan_id  ? s_cfg.pan_id  : CONFIG_ZB_DEFAULT_PAN_ID;
    *channel = s_cfg.channel ? s_cfg.channel : CONFIG_ZB_DEFAULT_CHANNEL;
    memcpy(nwk_key, s_cfg.nwk_key, 16);

    /* Override with persisted values when available */
    zb_net_config_t stored = {0};
    if (zb_storage_config_load(&stored) == ESP_OK) {
        if (stored.pan_id)  *pan_id  = stored.pan_id;
        if (stored.channel) *channel = stored.channel;

        /* Only override key if it's not all-zeros */
        bool key_set = false;
        for (int i = 0; i < 16 && !key_set; i++) {
            key_set = stored.nwk_key[i] != 0;
        }
        if (key_set) {
            memcpy(nwk_key, stored.nwk_key, 16);
        }
    }

    /* Cache for zb_network_info_get() — read-only after this point until
     * the next reconfigure cycle (FR-11.2) */
    s_active_pan_id  = *pan_id;
    s_active_channel = *channel;
}

static void save_net_config(uint16_t pan_id, uint8_t channel, const uint8_t nwk_key[16])
{
    zb_net_config_t to_save = {
        .pan_id    = pan_id,
        .channel   = channel,
        .uart_baud = s_cfg.uart_baud,
    };
    memcpy(to_save.nwk_key, nwk_key, 16);

    esp_err_t err = zb_storage_config_save(&to_save);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "config save failed: %s", esp_err_to_name(err));
    }
}


/* ---- State handlers ---- */

static esp_err_t handle_znp_init(void)
{
    set_state(ZB_STATE_ZNP_INIT);

    esp_err_t err = znp_transport_reset_coprocessor();
    if (err != ESP_OK) {
        return err;
    }

    err = zb_net_ping();
    if (err != ESP_OK) {
        return err;
    }

    err = zb_net_get_version(s_versions.cc26xx_fw, sizeof(s_versions.cc26xx_fw));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SYS_VERSION failed — continuing without version info");
    }

    /* Register coordinator AF endpoint so the stack delivers AF_INCOMING_MSG AREQs */
    err = zb_net_af_register(1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "AF_REGISTER failed — device attribute reports won't be received");
    }

    return ESP_OK;
}

static esp_err_t handle_network_check(uint16_t *pan_id_out, uint8_t *channel_out,
                                       uint8_t nwk_key_out[16])
{
    set_state(ZB_STATE_NETWORK_CHECK);

    load_net_config(pan_id_out, channel_out, nwk_key_out);

    ESP_LOGI(TAG, "desired: pan_id=0x%04x channel=%d", *pan_id_out, *channel_out);

    esp_err_t nv_status = zb_net_check_nv_state(*pan_id_out, *channel_out);

    if (nv_status == ESP_OK) {
        /* NV empty → need to form */
        return ESP_ERR_NOT_FOUND;
    } else if (nv_status == ESP_ERR_NOT_FINISHED) {
        /* NV matches → resume */
        return ESP_OK;
    } else {
        /* NV mismatch → reconfigure */
        return ESP_ERR_INVALID_STATE;
    }
}

static esp_err_t handle_forming(uint16_t pan_id, uint8_t channel,
                                  const uint8_t nwk_key[16])
{
    set_state(ZB_STATE_FORMING);

    esp_err_t err = zb_net_form_network(pan_id, channel, nwk_key);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "zb_net_form_network failed: %s", esp_err_to_name(err));
        return err;
    }

    err = zb_net_startup_and_wait(STARTUP_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "coordinator failed to start: %s", esp_err_to_name(err));
        return err;
    }

    save_net_config(pan_id, channel, nwk_key);
    return ESP_OK;
}

static esp_err_t handle_reconfiguring(uint16_t pan_id, uint8_t channel,
                                        const uint8_t nwk_key[16])
{
    set_state(ZB_STATE_RECONFIGURING);
    ESP_LOGI(TAG, "reconfiguring: clearing CC2652P7 NV, re-forming network");

    /* Clear network NV items so formation starts fresh */
    uint8_t zeros[16] = {0};
    zb_net_write_nv(ZCD_NV_PANID,    zeros, 2);
    zb_net_write_nv(ZCD_NV_CHANLIST, zeros, 4);
    zb_net_write_nv(ZCD_NV_PRECFGKEY, zeros, 16);

    /* Re-form with new parameters */
    return handle_forming(pan_id, channel, nwk_key);
}

/* Process one command from the queue; returns true if re-init is needed */
static bool process_cmd(const zb_cmd_t *cmd)
{
    switch (cmd->type) {
    case ZB_CMD_PERMIT_JOIN: {
        uint8_t dur = cmd->params.duration_s;
        zb_net_permit_join(dur);
        /* Update permit-join deadline for zb_network_info_get() (FR-11.6) */
        if (dur == 0) {
            s_permit_join_deadline = 0;
        } else if (dur == 0xFF) {
            s_permit_join_deadline = (TickType_t)UINT32_MAX;
        } else {
            TickType_t dl = xTaskGetTickCount() + pdMS_TO_TICKS((uint32_t)dur * 1000);
            /* Avoid the UINT32_MAX sentinel value */
            if (dl == (TickType_t)UINT32_MAX) dl--;
            s_permit_join_deadline = dl;
        }
        break;
    }
    case ZB_CMD_CHANGE_CHANNEL:
        /* Save new channel to config.json; re-init will detect mismatch → RECONFIGURING */
        {
            zb_net_config_t cfg = {0};
            zb_storage_config_load(&cfg);
            cfg.channel = cmd->params.channel;
            zb_storage_config_save(&cfg);
        }
        return true;  /* trigger re-init */
    case ZB_CMD_RECONFIGURE:
        return true;  /* trigger re-init; new params already saved to config.json */
    }
    return false;
}

static void handle_ready(bool *should_reinit)
{
    set_state(ZB_STATE_READY);
    ESP_LOGI(TAG, "network ready — coordinator operational");

    /* Notify subscribers that network is ready */
    zb_event_emit(&(zb_event_t){ .type = ZB_EVENT_NETWORK_READY });

    znp_frame_t areq;
    for (;;) {
        /* Drain command queue before blocking on next AREQ */
        zb_cmd_t cmd;
        while (zb_cmd_dequeue(&cmd, 0) == pdTRUE) {
            if (process_cmd(&cmd)) {
                s_permit_join_deadline = 0;
                zb_event_emit(&(zb_event_t){ .type = ZB_EVENT_NETWORK_LOST });
                *should_reinit = true;
                return;
            }
        }

        /* Wait for next AREQ (100 ms timeout so command queue is checked regularly) */
        if (znp_transport_receive_areq(&areq, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        if (areq.cmd_type == ZNP_SUBSYS_SYS_AREQ && areq.cmd_id == SYS_RESET_IND_CMD) {
            ESP_LOGW(TAG, "unexpected SYS_RESET_IND — re-initializing");
            s_permit_join_deadline = 0;
            zb_event_emit(&(zb_event_t){ .type = ZB_EVENT_NETWORK_LOST });
            *should_reinit = true;
            return;
        }

        zb_dev_mgr_on_areq(&areq);
    }
}

/* ---- Framework task ---- */

static void framework_task(void *arg)
{
    int  retry     = 0;
    bool reinit    = false;
    uint16_t pan_id  = 0;
    uint8_t  channel = 0;
    uint8_t  nwk_key[16];

    for (;;) {
        esp_err_t err;

        /* --- ZNP_INIT --- */
        err = handle_znp_init();
        if (err != ESP_OK) {
            if (++retry > MAX_INIT_RETRIES) {
                ESP_LOGE(TAG, "ZNP_INIT failed after %d retries — halting", MAX_INIT_RETRIES);
                vTaskSuspend(NULL);
            }
            ESP_LOGW(TAG, "ZNP_INIT failed (%s), retry %d/%d in 1 s",
                     esp_err_to_name(err), retry, MAX_INIT_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        retry = 0;

        /* --- NETWORK_CHECK --- */
        err = handle_network_check(&pan_id, &channel, nwk_key);

        if (err == ESP_ERR_NOT_FOUND) {
            /* NV empty → FORMING */
            err = handle_forming(pan_id, channel, nwk_key);

        } else if (err == ESP_ERR_INVALID_STATE) {
            /* NV mismatch → RECONFIGURING → FORMING */
            err = handle_reconfiguring(pan_id, channel, nwk_key);

        } else if (err == ESP_OK) {
            /* NV matches → startup coordinator directly */
            set_state(ZB_STATE_FORMING); /* startup still needed */
            err = zb_net_startup_and_wait(STARTUP_TIMEOUT_MS);
        }

        if (err != ESP_OK) {
            if (++retry > MAX_INIT_RETRIES) {
                ESP_LOGE(TAG, "network bring-up failed after %d retries — halting",
                         MAX_INIT_RETRIES);
                vTaskSuspend(NULL);
            }
            ESP_LOGW(TAG, "network bring-up failed (%s), retry %d/%d in 2 s",
                     esp_err_to_name(err), retry, MAX_INIT_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        retry = 0;

        /* --- READY --- */
        reinit = false;
        handle_ready(&reinit);

        if (reinit) {
            /* Coprocessor reset during READY → restart from ZNP_INIT */
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

/* ---- Network info & param API (FR-11) ---- */

esp_err_t zb_network_info_get(zb_network_info_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    out->fw_state     = s_state;
    out->pan_id       = s_active_pan_id;
    out->channel      = s_active_channel;
    out->device_count = (uint16_t)zb_dev_mgr_count();

    /* Compute permit_join_ttl from stored deadline (FR-11.6) */
    TickType_t dl = s_permit_join_deadline;
    if (dl == 0) {
        out->permit_join_ttl = 0;
    } else if (dl == (TickType_t)UINT32_MAX) {
        out->permit_join_ttl = 0xFF;
    } else {
        int32_t remaining = (int32_t)(dl - xTaskGetTickCount());
        if (remaining <= 0) {
            out->permit_join_ttl = 0;
        } else {
            uint32_t secs = (uint32_t)remaining / configTICK_RATE_HZ;
            out->permit_join_ttl = (secs > 254) ? 254 : (uint8_t)secs;
        }
    }

    /* Derive high-level status (FR-11.3) */
    switch (out->fw_state) {
    case ZB_STATE_UNINITIALIZED:
    case ZB_STATE_ZNP_INIT:
        out->status = ZB_NET_STATUS_OFFLINE;
        break;
    case ZB_STATE_NETWORK_CHECK:
    case ZB_STATE_FORMING:
    case ZB_STATE_RECONFIGURING:
        out->status = ZB_NET_STATUS_FORMING;
        break;
    case ZB_STATE_READY:
        out->status = (out->permit_join_ttl > 0)
                      ? ZB_NET_STATUS_PERMIT_JOIN
                      : ZB_NET_STATUS_READY;
        break;
    case ZB_STATE_FW_UPDATE:
        out->status = ZB_NET_STATUS_FW_UPDATE;
        break;
    default:
        out->status = ZB_NET_STATUS_OFFLINE;
        break;
    }

    return ESP_OK;
}

esp_err_t zb_network_param_set(const zb_net_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    esp_err_t err = zb_storage_config_save(cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "zb_network_param_set: config save failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    /* If READY and PAN ID or channel changed, trigger RECONFIGURING (FR-11.4) */
    if (s_state == ZB_STATE_READY &&
        (cfg->pan_id != s_active_pan_id || cfg->channel != s_active_channel)) {
        ESP_LOGI(TAG, "network params changed (pan_id=0x%04x ch=%d) — triggering reconfigure",
                 cfg->pan_id, cfg->channel);
        return zb_cmd_reconfigure();
    }

    return ESP_OK;
}

/* ---- Default button handler (FR-10.6) ---- */

static void default_button_handler(const zb_event_t *e, void *ctx)
{
    (void)ctx;
    if (e->data.button.event == ZB_BTN_SINGLE && s_state == ZB_STATE_READY) {
        zb_cmd_permit_join(CONFIG_ZB_BUTTON_PERMIT_JOIN_SECONDS);
    }
}

/* ---- Public API ---- */

esp_err_t zb_framework_init(const zb_config_t *cfg)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_cfg = *cfg;

    /* Populate ESP32 firmware version */
    const esp_app_desc_t *app_desc = esp_app_get_description();
    if (app_desc) {
        strncpy(s_versions.esp32_fw, app_desc->version, sizeof(s_versions.esp32_fw) - 1);
    }

    /* Initialize LittleFS storage */
    esp_err_t storage_err = zb_storage_init();
    if (storage_err != ESP_OK) {
        ESP_LOGE(TAG, "storage init failed: %s", esp_err_to_name(storage_err));
        return storage_err;
    }

    /* Initialize command queue */
    esp_err_t cmd_err = zb_cmd_init();
    if (cmd_err != ESP_OK) {
        ESP_LOGE(TAG, "cmd queue init failed: %s", esp_err_to_name(cmd_err));
        return cmd_err;
    }

    /* Initialize device manager (loads schemas + restores registry from LittleFS) */
    esp_err_t dm_err = zb_dev_mgr_init();
    if (dm_err != ESP_OK) {
        ESP_LOGE(TAG, "device manager init failed: %s", esp_err_to_name(dm_err));
        return dm_err;
    }

    /* Initialize button GPIO */
    esp_err_t btn_err = zb_button_init();
    if (btn_err != ESP_OK) {
        ESP_LOGW(TAG, "button init failed: %s — button events disabled",
                 esp_err_to_name(btn_err));
    }

    /* Initialize ZNP transport */
    znp_transport_config_t transport_cfg = {
        .uart_port        = cfg->uart_port,
        .baud_rate        = (int)cfg->uart_baud,
        .gpio_tx          = cfg->gpio_tx,
        .gpio_rx          = cfg->gpio_rx,
        .pca_sda          = cfg->pca_sda,
        .pca_scl          = cfg->pca_scl,
        .pca_i2c_port     = cfg->pca_i2c_port,
        .pca_addr         = cfg->pca_addr,
        .pca_reset_bit    = cfg->pca_reset_bit,
        .pca_bsl_bit      = cfg->pca_bsl_bit,
        .pca_rst_gpio     = cfg->pca_rst_gpio,
        .sreq_timeout_ms  = 1000,
        .reset_timeout_ms = 6000,
    };
    esp_err_t err = znp_transport_init(&transport_cfg);
    if (err != ESP_OK) {
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "initialized (esp32_fw=%s)", s_versions.esp32_fw);
    return ESP_OK;
}

esp_err_t zb_framework_start(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Start button polling task and register default single-press → permit-join handler */
    esp_err_t btn_err = zb_button_start();
    if (btn_err != ESP_OK) {
        ESP_LOGW(TAG, "button task start failed: %s", esp_err_to_name(btn_err));
    } else {
        zb_subscribe(ZB_EVENT_BUTTON, default_button_handler, NULL);
    }

    BaseType_t ret = xTaskCreate(framework_task, "zb_framework",
                                  CONFIG_ZB_FRAMEWORK_TASK_STACK,
                                  NULL, CONFIG_ZB_FRAMEWORK_TASK_PRIORITY, NULL);
    return (ret == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM;
}

zb_state_t zb_framework_get_state(void)
{
    return s_state;
}

esp_err_t zb_framework_get_versions(zb_versions_t *out)
{
    if (!out) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = s_versions;
    return ESP_OK;
}
