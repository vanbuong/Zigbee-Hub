#include "zb_network.h"
#include "zb_types.h"
#include "znp_transport.h"
#include "znp_mt_protocol.h"
#include "znp_types.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "zb_network";

/* ---- Helpers ---- */

/* Build a SREQ frame and send it, return SRSP in resp */
static esp_err_t send_sreq(uint8_t subsys, uint8_t cmd_id, uint8_t frame_type,
                            const uint8_t *payload, uint8_t payload_len,
                            znp_frame_t *resp)
{
    znp_frame_t req = {
        .cmd_type    = ZNP_CMD_TYPE(subsys, frame_type),
        .cmd_id      = cmd_id,
        .payload_len = payload_len,
    };
    if (payload && payload_len > 0) {
        memcpy(req.payload, payload, payload_len);
    }
    return znp_transport_send_sreq(&req, resp);
}

/* ---- SYS commands ---- */

esp_err_t zb_net_ping(void)
{
    znp_frame_t resp;
    esp_err_t err = send_sreq(ZNP_SUBSYS_SYS, SYS_PING_CMD,
                               ZNP_FRAME_TYPE_SREQ, NULL, 0, &resp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SYS_PING failed: %s", esp_err_to_name(err));
        return err;
    }
    /* SRSP payload[0..1] = capabilities word */
    ESP_LOGI(TAG, "SYS_PING OK capabilities=0x%02x%02x",
             resp.payload[1], resp.payload[0]);
    return ESP_OK;
}

esp_err_t zb_net_get_version(char *out, size_t len)
{
    znp_frame_t resp;
    esp_err_t err = send_sreq(ZNP_SUBSYS_SYS, SYS_VERSION_CMD,
                               ZNP_FRAME_TYPE_SREQ, NULL, 0, &resp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SYS_VERSION failed: %s", esp_err_to_name(err));
        return err;
    }
    /*
     * SYS_VERSION SRSP layout (Z-Stack 3.x):
     *   [0]   Transport rev
     *   [1]   Product ID
     *   [2]   Major
     *   [3]   Minor
     *   [4]   HW rev
     */
    if (out && len > 0) {
        snprintf(out, len, "Z-Stack %d.%d (TransRev=%d HW=%d)",
                 resp.payload[2], resp.payload[3],
                 resp.payload[0], resp.payload[4]);
    }
    ESP_LOGI(TAG, "CC2652P7 firmware: %s", out ? out : "(not stored)");
    return ESP_OK;
}

/* ---- NV operations ---- */

esp_err_t zb_net_read_nv(uint16_t id, uint8_t *buf, uint8_t len)
{
    /* OsalNvRead SREQ payload: ID(2) + offset(1) + len(1) */
    uint8_t payload[4] = {
        (uint8_t)(id & 0xFF),
        (uint8_t)(id >> 8),
        0,      /* offset */
        len,
    };
    znp_frame_t resp;
    esp_err_t err = send_sreq(ZNP_SUBSYS_SYS, SYS_OSAL_NV_READ_CMD,
                               ZNP_FRAME_TYPE_SREQ, payload, sizeof(payload), &resp);
    if (err != ESP_OK) {
        return err;
    }
    /* SRSP: status(1) + len(1) + data(n) */
    if (resp.payload[0] != 0x00) {
        ESP_LOGD(TAG, "NV read id=0x%04x status=0x%02x (item may not exist)",
                 id, resp.payload[0]);
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t rlen = resp.payload[1];
    if (buf && rlen > 0) {
        memcpy(buf, &resp.payload[2], (rlen < len) ? rlen : len);
    }
    return ESP_OK;
}

esp_err_t zb_net_write_nv(uint16_t id, const uint8_t *buf, uint8_t len)
{
    /* OsalNvWrite SREQ payload: ID(2) + offset(1) + len(1) + data(n) */
    uint8_t payload[4 + len];
    payload[0] = (uint8_t)(id & 0xFF);
    payload[1] = (uint8_t)(id >> 8);
    payload[2] = 0;        /* offset */
    payload[3] = len;
    memcpy(&payload[4], buf, len);

    znp_frame_t resp;
    esp_err_t err = send_sreq(ZNP_SUBSYS_SYS, SYS_OSAL_NV_WRITE_CMD,
                               ZNP_FRAME_TYPE_SREQ, payload, sizeof(payload), &resp);
    if (err != ESP_OK) {
        return err;
    }
    if (resp.payload[0] != 0x00) {
        ESP_LOGE(TAG, "NV write id=0x%04x failed status=0x%02x",
                 id, resp.payload[0]);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ---- Network state check ---- */

esp_err_t zb_net_check_nv_state(uint16_t desired_pan_id, uint8_t desired_channel)
{
    uint8_t nv_pan_id[2]   = {0};
    uint8_t nv_chanlist[4] = {0};

    esp_err_t err_pan  = zb_net_read_nv(ZCD_NV_PANID,   nv_pan_id,   sizeof(nv_pan_id));
    esp_err_t err_chan = zb_net_read_nv(ZCD_NV_CHANLIST, nv_chanlist, sizeof(nv_chanlist));

    if (err_pan == ESP_ERR_NOT_FOUND || err_chan == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "CC2652P7 NV empty — needs network formation");
        return ESP_OK;  /* caller should FORM */
    }

    uint16_t stored_pan_id  = (uint16_t)nv_pan_id[0] | ((uint16_t)nv_pan_id[1] << 8);
    /* chanlist is a 32-bit bitmask; extract channel from it */
    uint32_t chanlist = (uint32_t)nv_chanlist[0]
                      | ((uint32_t)nv_chanlist[1] << 8)
                      | ((uint32_t)nv_chanlist[2] << 16)
                      | ((uint32_t)nv_chanlist[3] << 24);
    uint8_t stored_channel = 0;
    for (uint8_t c = 11; c <= 26; c++) {
        if (chanlist & (1UL << c)) {
            stored_channel = c;
            break;
        }
    }

    ESP_LOGI(TAG, "CC2652P7 NV: pan_id=0x%04x channel=%d | desired: pan_id=0x%04x channel=%d",
             stored_pan_id, stored_channel, desired_pan_id, desired_channel);

    if (stored_pan_id == desired_pan_id && stored_channel == desired_channel) {
        return ESP_ERR_NOT_FINISHED;  /* params match → can resume */
    }
    return ESP_ERR_NOT_FOUND;           /* mismatch → needs reconfiguring */
}

/* ---- Network formation ---- */

esp_err_t zb_net_form_network(uint16_t pan_id, uint8_t channel,
                               const uint8_t nwk_key[16])
{
    esp_err_t err;

    /* Set logical device type = coordinator (0x00) */
    uint8_t dev_type = ZB_DEVICE_TYPE_COORDINATOR;
    err = zb_net_write_nv(ZCD_NV_LOGICAL_TYPE, &dev_type, 1);
    if (err != ESP_OK) {
        return err;
    }

    /* Set PAN ID */
    uint8_t pan_bytes[2] = { (uint8_t)(pan_id & 0xFF), (uint8_t)(pan_id >> 8) };
    err = zb_net_write_nv(ZCD_NV_PANID, pan_bytes, sizeof(pan_bytes));
    if (err != ESP_OK) {
        return err;
    }

    /* Set channel list bitmask */
    uint32_t chanlist = (1UL << channel);
    uint8_t chan_bytes[4] = {
        (uint8_t)(chanlist & 0xFF),
        (uint8_t)((chanlist >> 8) & 0xFF),
        (uint8_t)((chanlist >> 16) & 0xFF),
        (uint8_t)((chanlist >> 24) & 0xFF),
    };
    err = zb_net_write_nv(ZCD_NV_CHANLIST, chan_bytes, sizeof(chan_bytes));
    if (err != ESP_OK) {
        return err;
    }

    /* Enable pre-configured network key */
    uint8_t precfg_en = 0x01;
    err = zb_net_write_nv(ZCD_NV_PRECFGKEYS_EN, &precfg_en, 1);
    if (err != ESP_OK) {
        return err;
    }

    /* Write network key */
    err = zb_net_write_nv(ZCD_NV_PRECFGKEY, nwk_key, 16);
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "NV written: pan_id=0x%04x channel=%d", pan_id, channel);
    return ESP_OK;
}

esp_err_t zb_net_permit_join(uint8_t duration_s)
{
    /* ZDO_MGMT_PERMIT_JOIN_REQ: addrMode[1], dstAddr[2], duration[1], tcSignificance[1] */
    uint8_t payload[5] = {
        0x0F,          /* broadcast address mode */
        0xFC, 0xFF,    /* 0xFFFC = all routers + coordinator */
        duration_s,
        0x01,          /* TC significance */
    };
    znp_frame_t resp;
    esp_err_t err = send_sreq(ZNP_SUBSYS_ZDO, ZDO_MGMT_PERMIT_JOIN_CMD,
                               ZNP_FRAME_TYPE_SREQ, payload, sizeof(payload), &resp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "permit_join failed: %s", esp_err_to_name(err));
        return err;
    }
    if (resp.payload[0] != 0x00) {
        ESP_LOGE(TAG, "permit_join status=0x%02x", resp.payload[0]);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "permit_join: %d s", duration_s);
    return ESP_OK;
}

esp_err_t zb_net_af_register(uint8_t ep)
{
    /*
     * AF_REGISTER SREQ payload:
     * endpoint[1], profile_id[2], device_id[2], device_ver[1],
     * latency[1], num_in_clusters[1], num_out_clusters[1]
     * (0 clusters — coordinator accepts all incoming messages regardless)
     */
    uint8_t payload[] = {
        ep,
        0x04, 0x01,   /* profile = 0x0104 (Home Automation) */
        0x05, 0x00,   /* device  = 0x0005 (Configuration Tool) */
        0x00,         /* device version */
        0x00,         /* latency */
        0x00,         /* num in clusters */
        0x00,         /* num out clusters */
    };
    znp_frame_t resp;
    esp_err_t err = send_sreq(ZNP_SUBSYS_AF, AF_REGISTER_CMD,
                               ZNP_FRAME_TYPE_SREQ, payload, sizeof(payload), &resp);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AF_REGISTER failed: %s", esp_err_to_name(err));
        return err;
    }
    if (resp.payload[0] != 0x00) {
        ESP_LOGE(TAG, "AF_REGISTER status=0x%02x", resp.payload[0]);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "AF endpoint %d registered", ep);
    return ESP_OK;
}

#define ZDO_STARTUP_FROM_APP_SREQ_TIMEOUT_MS  10000

esp_err_t zb_net_startup_and_wait(uint32_t timeout_ms)
{
    /* ZDO_STARTUP_FROM_APP SREQ: payload = start delay (2 bytes, typically 0).
     * In Z-Stack 2.x the coprocessor doesn't return SRSP until commissioning /
     * scan completes, so use a 10 s per-call timeout instead of the default. */
    uint8_t payload[2] = {0, 0};
    znp_frame_t req = {
        .cmd_type    = ZNP_CMD_TYPE(ZNP_SUBSYS_ZDO, ZNP_FRAME_TYPE_SREQ),
        .cmd_id      = ZDO_STARTUP_FROM_APP_CMD,
        .payload_len = sizeof(payload),
    };
    memcpy(req.payload, payload, sizeof(payload));

    znp_frame_t resp;
    esp_err_t err = znp_transport_send_sreq_timeout(
        &req, &resp, ZDO_STARTUP_FROM_APP_SREQ_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ZDO_STARTUP_FROM_APP SREQ failed");
        return err;
    }
    ESP_LOGI(TAG, "ZDO_STARTUP_FROM_APP SRSP status=0x%02x", resp.payload[0]);

    /* Wait for ZDO_STATE_CHANGE_IND AREQ with DEV_ZB_COORD */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    znp_frame_t areq;
    while (xTaskGetTickCount() < deadline) {
        TickType_t remaining = deadline - xTaskGetTickCount();
        if (znp_transport_receive_areq(&areq, remaining) != pdTRUE) {
            break;
        }
        if (areq.cmd_id == ZDO_STATE_CHANGE_IND_CMD) {
            uint8_t dev_state = areq.payload[0];
            ESP_LOGI(TAG, "ZDO_STATE_CHANGE_IND state=0x%02x", dev_state);
            if (dev_state == DEV_ZB_COORD) {
                ESP_LOGI(TAG, "Coordinator started successfully");
                return ESP_OK;
            }
        }
        /* Re-post non-coordinator AREQs — they'll be processed later by framework */
    }

    ESP_LOGE(TAG, "Coordinator did not start within %"PRIu32" ms", timeout_ms);
    return ESP_ERR_TIMEOUT;
}
