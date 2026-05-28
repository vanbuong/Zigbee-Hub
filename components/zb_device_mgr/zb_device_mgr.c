#include "zb_device_mgr.h"
#include "zb_cap.h"
#include "zb_storage.h"
#include "zb_subscribe.h"
#include "znp_transport.h"
#include "znp_mt_protocol.h"
#include "znp_types.h"

#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "zb_dev_mgr";

/* ---- Device registry ---- */

static zb_dev_t          s_devices[ZB_DEV_MAX_DEVICES];
static SemaphoreHandle_t s_mutex;

/* ---- ZCL attribute report command ---- */
#define ZCL_CMD_REPORT_ATTRIBUTES   0x0A

/* ---- Internal: declare zb_subscribe_init ---- */
void zb_subscribe_init(void);

/* ---- Internal: ZDO discovery helpers ---- */

static esp_err_t send_zdo_active_ep_req(uint16_t nwk_addr)
{
    uint8_t p[4] = { nwk_addr & 0xFF, nwk_addr >> 8,
                     nwk_addr & 0xFF, nwk_addr >> 8 };
    znp_frame_t req = {
        .cmd_type    = ZNP_CMD_TYPE(ZNP_SUBSYS_ZDO, ZNP_FRAME_TYPE_SREQ),
        .cmd_id      = ZDO_ACTIVE_EP_REQ_CMD,
        .payload_len = 4,
    };
    memcpy(req.payload, p, 4);
    znp_frame_t resp;
    return znp_transport_send_sreq(&req, &resp);
}

static esp_err_t send_zdo_simple_desc_req(uint16_t nwk_addr, uint8_t ep)
{
    uint8_t p[5] = { nwk_addr & 0xFF, nwk_addr >> 8,
                     nwk_addr & 0xFF, nwk_addr >> 8, ep };
    znp_frame_t req = {
        .cmd_type    = ZNP_CMD_TYPE(ZNP_SUBSYS_ZDO, ZNP_FRAME_TYPE_SREQ),
        .cmd_id      = ZDO_SIMPLE_DESC_REQ_CMD,
        .payload_len = 5,
    };
    memcpy(req.payload, p, 5);
    znp_frame_t resp;
    return znp_transport_send_sreq(&req, &resp);
}

/* ---- Helpers ---- */

static zb_dev_t *find_by_ieee(uint64_t ieee)
{
    for (int i = 0; i < ZB_DEV_MAX_DEVICES; i++) {
        if (s_devices[i].active && s_devices[i].ieee_addr == ieee)
            return &s_devices[i];
    }
    return NULL;
}

static zb_dev_t *find_by_nwk(uint16_t nwk)
{
    for (int i = 0; i < ZB_DEV_MAX_DEVICES; i++) {
        if (s_devices[i].active && s_devices[i].nwk_addr == nwk)
            return &s_devices[i];
    }
    return NULL;
}

static zb_dev_t *find_free_slot(void)
{
    for (int i = 0; i < ZB_DEV_MAX_DEVICES; i++) {
        if (!s_devices[i].active) return &s_devices[i];
    }
    return NULL;
}

/* ---- Storage restore callback ---- */

static void on_pb_device_loaded(const ZbDeviceRecord *pb, void *ctx)
{
    zb_dev_t *dev = find_by_ieee(pb->ieee_addr);
    if (!dev) {
        dev = find_free_slot();
        if (!dev) {
            ESP_LOGW(TAG, "registry full — skipping stored device %016llx",
                     (unsigned long long)pb->ieee_addr);
            return;
        }
    }
    dev->active    = true;
    dev->ieee_addr = pb->ieee_addr;
    dev->nwk_addr  = (uint16_t)pb->network_addr;
    dev->num_caps  = 0;

    /* Reconstruct cap list from stored endpoints/clusters */
    for (pb_size_t e = 0; e < pb->endpoints_count; e++) {
        const ZbEndpoint *ep = &pb->endpoints[e];
        for (pb_size_t c = 0; c < ep->clusters_count; c++) {
            if (dev->num_caps >= ZB_DEV_MAX_CAPS) break;
            dev->caps[dev->num_caps++] =
                ZB_CAP_ID((uint8_t)ep->ep_id, (uint16_t)ep->clusters[c].cluster_id);
        }
    }
    int *count = (int *)ctx;
    (*count)++;
}

/* ---- Public API ---- */

esp_err_t zb_dev_mgr_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    if (!s_mutex) return ESP_ERR_NO_MEM;

    memset(s_devices, 0, sizeof(s_devices));

    /* Init event bus */
    zb_subscribe_init();

    /* Init cap layer (schemas + notify task) */
    esp_err_t err = zb_cap_init();
    if (err != ESP_OK) return err;

    /* Restore device registry from LittleFS */
    int count = 0;
    zb_storage_device_load_all(on_pb_device_loaded, &count);
    ESP_LOGI(TAG, "registry restored: %d device(s)", count);
    return ESP_OK;
}

esp_err_t zb_dev_mgr_join(uint64_t ieee_addr, uint16_t nwk_addr)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    zb_dev_t *dev = find_by_ieee(ieee_addr);
    if (!dev) {
        dev = find_free_slot();
        if (!dev) {
            xSemaphoreGive(s_mutex);
            ESP_LOGW(TAG, "registry full, cannot add %016llx",
                     (unsigned long long)ieee_addr);
            return ESP_ERR_NO_MEM;
        }
    }
    dev->active    = true;
    dev->ieee_addr = ieee_addr;
    dev->nwk_addr  = nwk_addr;

    xSemaphoreGive(s_mutex);

    /* Persist (with any previously-known caps — empty for new devices) */
    ZbDeviceRecord pb = ZbDeviceRecord_init_zero;
    pb.ieee_addr    = ieee_addr;
    pb.network_addr = nwk_addr;
    zb_storage_device_save(&pb);

    ESP_LOGI(TAG, "device joined: %016llx nwk=0x%04x",
             (unsigned long long)ieee_addr, nwk_addr);

    /* Emit DEVICE_JOINED event to subscribers */
    zb_event_t ev = {
        .type = ZB_EVENT_DEVICE_JOINED,
        .data.device = { .ieee_addr = ieee_addr, .nwk_addr = nwk_addr },
    };
    zb_event_emit(&ev);

    /* Trigger ZDO endpoint discovery (SREQ only — response arrives as AREQ later) */
    send_zdo_active_ep_req(nwk_addr);

    return ESP_OK;
}

void zb_dev_mgr_leave(uint64_t ieee_addr)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    zb_dev_t *dev = find_by_ieee(ieee_addr);
    if (dev) {
        dev->active = false;
    }
    xSemaphoreGive(s_mutex);

    zb_storage_device_delete(ieee_addr);
    ESP_LOGI(TAG, "device left: %016llx", (unsigned long long)ieee_addr);

    zb_event_t ev = {
        .type = ZB_EVENT_DEVICE_LEFT,
        .data.device = { .ieee_addr = ieee_addr, .nwk_addr = 0xFFFF },
    };
    zb_event_emit(&ev);
}

const zb_dev_t *zb_dev_mgr_get(uint64_t ieee_addr)
{
    return find_by_ieee(ieee_addr);
}

uint16_t zb_dev_mgr_get_nwk(uint64_t ieee_addr)
{
    const zb_dev_t *dev = find_by_ieee(ieee_addr);
    return dev ? dev->nwk_addr : 0xFFFF;
}

esp_err_t zb_dev_mgr_learn_cap(uint64_t ieee_addr, zb_cap_id_t cap)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    zb_dev_t *dev = find_by_ieee(ieee_addr);
    if (!dev) {
        xSemaphoreGive(s_mutex);
        return ESP_ERR_NOT_FOUND;
    }

    /* Add if not already present */
    for (int i = 0; i < dev->num_caps; i++) {
        if (dev->caps[i] == cap) {
            xSemaphoreGive(s_mutex);
            return ESP_OK;
        }
    }
    if (dev->num_caps < ZB_DEV_MAX_CAPS) {
        dev->caps[dev->num_caps++] = cap;
        ESP_LOGD(TAG, "learned cap 0x%08"PRIx32" for %016llx",
                 (uint32_t)cap, (unsigned long long)ieee_addr);
    }
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t zb_dev_get_caps(uint64_t ieee_addr, zb_cap_id_t *list, uint8_t *count)
{
    const zb_dev_t *dev = find_by_ieee(ieee_addr);
    if (!dev) {
        *count = 0;
        return ESP_ERR_NOT_FOUND;
    }
    uint8_t n = dev->num_caps < *count ? dev->num_caps : *count;
    memcpy(list, dev->caps, n * sizeof(zb_cap_id_t));
    *count = n;
    return ESP_OK;
}

int zb_dev_mgr_count(void)
{
    if (!s_mutex) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int n = 0;
    for (int i = 0; i < ZB_DEV_MAX_DEVICES; i++) {
        if (s_devices[i].active) n++;
    }
    xSemaphoreGive(s_mutex);
    return n;
}

/* ---- ZCL command send ---- */

esp_err_t zb_zcl_send(uint64_t ieee, zb_cap_id_t cap,
                       const uint8_t *zcl_frame, uint8_t zcl_len)
{
    uint16_t nwk = zb_dev_mgr_get_nwk(ieee);
    if (nwk == 0xFFFF) {
        ESP_LOGW(TAG, "device not found: %016llx", (unsigned long long)ieee);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t ep      = ZB_CAP_EP(cap);
    uint16_t cluster = ZB_CAP_CLUSTER(cap);

    /* AF_DATA_REQUEST payload */
    uint8_t payload[10 + zcl_len];
    payload[0] = (uint8_t)(nwk & 0xFF);
    payload[1] = (uint8_t)(nwk >> 8);
    payload[2] = ep;
    payload[3] = 0x01;   /* source endpoint 1 (coordinator) */
    payload[4] = (uint8_t)(cluster & 0xFF);
    payload[5] = (uint8_t)(cluster >> 8);
    payload[6] = 0x01;   /* transaction ID */
    payload[7] = 0x00;   /* TX options */
    payload[8] = 0x0F;   /* radius */
    payload[9] = zcl_len;
    memcpy(&payload[10], zcl_frame, zcl_len);

    znp_frame_t req = {
        .cmd_type    = ZNP_CMD_TYPE(ZNP_SUBSYS_AF, ZNP_FRAME_TYPE_SREQ),
        .cmd_id      = AF_DATA_REQUEST_CMD,
        .payload_len = 10 + zcl_len,
    };
    memcpy(req.payload, payload, req.payload_len);

    znp_frame_t resp;
    esp_err_t err = znp_transport_send_sreq(&req, &resp);
    if (err != ESP_OK) return err;

    if (resp.payload[0] != 0x00) {
        ESP_LOGW(TAG, "AF_DATA_REQUEST status=0x%02x", resp.payload[0]);
        return ESP_FAIL;
    }
    return ESP_OK;
}

/* ---- AREQ parsing ---- */

static void parse_tc_dev_ind(const znp_frame_t *areq)
{
    /* TC_DEV_IND: nwkAddr[2] extAddr[8] action[1] secLevel[1] */
    if (areq->payload_len < 12) return;
    uint16_t nwk = (uint16_t)(areq->payload[0] | (areq->payload[1] << 8));
    uint64_t ieee = 0;
    for (int i = 0; i < 8; i++)
        ieee |= (uint64_t)areq->payload[2 + i] << (8 * i);
    zb_dev_mgr_join(ieee, nwk);
}

static void parse_end_device_annce(const znp_frame_t *areq)
{
    /* END_DEVICE_ANNCE_IND: srcAddr[2] nwkAddr[2] extAddr[8] capabilities[1] */
    if (areq->payload_len < 13) return;
    uint16_t nwk = (uint16_t)(areq->payload[2] | (areq->payload[3] << 8));
    uint64_t ieee = 0;
    for (int i = 0; i < 8; i++)
        ieee |= (uint64_t)areq->payload[4 + i] << (8 * i);
    zb_dev_mgr_join(ieee, nwk);
}

static void parse_leave_ind(const znp_frame_t *areq)
{
    /* LEAVE_IND: srcAddr[2] extAddr[8] remove[1] rejoin[1] */
    if (areq->payload_len < 12) return;
    uint64_t ieee = 0;
    for (int i = 0; i < 8; i++)
        ieee |= (uint64_t)areq->payload[2 + i] << (8 * i);
    zb_dev_mgr_leave(ieee);
}

static void parse_af_incoming(const znp_frame_t *areq)
{
    /*
     * AF_INCOMING_MSG payload:
     * group_id[2], cluster[2], src_addr[2], src_ep[1], dst_ep[1],
     * was_bcast[1], lqi[1], sec[1], timestamp[4], seqnum[1], len[1], data[len]
     */
    if (areq->payload_len < 17) return;

    const uint8_t *p   = areq->payload;
    uint16_t cluster   = (uint16_t)(p[2]  | (p[3]  << 8));
    uint16_t src_addr  = (uint16_t)(p[4]  | (p[5]  << 8));
    uint8_t  src_ep    = p[6];
    uint8_t  data_len  = p[16];

    if (areq->payload_len < (uint8_t)(17 + data_len)) return;
    const uint8_t *zcl = &p[17];

    /* Need ieee_addr from nwk_addr */
    zb_dev_t *dev = find_by_nwk(src_addr);
    if (!dev) {
        ESP_LOGD(TAG, "AF msg from unknown nwk=0x%04x", src_addr);
        return;
    }
    uint64_t ieee = dev->ieee_addr;

    /* Learn this cap */
    zb_cap_id_t cap = ZB_CAP_ID(src_ep, cluster);
    zb_dev_mgr_learn_cap(ieee, cap);

    /* Decode ZCL frame: frame_ctrl[1], seq[1], cmd[1], payload... */
    if (data_len < 3) return;
    uint8_t zcl_cmd = zcl[2];

    if (zcl_cmd == ZCL_CMD_REPORT_ATTRIBUTES) {
        /* Report Attributes: attr_id[2], dtype[1], value[n], ... */
        const uint8_t *rp        = &zcl[3];
        uint8_t        remaining = data_len - 3;

        while (remaining >= 4) {
            uint16_t attr_id = (uint16_t)(rp[0] | (rp[1] << 8));
            uint8_t  dtype   = rp[2];

            /* Look up data type size */
            uint8_t sz;
            switch (dtype) {
            case 0x10: sz = 1; break;
            case 0x20: sz = 1; break;
            case 0x21: sz = 2; break;
            case 0x22: sz = 3; break;
            case 0x23: sz = 4; break;
            case 0x28: sz = 1; break;
            case 0x29: sz = 2; break;
            case 0x2A: sz = 3; break;
            case 0x2B: sz = 4; break;
            default:   sz = 0; break;
            }
            if (sz == 0 || remaining < (uint8_t)(3 + sz)) break;

            zb_cap_dispatch_attr(ieee, src_ep, cluster, attr_id, dtype,
                                 &rp[3], sz);

            rp        += 3 + sz;
            remaining -= 3 + sz;
        }
    }
}

/* ---- ZDO discovery response parsers ---- */

static void parse_active_ep_rsp(const znp_frame_t *areq)
{
    /* ZDO_ACTIVE_EP_RSP: srcAddr[2], status[1], nwkAddr[2], epCnt[1], eps[n] */
    if (areq->payload_len < 6) return;
    if (areq->payload[2] != 0x00) return;   /* status != success */

    uint16_t nwk   = (uint16_t)(areq->payload[3] | (areq->payload[4] << 8));
    uint8_t  cnt   = areq->payload[5];
    const uint8_t *eps = &areq->payload[6];

    if (areq->payload_len < (uint8_t)(6 + cnt)) return;

    ESP_LOGD(TAG, "active EP rsp nwk=0x%04x cnt=%d", nwk, cnt);
    for (uint8_t i = 0; i < cnt; i++) {
        send_zdo_simple_desc_req(nwk, eps[i]);
    }
}

static void parse_simple_desc_rsp(const znp_frame_t *areq)
{
    /*
     * ZDO_SIMPLE_DESC_RSP: srcAddr[2], status[1], nwkAddr[2], descLen[1],
     * ep[1], profileId[2], deviceId[2], deviceVer[1],
     * numInClusters[1], inClusters[2*n], numOutClusters[1], outClusters[2*m]
     */
    if (areq->payload_len < 12 || areq->payload[2] != 0x00) return;

    uint16_t nwk = (uint16_t)(areq->payload[3] | (areq->payload[4] << 8));
    uint8_t  ep  = areq->payload[6];

    /* Lookup device */
    zb_dev_t *dev = find_by_nwk(nwk);
    if (!dev) return;
    uint64_t ieee = dev->ieee_addr;

    uint8_t num_in = areq->payload[11];
    const uint8_t *in_c = &areq->payload[12];

    /* Register in-clusters as caps */
    for (uint8_t i = 0; i < num_in; i++) {
        if (12 + i * 2 + 1 >= areq->payload_len) break;
        uint16_t cluster = (uint16_t)(in_c[i*2] | (in_c[i*2+1] << 8));
        zb_dev_mgr_learn_cap(ieee, ZB_CAP_ID(ep, cluster));
    }

    /* Also check out-clusters if present */
    uint8_t in_end = 12 + num_in * 2;
    if (in_end < areq->payload_len) {
        uint8_t num_out = areq->payload[in_end];
        const uint8_t *out_c = &areq->payload[in_end + 1];
        for (uint8_t i = 0; i < num_out; i++) {
            if (in_end + 1 + i * 2 + 1 >= areq->payload_len) break;
            uint16_t cluster = (uint16_t)(out_c[i*2] | (out_c[i*2+1] << 8));
            zb_dev_mgr_learn_cap(ieee, ZB_CAP_ID(ep, cluster));
        }
    }

    /* Persist updated caps */
    ZbDeviceRecord pb = ZbDeviceRecord_init_zero;
    pb.ieee_addr    = ieee;
    pb.network_addr = nwk;
    zb_storage_device_save(&pb);

    ESP_LOGD(TAG, "simple_desc nwk=0x%04x ep=%d: %d in-clusters", nwk, ep, num_in);
}

void zb_dev_mgr_on_areq(const znp_frame_t *areq)
{
    /* Match on full cmd_type (frame-type + subsystem), not just the subsystem
     * nibble — different subsystems can share cmd_id values. */
    if (areq->cmd_type == ZNP_SUBSYS_ZDO_AREQ) {
        switch (areq->cmd_id) {
        case ZDO_TC_DEV_IND_CMD:
            parse_tc_dev_ind(areq);
            break;
        case ZDO_END_DEVICE_ANNCE_IND_CMD:
            parse_end_device_annce(areq);
            break;
        case ZDO_LEAVE_IND_CMD:
            parse_leave_ind(areq);
            break;
        case ZDO_ACTIVE_EP_RSP_CMD:
            parse_active_ep_rsp(areq);
            break;
        case ZDO_SIMPLE_DESC_RSP_CMD:
            parse_simple_desc_rsp(areq);
            break;
        default:
            break;
        }
    } else if (areq->cmd_type == ZNP_SUBSYS_AF_AREQ) {
        if (areq->cmd_id == AF_INCOMING_MSG_CMD) {
            parse_af_incoming(areq);
        }
    }
}
