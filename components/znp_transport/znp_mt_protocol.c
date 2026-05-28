#include "znp_mt_protocol.h"
#include "znp_types.h"
#include <string.h>
#include "esp_err.h"

uint8_t znp_fcs_calculate(const znp_frame_t *frame)
{
    uint8_t fcs = frame->payload_len ^ frame->cmd_type ^ frame->cmd_id;
    for (int i = 0; i < frame->payload_len; i++) {
        fcs ^= frame->payload[i];
    }
    return fcs;
}

esp_err_t znp_frame_encode(const znp_frame_t *frame,
                            uint8_t *buf, size_t buf_len,
                            size_t *out_len)
{
    /* Wire format: SOF(1) + LEN(1) + CMD_TYPE(1) + CMD_ID(1) + PAYLOAD(n) + FCS(1) */
    size_t required = (size_t)frame->payload_len + 5;
    if (buf_len < required) {
        return ESP_ERR_INVALID_SIZE;
    }

    buf[0] = ZNP_SOF;
    buf[1] = frame->payload_len;
    buf[2] = frame->cmd_type;
    buf[3] = frame->cmd_id;
    if (frame->payload_len > 0) {
        memcpy(&buf[4], frame->payload, frame->payload_len);
    }
    buf[4 + frame->payload_len] = znp_fcs_calculate(frame);

    if (out_len) {
        *out_len = required;
    }
    return ESP_OK;
}

esp_err_t znp_frame_decode(const uint8_t *buf, size_t len, znp_frame_t *frame)
{
    /* Minimum frame: SOF + LEN + CMD_TYPE + CMD_ID + FCS = 5 bytes */
    if (len < 5 || buf[0] != ZNP_SOF) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t payload_len = buf[1];
    if (len < (size_t)(payload_len + 5)) {
        return ESP_ERR_INVALID_SIZE;
    }

    frame->payload_len = payload_len;
    frame->cmd_type    = buf[2];
    frame->cmd_id      = buf[3];
    if (payload_len > 0) {
        memcpy(frame->payload, &buf[4], payload_len);
    }

    uint8_t expected_fcs = znp_fcs_calculate(frame);
    uint8_t received_fcs = buf[4 + payload_len];
    if (expected_fcs != received_fcs) {
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}
