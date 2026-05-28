#pragma once

#include <stdint.h>

#define ZNP_SOF              0xFE
#define ZNP_MAX_PAYLOAD      250

/* MT frame type nibbles (low nibble of cmd_type byte) */
#define ZNP_FRAME_TYPE_POLL  0x00
#define ZNP_FRAME_TYPE_SREQ  0x20
#define ZNP_FRAME_TYPE_AREQ  0x40
#define ZNP_FRAME_TYPE_SRSP  0x60

#define ZNP_FRAME_TYPE_MASK  0xE0

typedef struct {
    uint8_t cmd_type;
    uint8_t cmd_id;
    uint8_t payload[ZNP_MAX_PAYLOAD];
    uint8_t payload_len;
} znp_frame_t;

/* Extended error codes returned by znp_transport functions */
#define ZNP_ERR_BASE           0x1000
#define ZNP_ERR_TIMEOUT        (ZNP_ERR_BASE + 1)
#define ZNP_ERR_BAD_FCS        (ZNP_ERR_BASE + 2)
#define ZNP_ERR_RESET_TIMEOUT  (ZNP_ERR_BASE + 3)
#define ZNP_ERR_QUEUE_FULL     (ZNP_ERR_BASE + 4)
#define ZNP_ERR_NOT_INIT       (ZNP_ERR_BASE + 5)
