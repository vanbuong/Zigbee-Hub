#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "znp_types.h"

/*  Each subsystem constant is the cmd_type byte pre-OR'd with the SREQ frame
 *  type (0x20). ZNP_CMD_TYPE() re-applies the frame-type bits, so callers can
 *  pass these directly. The low 5 bits hold the actual MT subsystem ID. */
#define ZNP_SUBSYS_SYS      0x21    /* subsystem 0x01 */
#define ZNP_SUBSYS_MAC      0x22    /* subsystem 0x02 */
#define ZNP_SUBSYS_NWK      0x23    /* subsystem 0x03 */
#define ZNP_SUBSYS_AF       0x24    /* subsystem 0x04 */
#define ZNP_SUBSYS_ZDO      0x25    /* subsystem 0x05 */
#define ZNP_SUBSYS_SAPI     0x26    /* subsystem 0x06 */
#define ZNP_SUBSYS_UTIL     0x27    /* subsystem 0x07 */
#define ZNP_SUBSYS_DEBUG    0x28    /* subsystem 0x08 */
#define ZNP_SUBSYS_APP      0x29    /* subsystem 0x09 */
#define ZNP_SUBSYS_APP_CNF  0x2F    /* subsystem 0x0F */

/* Helper: build cmd_type byte from subsystem and frame type */
#define ZNP_CMD_TYPE(subsys, ftype)  (((subsys) & 0x1F) | (ftype))

/* ---- SYS subsystem command IDs ---- */
#define SYS_RESET_REQ_CMD   0x00    /* SREQ: soft-reset coprocessor */
#define SYS_PING_CMD        0x01    /* SREQ/SRSP */
#define SYS_VERSION_CMD     0x02    /* SREQ/SRSP */
#define SYS_RESET_IND_CMD   0x80    /* AREQ: reset indication */
#define SYS_OSAL_NV_READ_CMD    0x08  /* SREQ/SRSP: read Z-Stack NV item */
#define SYS_OSAL_NV_WRITE_CMD   0x09  /* SREQ/SRSP: write Z-Stack NV item */
#define SYS_OSAL_NV_DELETE_CMD  0x12  /* SREQ/SRSP: delete Z-Stack NV item */

/* ---- ZDO subsystem command IDs (cmd_id byte; frame type lives in cmd_type) ---- */
#define ZDO_SIMPLE_DESC_REQ_CMD     0x04    /* SREQ: request simple descriptor */
#define ZDO_ACTIVE_EP_REQ_CMD       0x05    /* SREQ: request active endpoints */
#define ZDO_MGMT_PERMIT_JOIN_CMD    0x36    /* SREQ: permit join */
#define ZDO_STARTUP_FROM_APP_CMD    0x40    /* SREQ/SRSP: start coordinator */
#define ZDO_SIMPLE_DESC_RSP_CMD     0x84    /* AREQ: simple descriptor response */
#define ZDO_ACTIVE_EP_RSP_CMD       0x85    /* AREQ: active endpoint response */
#define ZDO_MGMT_PERMIT_JOIN_RSP_CMD 0xB6   /* AREQ: permit join response */
#define ZDO_STATE_CHANGE_IND_CMD    0xC0    /* AREQ: network state change */
#define ZDO_END_DEVICE_ANNCE_IND_CMD 0xC1   /* AREQ: end device announce */
#define ZDO_LEAVE_IND_CMD           0xC9    /* AREQ: device left */
#define ZDO_TC_DEV_IND_CMD          0xCA    /* AREQ: device joined (trust centre) */

/* ---- AF subsystem command IDs ---- */
#define AF_REGISTER_CMD             0x00    /* SREQ/SRSP: register endpoint */
#define AF_DATA_REQUEST_CMD         0x01    /* SREQ/SRSP: send AF data */
#define AF_INCOMING_MSG_CMD         0x81    /* AREQ: incoming AF message */

/* ---- APP_CNF subsystem command IDs ---- */
#define APP_CNF_BDB_START_CMD       0x05    /* SREQ: BDB commissioning start */
#define APP_CNF_BDB_SET_CHANNEL_CMD 0x08    /* SREQ: BDB set channel */

/* ---- ZCD NV item IDs (Z-Stack NV) ---- */
#define ZCD_NV_STARTUP_OPTION   0x0003
#define ZCD_NV_LOGICAL_TYPE     0x0087
#define ZCD_NV_PRECFGKEYS_EN    0x0063
#define ZCD_NV_PRECFGKEY        0x0062  /* 16-byte network key */
#define ZCD_NV_PANID            0x0083  /* 2-byte PAN ID */
#define ZCD_NV_CHANLIST         0x0084  /* 4-byte channel mask */
#define ZCD_NV_BCAST_RETRIES    0x000F
#define ZCD_NV_POLL_RATE        0x0024

/* Coordinator device type */
#define ZB_DEVICE_TYPE_COORDINATOR  0x00

/* ZDO device states */
#define DEV_HOLD                    0x00
#define DEV_INIT                    0x01
#define DEV_NWK_DISC                0x02
#define DEV_NWK_JOINING             0x03
#define DEV_NWK_REJOIN              0x04
#define DEV_END_DEVICE_UNAUTH       0x05
#define DEV_END_DEVICE              0x06
#define DEV_ROUTER                  0x07
#define DEV_COORD_STARTING          0x08
#define DEV_ZB_COORD                0x09    /* Coordinator fully started */
#define DEV_NWK_ORPHAN              0x0A

/* ---- Frame encode / decode ---- */

/**
 * Encode a znp_frame_t into a wire-format byte buffer.
 * @param frame    Source frame.
 * @param buf      Output buffer (must be >= payload_len + 5 bytes).
 * @param buf_len  Size of output buffer.
 * @param out_len  Filled with number of bytes written.
 * @return ESP_OK on success, ESP_ERR_INVALID_SIZE if buffer too small.
 */
esp_err_t znp_frame_encode(const znp_frame_t *frame,
                            uint8_t *buf, size_t buf_len,
                            size_t *out_len);

/**
 * Decode a complete wire-format buffer into a znp_frame_t.
 * Assumes buf[0] == ZNP_SOF and the buffer is exactly one frame.
 * @return ESP_OK on success, ESP_ERR_INVALID_CRC on FCS mismatch.
 */
esp_err_t znp_frame_decode(const uint8_t *buf, size_t len, znp_frame_t *frame);

/**
 * Calculate FCS over length, cmd_type, cmd_id, and payload.
 */
uint8_t znp_fcs_calculate(const znp_frame_t *frame);
