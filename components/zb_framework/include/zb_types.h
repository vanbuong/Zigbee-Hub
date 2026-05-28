#pragma once

#include <stdint.h>
#include "driver/uart.h"

/* ---- State machine ---- */

typedef enum {
    ZB_STATE_UNINITIALIZED = 0,
    ZB_STATE_ZNP_INIT,
    ZB_STATE_NETWORK_CHECK,
    ZB_STATE_FORMING,
    ZB_STATE_RECONFIGURING,
    ZB_STATE_READY,
    ZB_STATE_FW_UPDATE,
} zb_state_t;

/* ---- Configuration ---- */

typedef struct {
    uint16_t    pan_id;         /* PAN ID; 0 → use CONFIG_ZB_DEFAULT_PAN_ID */
    uint8_t     channel;        /* Zigbee channel 11-26; 0 → use CONFIG_ZB_DEFAULT_CHANNEL */
    uint8_t     nwk_key[16];    /* 128-bit network key; all-zeros → use default */

    /* UART to CC2652P7 */
    uart_port_t uart_port;
    uint32_t    uart_baud;
    int         gpio_tx;
    int         gpio_rx;

    /* CC2652P7 RESET / BSL are routed through a PCA9538 I2C GPIO expander */
    int         pca_sda;        /* I2C SDA GPIO */
    int         pca_scl;        /* I2C SCL GPIO */
    int         pca_i2c_port;   /* I2C peripheral number */
    uint8_t     pca_addr;       /* PCA9538 7-bit I2C address */
    uint8_t     pca_reset_bit;  /* PCA9538 bit driving CC2652P7 RESET (active-low) */
    uint8_t     pca_bsl_bit;    /* PCA9538 bit driving CC2652P7 BSL_INVOKE (active-low) */
    int         pca_rst_gpio;   /* ESP32 GPIO -> PCA9538 RESET (active-low); -1 = none */
} zb_config_t;

/* ---- Version info ---- */

typedef struct {
    char esp32_fw[32];          /* ESP32 app version string */
    char cc26xx_fw[64];         /* Z-Stack version string from SYS_VERSION */
} zb_versions_t;

/* ---- Network info (FR-11) ---- */

typedef enum {
    ZB_NET_STATUS_OFFLINE = 0,   /* UNINITIALIZED or ZNP_INIT — coprocessor not responding */
    ZB_NET_STATUS_FORMING,       /* NETWORK_CHECK / FORMING / RECONFIGURING */
    ZB_NET_STATUS_READY,         /* READY — coordinator operational, no join window open */
    ZB_NET_STATUS_PERMIT_JOIN,   /* READY — join window open (permit_join_ttl > 0) */
    ZB_NET_STATUS_FW_UPDATE,     /* FW_UPDATE — normal traffic suspended */
} zb_network_status_t;

typedef struct {
    zb_network_status_t status;          /* high-level derived status (FR-11.3) */
    zb_state_t          fw_state;        /* raw framework state machine value */
    uint16_t            pan_id;          /* active PAN ID; 0 if not yet configured */
    uint8_t             channel;         /* active channel; 0 if not yet configured */
    uint8_t             permit_join_ttl; /* seconds remaining in join window; 0 = closed */
    uint16_t            device_count;    /* devices in the in-memory registry */
} zb_network_info_t;

/* Event types are defined in zb_subscribe.h (zb_device_mgr component) */
