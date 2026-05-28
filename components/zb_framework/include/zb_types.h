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
} zb_config_t;

/* ---- Version info ---- */

typedef struct {
    char esp32_fw[32];          /* ESP32 app version string */
    char cc26xx_fw[64];         /* Z-Stack version string from SYS_VERSION */
} zb_versions_t;

/* Event types are defined in zb_subscribe.h (zb_device_mgr component) */
