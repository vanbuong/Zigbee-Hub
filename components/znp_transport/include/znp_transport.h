#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "znp_types.h"

typedef struct {
    uart_port_t uart_port;      /* UART peripheral number */
    int         baud_rate;      /* Baud rate (default 115200) */
    int         gpio_tx;        /* ESP32 TX GPIO (to CC2652P7 RX) */
    int         gpio_rx;        /* ESP32 RX GPIO (from CC2652P7 TX) */

    /* CC2652P7 RESET and BSL pins are not on the ESP32 directly — they are
     * controlled through a PCA9538 8-bit I2C GPIO expander.  Both signals are
     * active-low (output LOW = asserted). */
    int         pca_sda;        /* ESP32 I2C SDA GPIO */
    int         pca_scl;        /* ESP32 I2C SCL GPIO */
    int         pca_i2c_port;   /* I2C peripheral number (0 or 1) */
    uint8_t     pca_addr;       /* PCA9538 7-bit I2C address (0x70..0x73) */
    uint8_t     pca_reset_bit;  /* PCA9538 bit driving CC2652P7 RESET */
    uint8_t     pca_bsl_bit;    /* PCA9538 bit driving CC2652P7 BSL_INVOKE */
    int         pca_rst_gpio;   /* ESP32 GPIO -> PCA9538 RESET (active-low); -1 = none */

    uint32_t    sreq_timeout_ms;  /* SREQ→SRSP timeout in ms (default 3000) */
    uint32_t    reset_timeout_ms; /* Wait for SYS_RESET_IND after reset (default 3000) */
} znp_transport_config_t;

/**
 * Initialize the ZNP transport layer: UART driver, GPIOs, internal queues,
 * and FreeRTOS tasks (znp_rx_task, znp_dispatch_task).
 * Must be called once before any other znp_transport_* function.
 */
esp_err_t znp_transport_init(const znp_transport_config_t *cfg);

/**
 * Send a synchronous SREQ and wait for the matching SRSP.
 * Thread-safe: internally serialized by a mutex — only one SREQ in flight at a time.
 * @return ESP_OK on success.
 *         ZNP_ERR_TIMEOUT if no SRSP received within sreq_timeout_ms.
 */
esp_err_t znp_transport_send_sreq(const znp_frame_t *req, znp_frame_t *resp);

/**
 * Same as znp_transport_send_sreq() but with an explicit per-call timeout.
 * Use for commands that take longer than the default (e.g. ZDO_STARTUP_FROM_APP
 * may need ~10 s in Z-Stack 2.x because the stack does scan / commissioning
 * before returning SRSP).
 * @param timeout_ms  Wait this long for SRSP instead of cfg->sreq_timeout_ms.
 */
esp_err_t znp_transport_send_sreq_timeout(const znp_frame_t *req,
                                           znp_frame_t *resp,
                                           uint32_t timeout_ms);

/**
 * Hard-reset the CC2652P7 by asserting its RESET GPIO, then wait for a
 * SYS_RESET_IND AREQ within reset_timeout_ms.
 * @return ESP_OK on success, ZNP_ERR_RESET_TIMEOUT if no indication received.
 */
esp_err_t znp_transport_reset_coprocessor(void);

/**
 * Receive an AREQ frame from the transport's AREQ queue.
 * @param out             Output frame buffer.
 * @param ticks_to_wait   FreeRTOS tick count to block (use portMAX_DELAY to block forever).
 * @return pdTRUE if a frame was received, pdFALSE on timeout.
 */
BaseType_t znp_transport_receive_areq(znp_frame_t *out, TickType_t ticks_to_wait);

/**
 * Return the cumulative error count (FCS errors + SREQ timeouts + UART overruns).
 */
uint32_t znp_transport_get_error_count(void);
