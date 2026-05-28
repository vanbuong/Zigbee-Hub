#include "znp_transport.h"
#include "znp_mt_protocol.h"
#include "znp_types.h"

#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_err.h"

static const char *TAG = "znp_transport";

/* ---- Internal state ---- */

static znp_transport_config_t s_cfg;
static bool                    s_initialized = false;

/* AREQ queue: populated by dispatch task, consumed by framework */
static QueueHandle_t s_areq_queue;

/* SRSP correlation */
static SemaphoreHandle_t s_sreq_mutex;   /* one SREQ in flight at a time */
static SemaphoreHandle_t s_srsp_sem;     /* signals SRSP arrival to send_sreq() */
static znp_frame_t       s_srsp_frame;  /* written by dispatch task */
static uint8_t           s_pending_cmd_type;
static uint8_t           s_pending_cmd_id;

/* Reset indication semaphore */
static SemaphoreHandle_t s_reset_ind_sem;

/* Error counter */
static volatile uint32_t s_error_count;

/* Internal raw-frame queue between rx_task and dispatch_task */
static QueueHandle_t s_raw_queue;

#define RAW_QUEUE_DEPTH   16
#define AREQ_QUEUE_DEPTH  CONFIG_ZNP_AREQ_QUEUE_DEPTH
#define UART_BUF_SIZE     (1024)

/* ---- RX state machine ---- */

typedef enum {
    RX_HUNT_SOF = 0,
    RX_READ_LEN,
    RX_READ_CMD_TYPE,
    RX_READ_CMD_ID,
    RX_READ_PAYLOAD,
    RX_READ_FCS,
} rx_state_t;

/* ---- Forward declarations ---- */
static void znp_rx_task(void *arg);
static void znp_dispatch_task(void *arg);

/* ---- PCA9538 (I2C GPIO expander) driver ----
 *
 * Register map (datasheet):
 *   0x00 input port (read-only)
 *   0x01 output port
 *   0x02 polarity inversion
 *   0x03 configuration (1 = input, 0 = output)
 *
 * We use bits cfg->pca_reset_bit and cfg->pca_bsl_bit as outputs (active-low).
 * Both pins idle HIGH (1 = not asserted) for normal CC2652P7 operation.
 */

static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_pca;
static uint8_t                 s_pca_output;   /* shadow of register 0x01 */

static esp_err_t pca_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_pca, buf, sizeof(buf), pdMS_TO_TICKS(100));
}

static esp_err_t pca_set_bit(uint8_t bit, bool high)
{
    if (high) s_pca_output |=  (uint8_t)(1u << bit);
    else      s_pca_output &= (uint8_t)~(1u << bit);
    return pca_write_reg(0x01, s_pca_output);
}

static esp_err_t pca_init(const znp_transport_config_t *cfg)
{
    /* Pulse the PCA9538 hardware RESET pin (active-low) before any I2C
     * traffic so the expander starts from a known state every cold boot.
     * Datasheet requires only a few ns; we hold for 10 ms / wait 1 ms to be
     * comfortable. Skip if not wired (pca_rst_gpio < 0). */
    if (cfg->pca_rst_gpio >= 0) {
        gpio_config_t io = {
            .pin_bit_mask = (1ULL << cfg->pca_rst_gpio),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "PCA9538 RST gpio_config failed");

        gpio_set_level(cfg->pca_rst_gpio, 1);    /* ensure high before going low */
        esp_rom_delay_us(100);
        gpio_set_level(cfg->pca_rst_gpio, 0);    /* assert reset (active-low) */
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(cfg->pca_rst_gpio, 1);    /* release reset */
        vTaskDelay(pdMS_TO_TICKS(1));            /* settle */
        ESP_LOGI(TAG, "PCA9538 hardware reset pulsed on GPIO%d", cfg->pca_rst_gpio);
    }

    i2c_master_bus_config_t bus_cfg = {
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .i2c_port                     = cfg->pca_i2c_port,
        .scl_io_num                   = cfg->pca_scl,
        .sda_io_num                   = cfg->pca_sda,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus),
                        TAG, "i2c_new_master_bus failed");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->pca_addr,
        .scl_speed_hz    = 100000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &s_pca),
                        TAG, "i2c_master_bus_add_device failed");

    /* Idle output value: both control bits HIGH (active-low signals not asserted).
     * Set output register first so the lines aren't briefly LOW when we switch
     * the configuration register to "output". */
    s_pca_output = 0xFF;
    ESP_RETURN_ON_ERROR(pca_write_reg(0x01, s_pca_output),
                        TAG, "pca write output failed");

    /* Configure reset/BSL bits as outputs (0 = output); leave others as inputs (1) */
    uint8_t config_reg = (uint8_t)~((1u << cfg->pca_reset_bit) |
                                     (1u << cfg->pca_bsl_bit));
    ESP_RETURN_ON_ERROR(pca_write_reg(0x03, config_reg),
                        TAG, "pca write config failed");

    ESP_LOGI(TAG, "PCA9538 init OK (i2c=%d sda=%d scl=%d addr=0x%02x rst_bit=%d bsl_bit=%d)",
             cfg->pca_i2c_port, cfg->pca_sda, cfg->pca_scl,
             cfg->pca_addr, cfg->pca_reset_bit, cfg->pca_bsl_bit);
    return ESP_OK;
}

/* ---- Init ---- */

esp_err_t znp_transport_init(const znp_transport_config_t *cfg)
{
    if (s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_cfg = *cfg;

    /* UART driver */
    uart_config_t uart_cfg = {
        .baud_rate  = cfg->baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_driver_install(cfg->uart_port,
                                            UART_BUF_SIZE * 2, UART_BUF_SIZE * 2,
                                            0, NULL, 0),
                        TAG, "uart_driver_install failed");
    ESP_RETURN_ON_ERROR(uart_param_config(cfg->uart_port, &uart_cfg),
                        TAG, "uart_param_config failed");
    ESP_RETURN_ON_ERROR(uart_set_pin(cfg->uart_port,
                                     cfg->gpio_tx, cfg->gpio_rx,
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE),
                        TAG, "uart_set_pin failed");
    gpio_pullup_en(cfg->gpio_rx);  /* RX idles HIGH */

    /* PCA9538 — drives RESET (bit cfg->pca_reset_bit) and BSL (bit cfg->pca_bsl_bit).
     * Both idle HIGH (de-asserted). */
    ESP_RETURN_ON_ERROR(pca_init(cfg), TAG, "pca_init failed");

    /* Queues and synchronization primitives */
    s_areq_queue   = xQueueCreate(AREQ_QUEUE_DEPTH, sizeof(znp_frame_t));
    s_raw_queue    = xQueueCreate(RAW_QUEUE_DEPTH,  sizeof(znp_frame_t));
    s_sreq_mutex   = xSemaphoreCreateMutex();
    s_srsp_sem     = xSemaphoreCreateBinary();
    s_reset_ind_sem = xSemaphoreCreateBinary();

    if (!s_areq_queue || !s_raw_queue || !s_sreq_mutex ||
        !s_srsp_sem || !s_reset_ind_sem) {
        return ESP_ERR_NO_MEM;
    }

    /* Tasks */
    BaseType_t ret;
    ret = xTaskCreate(znp_rx_task, "znp_rx",
                      CONFIG_ZNP_RX_TASK_STACK, NULL,
                      configMAX_PRIORITIES - 2, NULL);
    if (ret != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ret = xTaskCreate(znp_dispatch_task, "znp_disp",
                      CONFIG_ZNP_DISPATCH_TASK_STACK, NULL,
                      configMAX_PRIORITIES - 2, NULL);
    if (ret != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "initialized (uart=%d baud=%d tx=%d rx=%d)",
             cfg->uart_port, cfg->baud_rate,
             cfg->gpio_tx, cfg->gpio_rx);
    return ESP_OK;
}

/* ---- SREQ / SRSP ---- */

esp_err_t znp_transport_send_sreq(const znp_frame_t *req, znp_frame_t *resp)
{
    if (!s_initialized) {
        return ZNP_ERR_NOT_INIT;
    }

    uint8_t wire[ZNP_MAX_PAYLOAD + 5];
    size_t  wire_len;
    ESP_RETURN_ON_ERROR(znp_frame_encode(req, wire, sizeof(wire), &wire_len),
                        TAG, "encode failed");

    if (xSemaphoreTake(s_sreq_mutex, pdMS_TO_TICKS(s_cfg.sreq_timeout_ms)) != pdTRUE) {
        s_error_count++;
        return ZNP_ERR_TIMEOUT;
    }

    /* Record what SRSP we expect */
    s_pending_cmd_type = (req->cmd_type & ~ZNP_FRAME_TYPE_MASK) | ZNP_FRAME_TYPE_SRSP;
    s_pending_cmd_id   = req->cmd_id;

    /* Clear any stale semaphore token */
    xSemaphoreTake(s_srsp_sem, 0);

    uart_write_bytes(s_cfg.uart_port, wire, wire_len);

    esp_err_t result = ESP_OK;
    if (xSemaphoreTake(s_srsp_sem, pdMS_TO_TICKS(s_cfg.sreq_timeout_ms)) != pdTRUE) {
        ESP_LOGW(TAG, "SRSP timeout cmd_type=0x%02x cmd_id=0x%02x",
                 req->cmd_type, req->cmd_id);
        s_error_count++;
        result = ZNP_ERR_TIMEOUT;
    } else if (resp) {
        *resp = s_srsp_frame;
    }

    xSemaphoreGive(s_sreq_mutex);
    return result;
}

/* ---- Coprocessor reset ---- */

esp_err_t znp_transport_reset_coprocessor(void)
{
    if (!s_initialized) {
        return ZNP_ERR_NOT_INIT;
    }

    /* Drain existing reset indications */
    xSemaphoreTake(s_reset_ind_sem, 0);

    ESP_LOGI(TAG, "asserting RESET via PCA9538 bit %d", s_cfg.pca_reset_bit);
    /* Ensure BSL is de-asserted so the chip enters normal app mode, not BSL. */
    pca_set_bit(s_cfg.pca_bsl_bit,   true);   /* HIGH = not asserting BSL */
    pca_set_bit(s_cfg.pca_reset_bit, false);  /* LOW  = assert reset */
    vTaskDelay(pdMS_TO_TICKS(10));
    pca_set_bit(s_cfg.pca_reset_bit, true);   /* release reset */

    if (xSemaphoreTake(s_reset_ind_sem,
                       pdMS_TO_TICKS(s_cfg.reset_timeout_ms)) != pdTRUE) {
        ESP_LOGE(TAG, "SYS_RESET_IND not received within %"PRIu32" ms",
                 s_cfg.reset_timeout_ms);
        s_error_count++;
        return ZNP_ERR_RESET_TIMEOUT;
    }

    ESP_LOGI(TAG, "SYS_RESET_IND received");
    return ESP_OK;
}

/* ---- AREQ receive ---- */

BaseType_t znp_transport_receive_areq(znp_frame_t *out, TickType_t ticks_to_wait)
{
    return xQueueReceive(s_areq_queue, out, ticks_to_wait);
}

/* ---- Error counter ---- */

uint32_t znp_transport_get_error_count(void)
{
    return s_error_count;
}

/* ---- RX task: UART bytes → assembled MT frames ---- */

static void znp_rx_task(void *arg)
{
    rx_state_t  state     = RX_HUNT_SOF;
    znp_frame_t frame;
    uint8_t     byte;
    int         payload_idx = 0;
    uint8_t     rx_byte[1];

    for (;;) {
        int n = uart_read_bytes(s_cfg.uart_port, rx_byte, 1, pdMS_TO_TICKS(10));
        if (n <= 0) {
            continue;
        }
        byte = rx_byte[0];

        switch (state) {
        case RX_HUNT_SOF:
            if (byte == ZNP_SOF) {
                state = RX_READ_LEN;
            }
            break;

        case RX_READ_LEN:
            frame.payload_len = byte;
            state = RX_READ_CMD_TYPE;
            break;

        case RX_READ_CMD_TYPE:
            frame.cmd_type = byte;
            state = RX_READ_CMD_ID;
            break;

        case RX_READ_CMD_ID:
            frame.cmd_id = byte;
            payload_idx  = 0;
            state = (frame.payload_len > 0) ? RX_READ_PAYLOAD : RX_READ_FCS;
            break;

        case RX_READ_PAYLOAD:
            if (payload_idx < ZNP_MAX_PAYLOAD) {
                frame.payload[payload_idx++] = byte;
            }
            if (payload_idx >= frame.payload_len) {
                state = RX_READ_FCS;
            }
            break;

        case RX_READ_FCS: {
            uint8_t expected = znp_fcs_calculate(&frame);
            if (byte != expected) {
                ESP_LOGW(TAG, "FCS error: expected 0x%02x got 0x%02x — re-syncing",
                         expected, byte);
                s_error_count++;
                /* Drain: re-hunt for SOF */
                state = RX_HUNT_SOF;
                break;
            }
            /* Valid frame — post to raw queue */
            if (xQueueSend(s_raw_queue, &frame, 0) != pdTRUE) {
                ESP_LOGW(TAG, "raw queue full, frame dropped");
                s_error_count++;
            }
            state = RX_HUNT_SOF;
            break;
        }
        }
    }
}

/* ---- Dispatch task: route assembled frames to SRSP correlator or AREQ queue ---- */

static void znp_dispatch_task(void *arg)
{
    znp_frame_t frame;

    for (;;) {
        if (xQueueReceive(s_raw_queue, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        uint8_t frame_type = frame.cmd_type & ZNP_FRAME_TYPE_MASK;

        if (frame_type == ZNP_FRAME_TYPE_SRSP) {
            /* Check if this matches the pending SREQ */
            if (frame.cmd_type == s_pending_cmd_type &&
                frame.cmd_id   == s_pending_cmd_id) {
                s_srsp_frame = frame;
                xSemaphoreGive(s_srsp_sem);
            } else {
                ESP_LOGW(TAG, "unexpected SRSP cmd_type=0x%02x cmd_id=0x%02x",
                         frame.cmd_type, frame.cmd_id);
            }

        } else if (frame_type == ZNP_FRAME_TYPE_AREQ) {
            /* SYS_RESET_IND — signal reset semaphore as well as posting to AREQ queue */
            if (frame.cmd_id == SYS_RESET_IND_CMD) {
                xSemaphoreGive(s_reset_ind_sem);
            }
            if (xQueueSend(s_areq_queue, &frame, 0) != pdTRUE) {
                ESP_LOGW(TAG, "AREQ queue full, frame dropped cmd_id=0x%02x",
                         frame.cmd_id);
                s_error_count++;
            }
        } else {
            ESP_LOGW(TAG, "unknown frame type 0x%02x", frame_type);
        }
    }
}
