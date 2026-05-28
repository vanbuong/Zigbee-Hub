#pragma once

#include "esp_err.h"

/**
 * Configure the button GPIO (FR-10.1, FR-10.7).
 * Must be called before zb_button_start().
 * GPIO number, polarity, and all timing constants are set via Kconfig.
 */
esp_err_t zb_button_init(void);

/**
 * Start the button polling task (FR-10.8).
 * Must be called after zb_subscribe infrastructure is ready
 * (i.e. after zb_dev_mgr_init()).
 * Button events are posted to the zb_subscribe event bus as ZB_EVENT_BUTTON.
 */
esp_err_t zb_button_start(void);
