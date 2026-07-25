#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PANEL_EVT_ROTATE = 1,
    PANEL_EVT_CLICK,
    PANEL_EVT_LONG_PRESS,
} panel_evt_type_t;

typedef struct {
    panel_evt_type_t type;
    int8_t delta; /* for ROTATE: +1 / -1 */
} panel_evt_t;

esp_err_t panel_encoder_init(QueueHandle_t evt_queue);
void panel_encoder_poll_button(void);

#ifdef __cplusplus
}
#endif
