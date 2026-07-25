#include "panel_encoder.h"
#include "hardware_config.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#if defined(BOARD_OLIMEX_POE)

static QueueHandle_t s_q;
static DRAM_ATTR volatile int8_t s_last_enc;
static DRAM_ATTR volatile int s_edge_accum;
static int64_t s_press_start_us;
static bool s_pressed;
static bool s_long_sent;

/* Must live in DRAM: enc_isr is IRAM and may run with flash cache disabled. */
static DRAM_ATTR const int8_t s_quad[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
};

static void IRAM_ATTR enc_isr(void *arg)
{
    (void)arg;
    int a = gpio_get_level((gpio_num_t)PANEL_ENC_A_GPIO);
    int b = gpio_get_level((gpio_num_t)PANEL_ENC_B_GPIO);
    int curr = (a << 1) | b;
    int idx = ((s_last_enc & 3) << 2) | (curr & 3);
    int8_t d = s_quad[idx & 15];
    s_last_enc = (int8_t)curr;
    if (d == 0 || !s_q) {
        return;
    }

    /* EC11: 4 quadrature edges per mechanical detent (literal — no flash load). */
    s_edge_accum += d;
    if (s_edge_accum >= 4) {
        s_edge_accum = 0;
        panel_evt_t ev = { .type = PANEL_EVT_ROTATE, .delta = 1 };
        BaseType_t hp = pdFALSE;
        xQueueSendFromISR(s_q, &ev, &hp);
        if (hp) {
            portYIELD_FROM_ISR();
        }
    } else if (s_edge_accum <= -4) {
        s_edge_accum = 0;
        panel_evt_t ev = { .type = PANEL_EVT_ROTATE, .delta = -1 };
        BaseType_t hp = pdFALSE;
        xQueueSendFromISR(s_q, &ev, &hp);
        if (hp) {
            portYIELD_FROM_ISR();
        }
    }
}

esp_err_t panel_encoder_init(QueueHandle_t evt_queue)
{
    s_q = evt_queue;
    s_edge_accum = 0;

    gpio_config_t io = {};
    io.pin_bit_mask = (1ULL << PANEL_ENC_A_GPIO) | (1ULL << PANEL_ENC_B_GPIO) | (1ULL << PANEL_ENC_SW_GPIO);
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&io);

    gpio_set_intr_type((gpio_num_t)PANEL_ENC_SW_GPIO, GPIO_INTR_DISABLE);

    s_last_enc = (int8_t)((gpio_get_level((gpio_num_t)PANEL_ENC_A_GPIO) << 1) |
                          gpio_get_level((gpio_num_t)PANEL_ENC_B_GPIO));

    /* IRAM flag required so the handler can run while flash cache is disabled. */
    esp_err_t isr_err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        return isr_err;
    }
    gpio_isr_handler_add((gpio_num_t)PANEL_ENC_A_GPIO, enc_isr, nullptr);
    gpio_isr_handler_add((gpio_num_t)PANEL_ENC_B_GPIO, enc_isr, nullptr);
    return ESP_OK;
}

void panel_encoder_poll_button(void)
{
    if (!s_q) {
        return;
    }
    bool down = gpio_get_level((gpio_num_t)PANEL_ENC_SW_GPIO) == 0;
    int64_t now = esp_timer_get_time();

    if (down && !s_pressed) {
        s_pressed = true;
        s_long_sent = false;
        s_press_start_us = now;
    } else if (down && s_pressed && !s_long_sent) {
        if ((now - s_press_start_us) >= 700000) {
            s_long_sent = true;
            panel_evt_t ev = { .type = PANEL_EVT_LONG_PRESS, .delta = 0 };
            xQueueSend(s_q, &ev, 0);
        }
    } else if (!down && s_pressed) {
        s_pressed = false;
        if (!s_long_sent && (now - s_press_start_us) >= 30000) {
            panel_evt_t ev = { .type = PANEL_EVT_CLICK, .delta = 0 };
            xQueueSend(s_q, &ev, 0);
        }
    }
}

#else

esp_err_t panel_encoder_init(QueueHandle_t evt_queue)
{
    (void)evt_queue;
    return ESP_ERR_NOT_SUPPORTED;
}

void panel_encoder_poll_button(void) {}

#endif
