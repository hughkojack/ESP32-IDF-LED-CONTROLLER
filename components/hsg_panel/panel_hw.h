#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_ops.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t panel_hw_init(void);
esp_lcd_panel_handle_t panel_hw_panel(void);
bool panel_hw_ready(void);

/**
 * Queue a bitmap and wait until its color DMA finishes.
 * Safe to reuse `color` immediately after this returns.
 */
esp_err_t panel_hw_draw_bitmap(int x0, int y0, int x1, int y1, const void *color);

#ifdef __cplusplus
}
#endif
