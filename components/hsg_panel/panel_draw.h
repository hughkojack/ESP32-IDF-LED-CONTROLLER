#pragma once

#include "esp_lcd_panel_ops.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PANEL_W 320
#define PANEL_H 240

/** 8x8 glyphs drawn at this integer scale (2 => 16x16 cells). */
#define PANEL_FONT_SCALE 2

#define COL_BG     0x0000u
#define COL_FG     0xFFFFu
#define COL_DIM    0x8410u
#define COL_ACCENT 0x07E0u
#define COL_WARN   0xFD20u
#define COL_SEL    0x001Fu

void panel_draw_init(esp_lcd_panel_handle_t panel);
void panel_draw_clear(uint16_t color);
void panel_draw_fill_rect(int x, int y, int w, int h, uint16_t color);
void panel_draw_rect_outline(int x, int y, int w, int h, uint16_t color);
void panel_draw_circle_outline(int cx, int cy, int r, uint16_t color);
void panel_draw_circle_fill(int cx, int cy, int r, uint16_t color);
void panel_draw_cross(int cx, int cy, int arm, uint16_t color);
void panel_draw_text(int x, int y, const char *s, uint16_t fg, uint16_t bg);
/** Same as panel_draw_text but with an explicit integer glyph scale (1 = 8x8). */
void panel_draw_text_scaled(int x, int y, const char *s, uint16_t fg, uint16_t bg, int scale);
/** Nearest-neighbour upscale to an arbitrary cell size in pixels (e.g. 12 ≈ midway 8..16). */
void panel_draw_text_px(int x, int y, const char *s, uint16_t fg, uint16_t bg, int cell_px);
void panel_draw_text_centered(int y, const char *s, uint16_t fg, uint16_t bg);
/** Draw text into a filled row band (avoids full-screen clear). */
void panel_draw_text_in_row(int y, int row_h, int x, const char *s, uint16_t fg, uint16_t bg);
int panel_draw_char_width(void);
int panel_draw_char_height(void);
int panel_draw_text_width(const char *s);

#ifdef __cplusplus
}
#endif
