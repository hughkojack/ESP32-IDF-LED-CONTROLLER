#include "hsg_panel.h"
#include "panel_hw.h"
#include "panel_encoder.h"
#include "panel_draw.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <string.h>

#if defined(BOARD_OLIMEX_POE)

static const char *TAG = "hsg_panel";

typedef enum {
    SCREEN_ROOT = 0,
    SCREEN_OUTPUTS,
    SCREEN_NODES,
    SCREEN_NETWORK,
    SCREEN_ABOUT,
} panel_screen_t;

static hsg_panel_callbacks_t s_cb;
static QueueHandle_t s_evt_q;
static panel_screen_t s_screen = SCREEN_ROOT;
static int s_sel = 0;
static int s_scroll = 0;

static const char *k_root_items[] = {
    "Outputs",
    "CAN Nodes",
    "Network",
    "About",
};
static const int k_root_count = 4;
static const int k_header_h = 32;
static const int k_hint_h = 20; /* fits 12px mid-size hint text */
static const int k_row_h = 28;
static const int k_list_y0 = 36;
static const int k_visible_rows = 6;

/* Hub: four 2×4 modules. Numbering matches connector drawing:
 * TL: 1,2 / 5,6  TR: 3,4 / 7,8  BL: 9,10 / 13,14  BR: 11,12 / 15,16
 *
 * Panel is BGR: greens look right with normal RGB565; red/yellow need R↔B
 * in the authored tuple so they display as intended. */
#define PIX_RGB(r, g, b) \
    ((uint16_t)(((((r) & 0xF8u) << 8) | (((g) & 0xFCu) << 3) | (((b) & 0xF8u) >> 3))))

static const int k_hub_outputs = 16;

/* Connector board = previous terminal/OFF green; terminals = previous ON green. */
static const uint16_t COL_BOARD  = PIX_RGB(0x38, 0xA0, 0x48); /* connector PCB */
static const uint16_t COL_STRIP  = PIX_RGB(0x20, 0xC0, 0x40); /* terminal body */
static const uint16_t COL_CH_OFF = PIX_RGB(0x20, 0xC0, 0x40); /* mapped OFF (= terminal) */
static const uint16_t COL_CH_ON  = PIX_RGB(0x00, 0x18, 0xE0); /* ON = red (BGR-compensated) */
static const uint16_t COL_SEL_Y  = PIX_RGB(0x00, 0xE0, 0xFF); /* yellow (BGR-compensated) */
static const uint16_t COL_SLOT   = PIX_RGB(0xFF, 0xFF, 0xFF);
static const uint16_t COL_POLAR  = PIX_RGB(0x00, 0x00, 0x00); /* + / - */
static const uint16_t COL_SCREW  = PIX_RGB(0x90, 0x90, 0x90);
static const uint16_t COL_SCREW_SLOT = PIX_RGB(0x38, 0x38, 0x38);
static const uint16_t COL_NUM    = PIX_RGB(0x00, 0x00, 0x00); /* channel labels */
static const uint16_t COL_SLOT_BASE = PIX_RGB(0x40, 0x40, 0x40);
static const int k_sel_thick = 3;

static const int k_mod_w = 150;
static const int k_mod_h = 76; /* fit 3x gaps + taller hint */
static const int k_mod_gap_x = 6;
static const int k_mod_gap_y = 12; /* same as bar→connector gap */
static const int k_mod_x0 = 7;
static const int k_mod_y0 = 44; /* header 32 + 3× previous 4px gap */
static const int k_strip_pad_x = 18; /* room for L/R screws beside strips */
static const int k_label_h = 9;
static const int k_strip_h = 20;
static const int k_pol_h = 14; /* room for larger + / - */
static const int k_pol_arm = 4; /* half-size of + arms / - half-width */
static const int k_slot_w = 10;
static const int k_slot_h = 12;
static const int k_screw_r = 6;
static const int k_hint_cell_px = 12; /* midway between 8 (prev) and 16 (2x) */

/* 5x7 digits 0-9, MSB = left pixel per row. */
static const uint8_t k_digit5x7[10][7] = {
    {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}, /* 0 */
    {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, /* 1 */
    {0x0E,0x11,0x01,0x06,0x08,0x10,0x1F}, /* 2 */
    {0x0E,0x11,0x01,0x06,0x01,0x11,0x0E}, /* 3 */
    {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}, /* 4 */
    {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}, /* 5 */
    {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}, /* 6 */
    {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}, /* 7 */
    {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}, /* 8 */
    {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}, /* 9 */
};

static bool s_pad_on[k_hub_outputs];
static bool s_pad_mapped[k_hub_outputs];
static int s_pad_id[k_hub_outputs];
static int s_pad_count = 0;

static void draw_digit5x7(int x, int y, int digit, uint16_t fg, uint16_t bg)
{
    if (digit < 0 || digit > 9) return;
    const uint8_t *g = k_digit5x7[digit];
    for (int row = 0; row < 7; ++row) {
        uint8_t bits = g[row];
        for (int col = 0; col < 5; ++col) {
            uint16_t pix = (bits & (0x10u >> col)) ? fg : bg;
            panel_draw_fill_rect(x + col, y + row, 1, 1, pix);
        }
    }
}

static void draw_number5x7(int cx, int y, int n, uint16_t fg, uint16_t bg)
{
    if (n < 0) n = 0;
    if (n > 99) n = 99;
    if (n < 10) {
        draw_digit5x7(cx - 2, y, n, fg, bg);
    } else {
        draw_digit5x7(cx - 6, y, n / 10, fg, bg);
        draw_digit5x7(cx + 1, y, n % 10, fg, bg);
    }
}

static void paint_screw(int cx, int cy)
{
    /* Flat-head: filled disc + single drive slot rotated 45°. */
    panel_draw_circle_fill(cx, cy, k_screw_r, COL_SCREW);
    panel_draw_circle_outline(cx, cy, k_screw_r, COL_SCREW_SLOT);
    const int r = k_screw_r - 1;
    for (int i = -r; i <= r; ++i) {
        panel_draw_fill_rect(cx + i - 1, cy + i, 3, 2, COL_SCREW_SLOT);
    }
}

static void clamp_sel(int count, int *scroll_changed)
{
    int old_scroll = s_scroll;
    if (count <= 0) {
        s_sel = 0;
        s_scroll = 0;
    } else {
        if (s_sel < 0) s_sel = 0;
        if (s_sel >= count) s_sel = count - 1;
        if (s_sel < s_scroll) s_scroll = s_sel;
        if (s_sel >= s_scroll + k_visible_rows) s_scroll = s_sel - k_visible_rows + 1;
    }
    if (scroll_changed) {
        *scroll_changed = (s_scroll != old_scroll);
    }
}

static void draw_header(const char *title)
{
    panel_draw_fill_rect(0, 0, PANEL_W, k_header_h, COL_SEL);
    panel_draw_text(8, (k_header_h - panel_draw_char_height()) / 2, title, COL_FG, COL_SEL);
}

static void draw_hint(const char *hint)
{
    panel_draw_fill_rect(0, PANEL_H - k_hint_h, PANEL_W, k_hint_h, COL_DIM);
    panel_draw_text_px(6, PANEL_H - k_hint_h + (k_hint_h - k_hint_cell_px) / 2,
                       hint, COL_FG, COL_DIM, k_hint_cell_px);
}

static void format_root_row(int idx, char *line, size_t len)
{
    snprintf(line, len, " %s", k_root_items[idx]);
}

static void format_node_row(const hsg_panel_node_info_t *n, char *line, size_t len)
{
    const char *typ = (n->node_type == 2) ? "MECH" : "LCD";
    snprintf(line, len, "ID %u  %s  %s", (unsigned)n->node_id, typ, n->online ? "up" : "down");
}

static int load_hub_pads(void)
{
    static hsg_panel_output_info_t outs[HSG_PANEL_MAX_OUTPUTS];
    int n = s_cb.list_outputs ? s_cb.list_outputs(outs, HSG_PANEL_MAX_OUTPUTS) : 0;

    for (int i = 0; i < k_hub_outputs; ++i) {
        s_pad_mapped[i] = false;
        s_pad_on[i] = false;
        s_pad_id[i] = i + 1;
    }

    int mapped = 0;
    for (int i = 0; i < n; ++i) {
        int id = outs[i].id;
        if (id < 1 || id > k_hub_outputs) continue;
        int idx = id - 1;
        s_pad_mapped[idx] = true;
        s_pad_on[idx] = outs[i].on;
        s_pad_id[idx] = id;
        ++mapped;
    }
    s_pad_count = mapped;
    return mapped;
}

static void module_origin(int mod, int *ox, int *oy)
{
    const int col = mod % 2;
    const int row = mod / 2;
    *ox = k_mod_x0 + col * (k_mod_w + k_mod_gap_x);
    *oy = k_mod_y0 + row * (k_mod_h + k_mod_gap_y);
}

/* Map output index 0..15 → module / strip / pair, matching the connector drawing. */
static void channel_place(int index, int *mod, int *strip, int *pair)
{
    const int band = index / 8;   /* 0: outs 1-8, 1: outs 9-16 */
    const int within = index % 8;
    *mod = band * 2 + (within % 4) / 2;
    *strip = within / 4;          /* 0 top strip, 1 bottom strip */
    *pair = within % 2;           /* 0 left channel, 1 right channel */
}

static int module_channel_id(int mod, int strip, int pair)
{
    /* mod0: 1,2 / 5,6  mod1: 3,4 / 7,8  mod2: 9,10 / 13,14  mod3: 11,12 / 15,16 */
    int base = (mod / 2) * 8 + (mod % 2) * 2 + 1;
    if (strip) base += 4;
    return base + pair;
}

static void strip_geometry(int ox, int oy, int strip, int *strip_x, int *strip_y, int *strip_w)
{
    *strip_x = ox + k_strip_pad_x;
    *strip_w = k_mod_w - 2 * k_strip_pad_x;
    /* top strip: numbers above; bottom strip: numbers below */
    if (strip == 0) {
        *strip_y = oy + 2 + k_label_h;
    } else {
        *strip_y = oy + 2 + k_label_h + k_strip_h + k_pol_h;
    }
}

static void channel_rect(int index, int *x, int *y, int *w, int *h)
{
    int mod, strip, pair;
    channel_place(index, &mod, &strip, &pair);
    int ox, oy;
    module_origin(mod, &ox, &oy);

    int strip_x, strip_y, strip_w;
    strip_geometry(ox, oy, strip, &strip_x, &strip_y, &strip_w);
    const int ch_w = strip_w / 2;

    *x = strip_x + pair * ch_w;
    *y = strip_y;
    *w = ch_w;
    *h = k_strip_h;
}

static void paint_strip_slots(int strip_x, int strip_y, int strip_w, uint16_t body)
{
    panel_draw_fill_rect(strip_x, strip_y, strip_w, k_strip_h, body);
    panel_draw_rect_outline(strip_x, strip_y, strip_w, k_strip_h, COL_DIM);

    const int mid_x = strip_x + strip_w / 2;
    panel_draw_fill_rect(mid_x, strip_y, 1, k_strip_h, COL_DIM);

    /* Four vertical slots across the strip */
    const int cell_w = strip_w / 4;
    for (int s = 0; s < 4; ++s) {
        const int cx = strip_x + s * cell_w + cell_w / 2;
        const int sx = cx - k_slot_w / 2;
        const int sy = strip_y + 2;
        panel_draw_fill_rect(sx, sy, k_slot_w, k_slot_h, COL_SLOT);
        panel_draw_rect_outline(sx, sy, k_slot_w, k_slot_h, COL_DIM);
        panel_draw_fill_rect(sx + 1, sy + k_slot_h - 3, k_slot_w - 2, 2, COL_SLOT_BASE);
    }
}

static void paint_module_frame(int mod)
{
    int ox, oy;
    module_origin(mod, &ox, &oy);

    panel_draw_fill_rect(ox, oy, k_mod_w, k_mod_h, COL_BOARD);
    panel_draw_rect_outline(ox, oy, k_mod_w, k_mod_h, COL_DIM);

    /* Notch the four corners to the screen background (black). */
    const int cut_w = k_strip_pad_x; /* out to start of 1st terminal */
    const int cut_h = k_label_h;     /* same height as channel numbers */
    if (cut_w > 0 && cut_h > 0) {
        panel_draw_fill_rect(ox, oy, cut_w, cut_h, COL_BG);
        panel_draw_fill_rect(ox + k_mod_w - cut_w, oy, cut_w, cut_h, COL_BG);
        panel_draw_fill_rect(ox, oy + k_mod_h - cut_h, cut_w, cut_h, COL_BG);
        panel_draw_fill_rect(ox + k_mod_w - cut_w, oy + k_mod_h - cut_h, cut_w, cut_h, COL_BG);
    }

    int top_x, top_y, top_w;
    int bot_x, bot_y, bot_w;
    strip_geometry(ox, oy, 0, &top_x, &top_y, &top_w);
    strip_geometry(ox, oy, 1, &bot_x, &bot_y, &bot_w);

    /* Screws left/right of each strip, horizontally centered with the strip. */
    const int screw_l = ox + k_strip_pad_x / 2;
    const int screw_r = ox + k_mod_w - k_strip_pad_x / 2;
    paint_screw(screw_l, top_y + k_strip_h / 2);
    paint_screw(screw_r, top_y + k_strip_h / 2);
    paint_screw(screw_l, bot_y + k_strip_h / 2);
    paint_screw(screw_r, bot_y + k_strip_h / 2);

    /* Channel numbers: above top strip; below bottom strip (2nd/4th rows). */
    const int ch_w = top_w / 2;
    for (int pair = 0; pair < 2; ++pair) {
        int id_top = module_channel_id(mod, 0, pair);
        int id_bot = module_channel_id(mod, 1, pair);
        int cx = top_x + pair * ch_w + ch_w / 2;
        draw_number5x7(cx, top_y - k_label_h, id_top, COL_NUM, COL_BOARD);
        draw_number5x7(cx, bot_y + k_strip_h + 1, id_bot, COL_NUM, COL_BOARD);
    }

    paint_strip_slots(top_x, top_y, top_w, COL_STRIP);
    paint_strip_slots(bot_x, bot_y, bot_w, COL_STRIP);

    /* + - vertically centred between strips, larger marks under each slot */
    const int pol_y = top_y + k_strip_h + k_pol_h / 2;
    const int cell_w = top_w / 4;
    for (int s = 0; s < 4; ++s) {
        const int cx = top_x + s * cell_w + cell_w / 2;
        if ((s % 2) == 0) {
            panel_draw_cross(cx, pol_y, k_pol_arm, COL_POLAR);
        } else {
            panel_draw_fill_rect(cx - k_pol_arm, pol_y - 1, 2 * k_pol_arm + 1, 3, COL_POLAR);
        }
    }
}

static void paint_output_pad(int index, bool selected)
{
    if (index < 0 || index >= k_hub_outputs) return;

    int x, y, w, h;
    channel_rect(index, &x, &y, &w, &h);

    const bool mapped = s_pad_mapped[index];
    const bool on = mapped && s_pad_on[index];

    uint16_t fill;
    if (!mapped) {
        fill = PIX_RGB(0x18, 0x18, 0x18);
    } else if (on) {
        fill = COL_CH_ON;
    } else {
        fill = COL_CH_OFF;
    }

    panel_draw_fill_rect(x + 1, y + 1, w - 2, h - 2, fill);

    /* Two white vertical slots for this channel (+ / -) */
    const int cell_w = w / 2;
    for (int s = 0; s < 2; ++s) {
        const int cx = x + s * cell_w + cell_w / 2;
        const int sx = cx - k_slot_w / 2;
        const int sy = y + 2;
        uint16_t slot = mapped ? COL_SLOT : COL_DIM;
        panel_draw_fill_rect(sx, sy, k_slot_w, k_slot_h, slot);
        panel_draw_rect_outline(sx, sy, k_slot_w, k_slot_h, COL_DIM);
        panel_draw_fill_rect(sx + 1, sy + k_slot_h - 3, k_slot_w - 2, 2, COL_SLOT_BASE);
    }

    /* Selection: thick yellow frame around the terminal half (fill unchanged). */
    if (selected) {
        for (int t = 0; t < k_sel_thick; ++t) {
            panel_draw_rect_outline(x + t, y + t, w - 2 * t, h - 2 * t, COL_SEL_Y);
        }
    }
}

static void paint_output_status(void)
{
    char line[40];
    if (s_sel < 0 || s_sel >= k_hub_outputs || !s_pad_mapped[s_sel]) {
        snprintf(line, sizeof(line), "Out -  ----");
    } else {
        snprintf(line, sizeof(line), "Out %d  %s", s_pad_id[s_sel], s_pad_on[s_sel] ? "ON" : "OFF");
    }
    /* Status replaces the title bar on the Outputs page. */
    panel_draw_fill_rect(0, 0, PANEL_W, k_header_h, COL_SEL);
    panel_draw_text(8, (k_header_h - panel_draw_char_height()) / 2, line, COL_FG, COL_SEL);
}

static void paint_row(int absolute_index, bool selected)
{
    if (absolute_index < s_scroll || absolute_index >= s_scroll + k_visible_rows) {
        return;
    }
    int y = k_list_y0 + (absolute_index - s_scroll) * k_row_h;
    uint16_t bg = selected ? COL_ACCENT : COL_BG;
    uint16_t fg = selected ? COL_BG : COL_FG;
    char line[40];

    if (s_screen == SCREEN_ROOT) {
        if (absolute_index < 0 || absolute_index >= k_root_count) return;
        format_root_row(absolute_index, line, sizeof(line));
        panel_draw_text_in_row(y, k_row_h, 8, line, fg, bg);
        return;
    }
    if (s_screen == SCREEN_NODES && s_cb.list_nodes) {
        hsg_panel_node_info_t nodes[HSG_PANEL_MAX_NODES];
        int n = s_cb.list_nodes(nodes, HSG_PANEL_MAX_NODES);
        if (absolute_index < 0 || absolute_index >= n) return;
        format_node_row(&nodes[absolute_index], line, sizeof(line));
        panel_draw_text_in_row(y, k_row_h, 8, line, fg, bg);
    }
}

static void paint_visible_list(int count)
{
    /* Clear list band only (not full screen). */
    int list_h = k_visible_rows * k_row_h;
    panel_draw_fill_rect(0, k_list_y0, PANEL_W, list_h, COL_BG);
    for (int i = s_scroll; i < count && i < s_scroll + k_visible_rows; ++i) {
        paint_row(i, i == s_sel);
    }
}

static void draw_root(void)
{
    panel_draw_clear(COL_BG);
    draw_header("HSG Panel");
    clamp_sel(k_root_count, nullptr);
    paint_visible_list(k_root_count);
    draw_hint("Rotate  Press  Hold: back");
}

static void draw_outputs(void)
{
    panel_draw_clear(COL_BG);

    int mapped = load_hub_pads();
    if (mapped <= 0) {
        paint_output_status();
        panel_draw_text(8, k_mod_y0, "No mapped outputs", COL_WARN, COL_BG);
        draw_hint("Hold: back");
        return;
    }

    /* Prefer first mapped pad if current selection is unmapped. */
    if (s_sel < 0 || s_sel >= k_hub_outputs || !s_pad_mapped[s_sel]) {
        s_sel = 0;
        for (int i = 0; i < k_hub_outputs; ++i) {
            if (s_pad_mapped[i]) {
                s_sel = i;
                break;
            }
        }
    }

    paint_output_status();
    for (int m = 0; m < 4; ++m) {
        paint_module_frame(m);
    }
    for (int i = 0; i < k_hub_outputs; ++i) {
        paint_output_pad(i, i == s_sel);
    }
    draw_hint("Press: toggle  Hold: back");
}

static void draw_nodes(void)
{
    hsg_panel_node_info_t nodes[HSG_PANEL_MAX_NODES];
    int n = s_cb.list_nodes ? s_cb.list_nodes(nodes, HSG_PANEL_MAX_NODES) : 0;

    panel_draw_clear(COL_BG);
    draw_header("CAN Nodes");
    if (n <= 0) {
        panel_draw_text(8, k_list_y0, "No nodes yet", COL_WARN, COL_BG);
        draw_hint("Hold: back");
        return;
    }
    clamp_sel(n, nullptr);
    paint_visible_list(n);
    draw_hint("Hold: back");
}

static void draw_network(void)
{
    char ip[48] = "-";
    bool link = false;
    if (s_cb.get_network) {
        s_cb.get_network(ip, sizeof(ip), &link);
    }
    const int lh = panel_draw_char_height() + 8;
    int y = k_list_y0;

    panel_draw_clear(COL_BG);
    draw_header("Network");
    panel_draw_text(8, y, link ? "Link: UP" : "Link: DOWN",
                    link ? COL_ACCENT : COL_WARN, COL_BG);
    y += lh;
    char line[64];
    snprintf(line, sizeof(line), "IP: %.40s", ip);
    panel_draw_text(8, y, line, COL_FG, COL_BG);
    y += lh;
    if (s_cb.fw_short_name) {
        snprintf(line, sizeof(line), "FW: %.24s", s_cb.fw_short_name);
        panel_draw_text(8, y, line, COL_DIM, COL_BG);
        y += lh;
    }
    panel_draw_text(8, y, "Board: Olimex-POE", COL_DIM, COL_BG);
    draw_hint("Hold: back");
}

static void draw_about(void)
{
    const int lh = panel_draw_char_height() + 10;
    int y = k_list_y0 + 8;
    panel_draw_clear(COL_BG);
    draw_header("About");
    panel_draw_text_centered(y, "HSG LED Hub", COL_FG, COL_BG);
    y += lh;
    if (s_cb.fw_short_name) {
        panel_draw_text_centered(y, s_cb.fw_short_name, COL_DIM, COL_BG);
        y += lh;
    }
    panel_draw_text_centered(y, "Commissioning panel", COL_DIM, COL_BG);
    y += lh;
    panel_draw_text_centered(y, "ST7789 + EC11", COL_DIM, COL_BG);
    draw_hint("Hold: back");
}

static void redraw(void)
{
    switch (s_screen) {
    case SCREEN_ROOT: draw_root(); break;
    case SCREEN_OUTPUTS: draw_outputs(); break;
    case SCREEN_NODES: draw_nodes(); break;
    case SCREEN_NETWORK: draw_network(); break;
    case SCREEN_ABOUT: draw_about(); break;
    }
}

static int current_list_count(void)
{
    if (s_screen == SCREEN_ROOT) return k_root_count;
    if (s_screen == SCREEN_OUTPUTS) return k_hub_outputs;
    if (s_screen == SCREEN_NODES) {
        hsg_panel_node_info_t tmp[HSG_PANEL_MAX_NODES];
        return s_cb.list_nodes ? s_cb.list_nodes(tmp, HSG_PANEL_MAX_NODES) : 0;
    }
    return 0;
}

static bool screen_is_list(void)
{
    return s_screen == SCREEN_ROOT || s_screen == SCREEN_OUTPUTS || s_screen == SCREEN_NODES;
}

static int next_mapped_pad(int from, int dir)
{
    for (int step = 0; step < k_hub_outputs; ++step) {
        from += dir;
        if (from < 0) from = k_hub_outputs - 1;
        if (from >= k_hub_outputs) from = 0;
        if (s_pad_mapped[from]) return from;
    }
    return from;
}

static void on_rotate(int8_t delta)
{
    if (!screen_is_list() || delta == 0) {
        return;
    }

    if (s_screen == SCREEN_OUTPUTS) {
        if (s_pad_count <= 0) return;
        int old_sel = s_sel;
        s_sel = next_mapped_pad(s_sel, (delta > 0) ? 1 : -1);
        if (old_sel != s_sel) {
            paint_output_pad(old_sel, false);
            paint_output_pad(s_sel, true);
            paint_output_status();
        }
        return;
    }

    int n = current_list_count();
    if (n <= 0) return;

    int old_sel = s_sel;
    /* Encoder already emits ±1 per detent — move exactly one row. */
    s_sel += (delta > 0) ? 1 : -1;

    int scroll_changed = 0;
    clamp_sel(n, &scroll_changed);

    if (scroll_changed) {
        paint_visible_list(n);
        return;
    }
    if (old_sel != s_sel) {
        paint_row(old_sel, false);
        paint_row(s_sel, true);
    }
}

static void go_back(void)
{
    if (s_screen == SCREEN_ROOT) return;
    s_screen = SCREEN_ROOT;
    s_sel = 0;
    s_scroll = 0;
    redraw();
}

static void on_click(void)
{
    if (s_screen == SCREEN_ROOT) {
        switch (s_sel) {
        case 0: s_screen = SCREEN_OUTPUTS; break;
        case 1: s_screen = SCREEN_NODES; break;
        case 2: s_screen = SCREEN_NETWORK; break;
        case 3: s_screen = SCREEN_ABOUT; break;
        default: break;
        }
        s_sel = 0;
        s_scroll = 0;
        redraw();
        return;
    }
    if (s_screen == SCREEN_OUTPUTS && s_cb.toggle_output) {
        if (s_sel < 0 || s_sel >= k_hub_outputs || !s_pad_mapped[s_sel]) return;
        s_cb.toggle_output(s_pad_id[s_sel]);
        load_hub_pads();
        paint_output_pad(s_sel, true);
        paint_output_status();
    }
}

static void refresh_outputs(void)
{
    bool old_mapped[k_hub_outputs];
    bool old_on[k_hub_outputs];
    int old_id[k_hub_outputs];
    const int old_count = s_pad_count;

    for (int i = 0; i < k_hub_outputs; ++i) {
        old_mapped[i] = s_pad_mapped[i];
        old_on[i] = s_pad_on[i];
        old_id[i] = s_pad_id[i];
    }

    load_hub_pads();

    bool mapping_changed = (old_count != s_pad_count);
    for (int i = 0; i < k_hub_outputs && !mapping_changed; ++i) {
        mapping_changed = old_mapped[i] != s_pad_mapped[i] ||
                          old_id[i] != s_pad_id[i];
    }

    if (mapping_changed) {
        /* Mapping changes can alter selection and the empty-page state. */
        draw_outputs();
        return;
    }

    bool selected_changed = false;
    for (int i = 0; i < k_hub_outputs; ++i) {
        if (old_on[i] != s_pad_on[i]) {
            paint_output_pad(i, i == s_sel);
            if (i == s_sel) selected_changed = true;
        }
    }
    if (selected_changed) {
        paint_output_status();
    }
}

static void panel_task(void *arg)
{
    (void)arg;
    redraw();
    int64_t last_net_refresh = 0;
    int64_t last_output_refresh = 0;

    for (;;) {
        panel_evt_t ev;
        panel_encoder_poll_button();

        if (xQueueReceive(s_evt_q, &ev, pdMS_TO_TICKS(40)) == pdTRUE) {
            /* Coalesce burst rotate events into one UI update. */
            if (ev.type == PANEL_EVT_ROTATE) {
                int delta = ev.delta;
                panel_evt_t more;
                while (xQueueReceive(s_evt_q, &more, 0) == pdTRUE) {
                    if (more.type == PANEL_EVT_ROTATE) {
                        delta += more.delta;
                    } else {
                        /* Process non-rotate after applying net rotation. */
                        if (delta != 0) {
                            int step = (delta > 0) ? 1 : -1;
                            int n = (delta > 0) ? delta : -delta;
                            for (int i = 0; i < n; ++i) {
                                on_rotate((int8_t)step);
                            }
                            delta = 0;
                        }
                        if (more.type == PANEL_EVT_CLICK) {
                            on_click();
                        } else if (more.type == PANEL_EVT_LONG_PRESS) {
                            go_back();
                        }
                    }
                }
                if (delta != 0) {
                    int step = (delta > 0) ? 1 : -1;
                    int n = (delta > 0) ? delta : -delta;
                    /* Cap to avoid long blocking paints if queue was full. */
                    if (n > 8) n = 8;
                    for (int i = 0; i < n; ++i) {
                        on_rotate((int8_t)step);
                    }
                }
            } else if (ev.type == PANEL_EVT_CLICK) {
                on_click();
            } else if (ev.type == PANEL_EVT_LONG_PRESS) {
                go_back();
            }
            continue;
        }

        if (s_screen == SCREEN_NETWORK) {
            int64_t now = (int64_t)xTaskGetTickCount() * (int64_t)portTICK_PERIOD_MS;
            if (now - last_net_refresh >= 1000) {
                last_net_refresh = now;
                redraw();
            }
        } else if (s_screen == SCREEN_OUTPUTS) {
            int64_t now = (int64_t)xTaskGetTickCount() * (int64_t)portTICK_PERIOD_MS;
            if (now - last_output_refresh >= 250) {
                last_output_refresh = now;
                refresh_outputs();
            }
        }
    }
}

void hsg_panel_start(const hsg_panel_callbacks_t *cb)
{
    if (!cb) {
        ESP_LOGW(TAG, "No callbacks; panel disabled");
        return;
    }
    s_cb = *cb;

    if (panel_hw_init() != ESP_OK) {
        ESP_LOGE(TAG, "Display init failed — panel disabled");
        return;
    }
    panel_draw_init(panel_hw_panel());

    s_evt_q = xQueueCreate(32, sizeof(panel_evt_t));
    if (!s_evt_q) {
        ESP_LOGE(TAG, "Event queue alloc failed");
        return;
    }
    if (panel_encoder_init(s_evt_q) != ESP_OK) {
        ESP_LOGE(TAG, "Encoder init failed — panel disabled");
        return;
    }

    BaseType_t ok = xTaskCreate(panel_task, "hsg_panel", 8192, nullptr, 5, nullptr);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to start panel task");
        return;
    }
    ESP_LOGI(TAG, "Commissioning panel started");
}

#else

void hsg_panel_start(const hsg_panel_callbacks_t *cb)
{
    (void)cb;
}

#endif
