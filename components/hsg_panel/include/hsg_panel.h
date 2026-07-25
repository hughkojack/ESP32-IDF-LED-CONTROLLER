#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HSG_PANEL_MAX_OUTPUTS 48
#define HSG_PANEL_MAX_NODES   32

typedef struct {
    int id;            /* logical output 1..N */
    int brightness;    /* 0-100 last/current ON level */
    bool on;           /* target PWM > 0 */
} hsg_panel_output_info_t;

typedef struct {
    uint8_t node_id;
    uint8_t node_type;
    bool online;
    int64_t last_seen_us;
} hsg_panel_node_info_t;

typedef struct {
    /** Toggle one mapped output (ON/OFF). Return true on success. */
    bool (*toggle_output)(int output_id);
    /** Fill mapped outputs; return count. */
    int (*list_outputs)(hsg_panel_output_info_t *out, int max_count);
    /** Fill known CAN nodes; return count. */
    int (*list_nodes)(hsg_panel_node_info_t *out, int max_count);
    /** Write IP string and link flag. */
    void (*get_network)(char *ip, size_t ip_len, bool *link_up);
    const char *fw_short_name;
} hsg_panel_callbacks_t;

/**
 * Start commissioning panel UI task (Olimex-POE only).
 * No-op on other boards or if hardware init fails.
 */
void hsg_panel_start(const hsg_panel_callbacks_t *cb);

#ifdef __cplusplus
}
#endif
