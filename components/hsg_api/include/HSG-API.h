#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include "esp_http_server.h"
#include "cJSON.h"
#include "freertos/event_groups.h"
#include "mqtt_client.h"

// Minimal CAN frame (compatible with your MCP2515 struct)
struct HSG_CanFrame {
    uint32_t id = 0;
    uint8_t  dlc = 0;
    uint8_t  data[8]{};
};

const int CONFIG_LOADED_BIT = BIT2;

namespace HSG {
namespace API {

EventGroupHandle_t get_event_group();

struct Init {
    int i2c_port = 0;                 // I2C_NUM_0 or I2C_NUM_1 (your I2C must be initialized by app)
    uint32_t i2c_scan_start = 0x40;   // first 7-bit addr to scan (default PCA9685 range)
    uint32_t i2c_scan_end   = 0x7F;   // last 7-bit addr to scan

    // output_cb(out, brightness[0..100], fade_ms)
    std::function<void(int out, int brightness, int fade_ms, const char* state)> output_cb;

    // group_cb(name, state["ON"/"OFF"], fade_ms)
    std::function<void(const char* name, int brightness, int fade_ms, const char* state)> group_cb;

    // NEW: Callback to notify main that config has been updated
    std::function<void()> config_updated_cb;
    std::function<cJSON*()> get_outputs_json_cb;
    // Callback to get nodes list (returns JSON array of nodes; may merge config + live for display)
    std::function<cJSON*()> get_nodes_json_cb;
    // When saving config (POST /api/config), use this if set to avoid re-persisting removed nodes from stale config. Returns g_nodes only.
    std::function<cJSON*()> get_nodes_json_for_config_save_cb;

    // Send node config over CAN (hub -> node). target_node_id 1..126 or NODE_ID_UNCONFIGURED (127).
    // cmd = CMD_SET_NODE_ID etc., data/len = payload after cmd byte (can be null/0).
    std::function<void(uint8_t target_node_id, uint8_t cmd, const uint8_t* data, size_t len)> send_node_config_cb;
    // Remove a node from the hub list (e.g. offline/stale). node_id 1..126 or 127. Returns true if removed.
    std::function<bool(uint8_t node_id)> remove_node_cb;
    // Set per-input label for a node (hub stores, persists, and sends CMD_SET_INPUT_LABEL to node). label can be empty.
    std::function<void(uint8_t node_id, uint8_t input_index, const char* label)> set_node_input_label_cb;
};

esp_err_t register_uris(httpd_handle_t server, const Init& init);

// Call once after networking is up (Wi-Fi or ETH has an IP).
// Starts the HTTP server (port 80) and registers all /api/* handlers.
esp_err_t start(const Init& init);
void stop();

// --- MQTT Client Functions ---
esp_err_t mqtt_start();
void mqtt_stop();

// Update the "last CAN frame" cache (exposed at GET /api/can/last)
void add_to_can_history(const HSG_CanFrame& f);

// Accessors for stored JSON config (NVS), useful if app needs them
// The JSON schema is:
// { "config": { "i2c": { "pca9685": { "0x40": [16 ints], "0x41": [...] } },
//               "groups": { "name":[outputs...] },
//               "mqtt": { "broker": "...", "port": 1883, "topicPrefix": "...", "clientId": "...", "username":"...", "password":"..." } } }
std::string get_config_json();             // returns full JSON string (pretty-printed)
cJSON* get_config_json_obj();  // caller must free with cJSON_Delete()

esp_err_t   set_config_json(const char*);  // replaces the stored JSON (validates minimal schema)

// Convenience: read MQTT subtree as separate JSON (string)
std::string get_mqtt_json();

// Scans I2C bus for PCA9685 devices, adds defaults to config if missing,
// and removes any PCA9685 entries from config that are no longer present on bus.
esp_err_t scan_and_prune_i2c(int i2c_port);
esp_mqtt_client_handle_t get_mqtt_client();
void send_ws_message(const std::string& msg);

} // namespace API
} // namespace HSG
