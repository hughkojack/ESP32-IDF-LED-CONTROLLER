#pragma once
#include <cstdint>
#include <functional>
#include <string>
#include "esp_http_server.h"
#include "cJSON.h"

// Minimal CAN frame (compatible with your MCP2515 struct)
struct HSG_CanFrame {
    uint32_t id = 0;
    uint8_t  dlc = 0;
    uint8_t  data[8]{};
};

namespace HSG {
namespace API {

struct Init {
    int i2c_port = 0;                 // I2C_NUM_0 or I2C_NUM_1 (your I2C must be initialized by app)
    uint32_t i2c_scan_start = 0x40;   // first 7-bit addr to scan (default PCA9685 range)
    uint32_t i2c_scan_end   = 0x7F;   // last 7-bit addr to scan

    // output_cb(out, brightness[0..100], fade_ms)
    std::function<void(int out, int brightness, int fade_ms)> output_cb;

    // group_cb(name, state["ON"/"OFF"], fade_ms)
    std::function<void(const char* name, const char* state, int fade_ms)> group_cb;

};

esp_err_t register_uris(httpd_handle_t server, const Init& init);

// Call once after networking is up (Wi-Fi or ETH has an IP).
// Starts the HTTP server (port 80) and registers all /api/* handlers.
esp_err_t start(const Init& init);

// Stop HTTP server (optional)
void stop();

// Update the "last CAN frame" cache (exposed at GET /api/can/last)
void update_last_can(const HSG_CanFrame& f);

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

} // namespace API
} // namespace HSG
