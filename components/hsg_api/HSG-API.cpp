#include "HSG-API.h"
//#include "hsg_outputs.h"
#include "mqtt_client.h"

#include <cstring>
#include <string>
#include <vector>
#include <unordered_set>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/i2c_master.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/opt.h"
#include "hardware_config.h"
#include "can_protocol.h"

static const char* TAG = "HSG-API";
static const char* NVS_NS = "cfg";
static const char* NVS_KEY_CONFIG = "app_cfg";

extern const char _binary_ESP32_POE_html_start[] asm("_binary_ESP32_POE_html_start");
extern const char _binary_ESP32_POE_html_end[]   asm("_binary_ESP32_POE_html_end");

extern const uint8_t _binary_favicon_ico_start[] asm("_binary_favicon_ico_start");
extern const uint8_t _binary_favicon_ico_end[]   asm("_binary_favicon_ico_end");

extern const char _binary_control_html_start[] asm("_binary_control_html_start");
extern const char _binary_control_html_end[]   asm("_binary_control_html_end");

extern i2c_master_bus_handle_t i2c_bus_handle;

namespace {

httpd_handle_t g_http = nullptr;
HSG::API::Init g_init{};
HSG_CanFrame g_last{};
SemaphoreHandle_t g_last_mutex = nullptr;
esp_mqtt_client_handle_t g_mqtt_client = nullptr;
static std::string g_mqtt_command_topic;
int g_ws_fd = -1; 
//static esp_err_t h_websocket(httpd_req_t *req);
const size_t MAX_CAN_HISTORY = 50; // Store the last 50 messages
static std::vector<HSG_CanFrame> g_can_history;
static SemaphoreHandle_t g_history_mutex = nullptr;

// ------------------ small helpers ------------------
static esp_err_t send_json(httpd_req_t *req, cJSON *root) {
    char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out ? out : "{}");
    if (out) free(out);
    cJSON_Delete(root);
    return ESP_OK;
}
static std::string req_read_all(httpd_req_t* req) {
    std::string body; body.resize(req->content_len);
    size_t got = 0;
    while (got < body.size()) {
        int r = httpd_req_recv(req, &body[got], body.size() - got);
        if (r <= 0) { body.resize(got); break; }
        got += r;
    }
    return body;
}

// NVS JSON load/save
static cJSON* load_cfg_json() {
    nvs_handle_t h; size_t len = 0;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return cJSON_CreateObject();
    if (nvs_get_blob(h, NVS_KEY_CONFIG, nullptr, &len) != ESP_OK || len == 0) { nvs_close(h); return cJSON_CreateObject(); }
    std::string buf(len, '\0');
    if (nvs_get_blob(h, NVS_KEY_CONFIG, buf.data(), &len) != ESP_OK) { nvs_close(h); return cJSON_CreateObject(); }
    nvs_close(h);
    cJSON* root = cJSON_Parse(buf.c_str());
    return root ? root : cJSON_CreateObject();
}
static esp_err_t save_cfg_json(cJSON* root) {
    if (!root) return ESP_ERR_INVALID_ARG;
    char* text = cJSON_PrintUnformatted(root);
    if (!text) return ESP_ERR_NO_MEM;
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) { free(text); return err; }
    err = nvs_set_blob(h, NVS_KEY_CONFIG, text, strlen(text));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    free(text);
    return err;
}
static void ensure_layout(cJSON* root) {
    cJSON* config = cJSON_GetObjectItem(root, "config");
    if (!config) cJSON_AddItemToObject(root, "config", config = cJSON_CreateObject());
    cJSON* i2c = cJSON_GetObjectItem(config, "i2c");
    if (!i2c) cJSON_AddItemToObject(config, "i2c", i2c = cJSON_CreateObject());
    cJSON* pca = cJSON_GetObjectItem(i2c, "pca9685");
    if (!pca) cJSON_AddItemToObject(i2c, "pca9685", pca = cJSON_CreateObject());
    cJSON* groups = cJSON_GetObjectItem(config, "groups");
    if (!groups) cJSON_AddItemToObject(config, "groups", groups = cJSON_CreateObject());
    cJSON* mqtt = cJSON_GetObjectItem(config, "mqtt");
    if (!mqtt) cJSON_AddItemToObject(config, "mqtt", mqtt = cJSON_CreateObject());
    cJSON* nodes = cJSON_GetObjectItem(config, "nodes");
    if (!nodes) cJSON_AddItemToObject(config, "nodes", nodes = cJSON_CreateArray());
}

// Probe an I2C address using the ESP-IDF v5+ I²C master API
static esp_err_t i2c_probe(i2c_master_bus_handle_t bus, uint8_t addr7) {
    // i2c_master_probe() returns ESP_OK if the device ACKs
    esp_err_t ret = i2c_master_probe(bus, addr7, pdMS_TO_TICKS(100));
    if (ret == ESP_OK) {
        ESP_LOGI("HSG-API", "Found I2C device at 0x%02X", addr7);
    }
    return ret;
}

static std::vector<uint8_t> scan_pca9685_addrs(i2c_port_t port) {
    std::vector<uint8_t> found;
    for (uint8_t addr = 0x40; addr <= 0x47; ++addr) {
        if (addr == 0x70) continue; // ignore All-Call
        if (i2c_probe(i2c_bus_handle, addr) == ESP_OK) {
//            ESP_LOGI(TAG, "Found I2C device at 0x%02X", addr);
            found.push_back(addr);
        }
    }
    return found;
}

// --- Helper function to get a human-readable uptime string ---
static std::string get_uptime_string() {
    int64_t uptime_us = esp_timer_get_time(); // Time since boot in microseconds
    int64_t uptime_s = uptime_us / 1000000;

    int days = uptime_s / (60 * 60 * 24);
    int hours = (uptime_s % (60 * 60 * 24)) / (60 * 60);
    int minutes = (uptime_s % (60 * 60)) / 60;
    int seconds = uptime_s % 60;

    char buf[100];
    snprintf(buf, sizeof(buf), "%d days, %02d:%02d:%02d", days, hours, minutes, seconds);
    return std::string(buf);
}

// --- Helper function to get the last reset reason as a string ---
static const char* get_reset_reason_string() {
    esp_reset_reason_t reason = esp_reset_reason();
    switch (reason) {
        case ESP_RST_UNKNOWN:   return "Unknown";
        case ESP_RST_POWERON:   return "Power-on";
        case ESP_RST_EXT:       return "External Pin";
        case ESP_RST_SW:        return "Software Reset";
        case ESP_RST_PANIC:     return "Panic / Exception";
        case ESP_RST_INT_WDT:   return "Interrupt Watchdog";
        case ESP_RST_TASK_WDT:  return "Task Watchdog"; // <-- This is the one!
        case ESP_RST_WDT:       return "Other Watchdog";
        case ESP_RST_DEEPSLEEP: return "Deep Sleep Wakeup";
        case ESP_RST_BROWNOUT:  return "Brownout (Voltage Dip)";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "Other";
    }
}

// ------------------ HTTP handlers ------------------
static esp_err_t h_favicon(httpd_req_t* req)
{
    size_t len = _binary_favicon_ico_end - _binary_favicon_ico_start;
    httpd_resp_set_type(req, "image/x-icon");
    return httpd_resp_send(req, (const char*)_binary_favicon_ico_start, len);
}

// GET /
static esp_err_t h_root(httpd_req_t* req) {
    size_t len = _binary_control_html_end - _binary_control_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, _binary_control_html_start, len);
    return ESP_OK;
}

static esp_err_t h_config_page(httpd_req_t* req) {
    size_t len = _binary_ESP32_POE_html_end - _binary_ESP32_POE_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, _binary_ESP32_POE_html_start, len);
    return ESP_OK;
}


// GET /api/adopt
static esp_err_t h_adopt(httpd_req_t* req) {
    cJSON* root = cJSON_CreateObject();

    // firmware
    cJSON* fw = cJSON_CreateObject();
    cJSON_AddStringToObject(fw, "name", "HSG Light Controller");
    cJSON_AddStringToObject(fw, "version", "1.0.0");
    cJSON_AddStringToObject(fw, "maker", "HSG");
    cJSON_AddItemToObject(root, "firmware", fw);

    // network
    cJSON* net = cJSON_CreateObject();
    if (auto* def = esp_netif_get_default_netif()) {
        const char* if_key = esp_netif_get_ifkey(def);
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(def, &ip) == ESP_OK) {
            char ipstr[16]; 
            snprintf(ipstr, sizeof ipstr, IPSTR, IP2STR(&ip.ip));
            cJSON_AddStringToObject(net, "ip", ipstr);
            cJSON_AddStringToObject(net, "activeInterface", if_key);
        }
        if (strcmp(if_key, "WIFI_STA_DEF") == 0) {
            wifi_ap_record_t ap_info;
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                cJSON_AddStringToObject(net, "ssid", (char*)ap_info.ssid);
                cJSON_AddNumberToObject(net, "rssi", ap_info.rssi); // Signal strength
            }
        }
    }
    uint8_t mac[6]; 
    esp_read_mac(mac, ESP_MAC_ETH); // prefer ETH MAC
    char macstr[18]; 
    snprintf(macstr, sizeof macstr, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    cJSON_AddStringToObject(net, "mac", macstr);
    cJSON_AddStringToObject(net, "mode", "ETH/WIFI failover");
    cJSON_AddItemToObject(root, "network", net);

    // system
    cJSON* sys = cJSON_CreateObject();
    cJSON_AddNumberToObject(sys, "heapFreeBytes", esp_get_free_heap_size());
    cJSON_AddNumberToObject(sys, "heapMinFree", esp_get_minimum_free_heap_size());
    if (auto* running = esp_ota_get_running_partition()) {
        cJSON_AddNumberToObject(sys, "sketchSpaceUsedBytes", running->size);
    }
    cJSON_AddStringToObject(sys, "uptime", get_uptime_string().c_str());
    cJSON_AddStringToObject(sys, "lastResetReason", get_reset_reason_string());
    cJSON_AddBoolToObject(sys, "wsConnected", (g_ws_fd != -1));
    
    cJSON_AddItemToObject(root, "system", sys);

    return send_json(req, root);
}

// GET /api/config  (returns ONLY the "config" object)
static esp_err_t h_config_get(httpd_req_t* req) {
    cJSON* root = load_cfg_json();
    ensure_layout(root);
    cJSON* config = cJSON_GetObjectItem(root, "config");
    char* out = cJSON_PrintUnformatted(config ? config : root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t res = httpd_resp_sendstr(req, out ? out : "{}");
    if (out) free(out);
    cJSON_Delete(root);
    return res;
}

// POST /api/config
static esp_err_t h_config_post(httpd_req_t* req) {
    auto body = req_read_all(req);
    ESP_LOGI(TAG, "Config POST body: %s", body.c_str());

    cJSON* posted_config = cJSON_Parse(body.c_str());
    if (!posted_config) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
    }

    // Load the full existing configuration from NVS
    cJSON* full_config_root = load_cfg_json();
    ensure_layout(full_config_root);
    cJSON* config_obj = cJSON_GetObjectItem(full_config_root, "config");

    // This loop iterates through all the keys in the posted JSON
    // (wifi, mqtt, i2c, groups, etc.) and updates them in the main config.
    // Skip "nodes" — we set it from the live node list below so config save never overwrites with stale client data.
    for (cJSON* new_section = posted_config->child; new_section != NULL; new_section = new_section->next) {
        if (strcmp(new_section->string, "nodes") == 0) continue;
        // Remove the old section if it exists
        if (cJSON_HasObjectItem(config_obj, new_section->string)) {
            cJSON_DeleteItemFromObject(config_obj, new_section->string);
        }
        // Add the new (or updated) section
        cJSON_AddItemToObject(config_obj, new_section->string, cJSON_Duplicate(new_section, 1));
        ESP_LOGI(TAG, "Updated config section: %s", new_section->string);
    }

    // Persist the live node list (from g_nodes) so we never overwrite with stale client snapshot
    if (g_init.get_nodes_json_for_config_save_cb) {
        cJSON* nodes_to_save = g_init.get_nodes_json_for_config_save_cb();
        if (nodes_to_save) {
            cJSON_DeleteItemFromObject(config_obj, "nodes");
            cJSON_AddItemToObject(config_obj, "nodes", nodes_to_save);
        }
    } else if (g_init.get_nodes_json_cb) {
        cJSON* live_nodes = g_init.get_nodes_json_cb();
        if (live_nodes) {
            cJSON_DeleteItemFromObject(config_obj, "nodes");
            cJSON_AddItemToObject(config_obj, "nodes", live_nodes);
        }
    }

    // Save the newly merged configuration back to NVS
    save_cfg_json(full_config_root);
    cJSON_Delete(posted_config);
    cJSON_Delete(full_config_root);
   
    // Reload the output mapping in case it changed
//    hsg_outputs_reload_config();

    // NEW: Notify main that the config has changed
    if (g_init.config_updated_cb) {
        g_init.config_updated_cb();
    }

    return httpd_resp_sendstr(req, "OK");
}

// GET /api/mqtt
static esp_err_t h_mqtt_get(httpd_req_t* req) {
    cJSON* full = load_cfg_json(); ensure_layout(full);
    cJSON* cfg  = cJSON_GetObjectItem(full, "config");
    cJSON* mqtt = cJSON_GetObjectItem(cfg, "mqtt");
    cJSON* out  = cJSON_Duplicate(mqtt ? mqtt : cJSON_CreateObject(), 1);
    cJSON_Delete(full);
    return send_json(req, out);
}

// POST /api/mqtt
/*
static esp_err_t h_mqtt_post(httpd_req_t* req) {
    auto body = req_read_all(req);
    cJSON* posted = cJSON_Parse(body.c_str());
    if (!posted) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");

    cJSON* full = load_cfg_json(); ensure_layout(full);
    cJSON* cfg  = cJSON_GetObjectItem(full, "config");
    cJSON_DeleteItemFromObject(cfg, "mqtt");
    cJSON_AddItemToObject(cfg, "mqtt", cJSON_Duplicate(posted, 1));
    cJSON_Delete(posted);
    save_cfg_json(full);
    cJSON_Delete(full);
    return httpd_resp_sendstr(req, "OK");
}*/

// POST /api/command
static esp_err_t h_command(httpd_req_t* req) {
    auto body = req_read_all(req);
    cJSON* cmd = cJSON_Parse(body.c_str());
    if (!cmd) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
    
    ESP_LOGI("HSG-API", "Command received: %s", body.c_str());

    if (cJSON_IsTrue(cJSON_GetObjectItem(cmd, "restart"))) {
        cJSON_Delete(cmd);
        httpd_resp_sendstr(req, "OK");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return ESP_OK;
    }

    int fade = 0;
    if (auto* v = cJSON_GetObjectItem(cmd, "fade"); cJSON_IsNumber(v)) fade = v->valueint;

    const cJSON *state_json = cJSON_GetObjectItem(cmd, "state");
    const char* state_str = cJSON_IsString(state_json) ? state_json->valuestring : nullptr;
    
    int brightness = 0; // Default to 0
    const cJSON *brightness_json = cJSON_GetObjectItem(cmd, "brightness");
    if (cJSON_IsNumber(brightness_json)) {
        brightness = brightness_json->valueint;
    }
    
    if (auto* out = cJSON_GetObjectItem(cmd, "output"); cJSON_IsNumber(out)) {
        if (g_init.output_cb) g_init.output_cb(out->valueint, brightness, fade, state_str);
    } else if (auto* grp = cJSON_GetObjectItem(cmd, "group"); cJSON_IsString(grp)) {
        if (g_init.group_cb) g_init.group_cb(grp->valuestring, brightness, fade, state_str);
    }

    cJSON_Delete(cmd);
    return httpd_resp_sendstr(req, "OK");
}

// POST /api/node/config - send hub->node config (set node ID, find-me, etc.)
// Body: { "target_node_id": 127|1..126, "command": "set_node_id"|"find_me"|"set_find_me_output"|..., ...params }
static esp_err_t h_node_config_post(httpd_req_t* req) {
    if (!g_init.send_node_config_cb) {
        return httpd_resp_send_err(req, HTTPD_501_METHOD_NOT_IMPLEMENTED, "send_node_config_cb not set");
    }
    auto body = req_read_all(req);
    cJSON* root = cJSON_Parse(body.c_str());
    if (!root) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");

    int target = NODE_ID_UNCONFIGURED;
    if (auto* t = cJSON_GetObjectItem(root, "target_node_id"); cJSON_IsNumber(t))
        target = t->valueint;
    if (target < 0 || target > 127) {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "target_node_id 0..127");
    }
    uint8_t target_node_id = (uint8_t)target;

    const char* cmd_str = nullptr;
    if (auto* c = cJSON_GetObjectItem(root, "command"); cJSON_IsString(c))
        cmd_str = c->valuestring;
    if (!cmd_str) {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "command required");
    }

    uint8_t payload[8];
    size_t plen = 0;
    uint8_t cmd_byte = 0;

    if (strcmp(cmd_str, "set_node_id") == 0) {
        cmd_byte = CMD_SET_NODE_ID;
        if (auto* v = cJSON_GetObjectItem(root, "new_id"); cJSON_IsNumber(v) && v->valueint >= 1 && v->valueint <= 126) {
            payload[0] = (uint8_t)v->valueint;
            plen = 1;
        } else {
            cJSON_Delete(root);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "new_id 1..126 required");
        }
    } else if (strcmp(cmd_str, "find_me") == 0) {
        cmd_byte = CMD_FIND_ME;
        int dur = 5;
        if (auto* v = cJSON_GetObjectItem(root, "duration_min"); cJSON_IsNumber(v)) dur = v->valueint;
        if (dur < 1) dur = 1;
        if (dur > 30) dur = 30;
        payload[0] = (uint8_t)dur;
        plen = 1;
    } else if (strcmp(cmd_str, "set_find_me_output") == 0) {
        cmd_byte = CMD_SET_FIND_ME_OUTPUT;
        if (auto* v = cJSON_GetObjectItem(root, "output_index"); cJSON_IsNumber(v)) {
            payload[0] = (uint8_t)(v->valueint & 0xFF);
            plen = 1;
        } else {
            cJSON_Delete(root);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "output_index required");
        }
    } else if (strcmp(cmd_str, "set_input_count") == 0) {
        cmd_byte = CMD_SET_INPUT_COUNT;
        if (auto* v = cJSON_GetObjectItem(root, "count"); cJSON_IsNumber(v) && v->valueint >= 1 && v->valueint <= 6) {
            payload[0] = (uint8_t)v->valueint;
            plen = 1;
        } else {
            cJSON_Delete(root);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "count 1..6 required");
        }
    } else if (strcmp(cmd_str, "set_input_cfg") == 0) {
        cmd_byte = CMD_SET_INPUT_CFG;
        auto* idx = cJSON_GetObjectItem(root, "input_index");
        auto* id = cJSON_GetObjectItem(root, "input_id");
        auto* mode = cJSON_GetObjectItem(root, "mode");
        int mode_val = -1;
        if (cJSON_IsNumber(mode))
            mode_val = (mode->valueint & 1);
        else if (cJSON_IsString(mode)) {
            if (strcmp(mode->valuestring, "momentary") == 0) mode_val = 0;
            else if (strcmp(mode->valuestring, "toggle") == 0) mode_val = 1;
        }
        if (cJSON_IsNumber(idx) && cJSON_IsNumber(id) && idx->valueint >= 0 && idx->valueint < 6 && mode_val >= 0) {
            payload[0] = (uint8_t)idx->valueint;
            payload[1] = (uint8_t)(id->valueint & 0xFF);
            payload[2] = (uint8_t)mode_val;
            plen = 3;
            auto* gpio_item = cJSON_GetObjectItem(root, "gpio");
            if (cJSON_IsNumber(gpio_item) && ((gpio_item->valueint >= 0 && gpio_item->valueint <= 48) || gpio_item->valueint == 255)) {
                payload[3] = (uint8_t)(gpio_item->valueint & 0xFF);
                plen = 4;
            }
        } else {
            cJSON_Delete(root);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "input_index 0..5, input_id, mode (0/1 or momentary/toggle) required");
        }
        // Optional label: update hub state and send CMD_SET_INPUT_LABEL to node
        if (g_init.set_node_input_label_cb) {
            const char* label_str = nullptr;
            if (auto* lab = cJSON_GetObjectItem(root, "label"); cJSON_IsString(lab) && lab->valuestring)
                label_str = lab->valuestring;
            g_init.set_node_input_label_cb(target_node_id, (uint8_t)idx->valueint, label_str ? label_str : "");
        }
    } else if (strcmp(cmd_str, "set_timing") == 0) {
        cmd_byte = CMD_SET_TIMING;
        int click_max = 500, dbl_gap = 400, hold_min = 800, long_hold = 2000;
        if (auto* v = cJSON_GetObjectItem(root, "click_max_ms"); cJSON_IsNumber(v)) click_max = v->valueint;
        if (auto* v = cJSON_GetObjectItem(root, "double_click_gap_ms"); cJSON_IsNumber(v)) dbl_gap = v->valueint;
        if (auto* v = cJSON_GetObjectItem(root, "hold_min_ms"); cJSON_IsNumber(v)) hold_min = v->valueint;
        if (auto* v = cJSON_GetObjectItem(root, "long_hold_min_ms"); cJSON_IsNumber(v)) long_hold = v->valueint;
        if (click_max < 0) click_max = 0;
        if (click_max > 65535) click_max = 65535;
        if (dbl_gap < 0) dbl_gap = 0;
        if (dbl_gap > 65535) dbl_gap = 65535;
        if (hold_min < 0) hold_min = 0;
        if (hold_min > 65535) hold_min = 65535;
        if (long_hold < 0) long_hold = 0;
        if (long_hold > 255) long_hold = 255;  // CAN frame limited to 8 bytes; long_hold sent as 1 byte
        uint16_t c = (uint16_t)click_max, g = (uint16_t)dbl_gap, h = (uint16_t)hold_min;
        payload[0] = (uint8_t)(c & 0xFF);
        payload[1] = (uint8_t)(c >> 8);
        payload[2] = (uint8_t)(g & 0xFF);
        payload[3] = (uint8_t)(g >> 8);
        payload[4] = (uint8_t)(h & 0xFF);
        payload[5] = (uint8_t)(h >> 8);
        payload[6] = (uint8_t)long_hold;
        plen = 7;
    } else if (strcmp(cmd_str, "reboot") == 0) {
        cmd_byte = CMD_REBOOT;
        plen = 0;
    } else {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unknown command");
    }

    cJSON_Delete(root);
    g_init.send_node_config_cb(target_node_id, cmd_byte, payload, plen);
    return httpd_resp_sendstr(req, "OK");
}

// POST /api/node/remove - remove a node from the hub list (e.g. offline/stale). Body: { "node_id": 1..127 }
static esp_err_t h_node_remove_post(httpd_req_t* req) {
    if (!g_init.remove_node_cb) {
        return httpd_resp_send_err(req, HTTPD_501_METHOD_NOT_IMPLEMENTED, "remove_node_cb not set");
    }
    auto body = req_read_all(req);
    cJSON* root = cJSON_Parse(body.c_str());
    if (!root) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");
    int node_id = -1;
    if (auto* n = cJSON_GetObjectItem(root, "node_id"); cJSON_IsNumber(n))
        node_id = n->valueint;
    cJSON_Delete(root);
    if (node_id < 1 || node_id > 127) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "node_id 1..127 required");
    }
    if (!g_init.remove_node_cb((uint8_t)node_id)) {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "node not in list");
    }
    return httpd_resp_sendstr(req, "OK");
}

// GET /api/can/history
static esp_err_t h_can_history(httpd_req_t *req) {
    cJSON* root = cJSON_CreateArray(); // Create a JSON array

    if (xSemaphoreTake(g_history_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (const auto& f : g_can_history) {
            cJSON* msg = cJSON_CreateObject();
            cJSON_AddNumberToObject(msg, "id", f.id);
            cJSON_AddNumberToObject(msg, "dlc", f.dlc);
            
            char hex[24] = {0}; // Format data as a hex string
            for (int i = 0; i < f.dlc; ++i) {
                sprintf(hex + (i * 2), "%02X", f.data[i]);
            }
            cJSON_AddStringToObject(msg, "data", hex);
            std::string description = "Unknown Message";
            if (!isStandardDataFrame(f.id)) {
                description = "Non-standard CAN frame (EFF/RTR/ERR) ignored by protocol";
                cJSON_AddStringToObject(msg, "description", description.c_str());
                cJSON_AddItemToArray(root, msg);
                continue;
            }

            const uint16_t sid = getStandardId(f.id);
            CanMessageType msgType = getMessageType(sid);

            if (static_cast<uint8_t>(msgType) == static_cast<uint8_t>(LIGHTING_COMMAND)) {
                if (f.dlc >= 2) {
                    int switchId = getNodeId(sid);
                    int button = f.data[0];
                    uint8_t evt  = f.data[1];

                    const char* action = decodeEventAction(evt);
        
                    char desc_buffer[120];
                    if (evt == 0x03 && f.dlc >= 3) {
                        snprintf(desc_buffer, sizeof(desc_buffer),
                            "Switch %d, Button %d, Event: %s", switchId, button, action);
                    } else {
                        snprintf(desc_buffer, sizeof(desc_buffer),
                            "Switch %d, Button %d, Event: %s", switchId, button, action);
                    }
                    description = desc_buffer;
                }
            } else if (msgType == HEARTBEAT) {
                int nodeId = getNodeId(sid);
                char desc_buffer[50];
                snprintf(desc_buffer, sizeof(desc_buffer), "Heartbeat from Node %d", nodeId);
                description = desc_buffer;
            }
            cJSON_AddStringToObject(msg, "description", description.c_str());
            cJSON_AddItemToArray(root, msg);
        }
        xSemaphoreGive(g_history_mutex);
    }
    return send_json(req, root); // send_json is already in your code
}

// GET /api/nodes
static esp_err_t h_nodes_get(httpd_req_t* req) {
    if (!g_init.get_nodes_json_cb) {
        return httpd_resp_send_err(req, HTTPD_501_METHOD_NOT_IMPLEMENTED, "get_nodes_json_cb not set");
    }
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    cJSON* nodes_array = g_init.get_nodes_json_cb();
    if (!nodes_array) {
        nodes_array = cJSON_CreateArray(); // Return empty array if callback fails
    }
    return send_json(req, nodes_array);
}

// POST /api/ota
static esp_err_t h_ota(httpd_req_t* req) {
    esp_ota_handle_t handle = 0;
    const esp_partition_t* update = esp_ota_get_next_update_partition(nullptr);
    if (!update) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no part");
    if (esp_ota_begin(update, OTA_SIZE_UNKNOWN, &handle) != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota begin");

    uint8_t buf[4096];
    int remaining = req->content_len;
    while (remaining > 0) {
        int to_read = remaining > (int)sizeof(buf) ? sizeof(buf) : remaining;
        int r = httpd_req_recv(req, (char*)buf, to_read);
        if (r <= 0) { esp_ota_end(handle); return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv"); }
        if (esp_ota_write(handle, buf, r) != ESP_OK) { esp_ota_end(handle); return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "write"); }
        remaining -= r;
    }
    if (esp_ota_end(handle) != ESP_OK) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "ota end");
    if (esp_ota_set_boot_partition(update) != ESP_OK) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "set boot");

    httpd_resp_sendstr(req, "OK");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;
}

static esp_err_t h_i2c_scan(httpd_req_t *req) {
    ESP_LOGI("API", "Received request to scan I2C devices.");

    // Call the existing scan and prune function
    // This is the same function we removed from app_main
    HSG::API::scan_and_prune_i2c(I2C_MASTER_NUM);

    // Send a success response
    const char* resp_str = "{\"status\": \"success\", \"message\": \"I2C scan and configuration prune complete.\"}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp_str, strlen(resp_str));

    return ESP_OK;
}

EventGroupHandle_t g_event_group = nullptr;

} // namespace (anon)

// This is the main handler for the WebSocket endpoint
static esp_err_t h_websocket(httpd_req_t *req) {
    // When a new client connects, store its file descriptor
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "New WebSocket client connected, fd=%d", httpd_req_to_sockfd(req));
        // Store the socket file descriptor for sending messages
        g_ws_fd = httpd_req_to_sockfd(req);
        return ESP_OK;
    }

    // --- The rest of the function handles receiving messages ---
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 128);
    if (ret != ESP_OK) {
        // This error (ESP_FAIL) is normal when a client disconnects
        if (ret == ESP_FAIL) {
            ESP_LOGW(TAG, "WebSocket client disconnected, fd=%d", httpd_req_to_sockfd(req));
            // Invalidate the file descriptor
            g_ws_fd = -1;
        } else {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
        }
        return ret;
    }
    
    // We don't expect messages from the client in this app, but if we did,
    // the logic would go here.
    
    return ESP_OK;
}

static esp_err_t h_state_get(httpd_req_t* req) {
    if (g_init.get_outputs_json_cb) {
        cJSON* root = cJSON_CreateObject();
        cJSON* outputs_json = g_init.get_outputs_json_cb(); // Call the function in main.cpp
        cJSON_AddItemToObject(root, "outputs", outputs_json);
        return send_json(req, root);
    }
    return httpd_resp_send_500(req); // Or send an empty object
}


// ------------------ Public API ------------------
namespace HSG {
namespace API {

esp_mqtt_client_handle_t get_mqtt_client() {
    return g_mqtt_client; 
}

cJSON* get_config_json_obj() {
    cJSON* root = load_cfg_json();   // already implemented in your file
    ensure_layout(root); // Ensures the "config" key exists
    if (!root) {
        ESP_LOGW(TAG, "get_config_json_obj: no config in NVS");
        return nullptr;
    }
    // We return a copy of the nested "config" object
    cJSON* config_obj = cJSON_GetObjectItem(root, "config");
    cJSON* config_copy = cJSON_Duplicate(config_obj, 1);
    
    cJSON_Delete(root);
    return config_copy;

    //return root;  // caller must cJSON_Delete()
}

esp_err_t register_uris(httpd_handle_t server, const Init& init) {
    if (!server) {
        ESP_LOGE(TAG, "Server handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    g_init = init;
    g_http = server; // Store the server handle for async sends

    // Root
    httpd_uri_t root = { .uri="/", .method=HTTP_GET, .handler=h_root, .user_ctx=nullptr };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root));

    // REST
    httpd_uri_t adopt_get  { .uri="/api/adopt",    .method=HTTP_GET,   .handler=h_adopt,      .user_ctx=nullptr };
    httpd_uri_t config_get { .uri="/api/config",   .method=HTTP_GET,   .handler=h_config_get, .user_ctx=nullptr };
    httpd_uri_t config_post{ .uri="/api/config",   .method=HTTP_POST,  .handler=h_config_post,.user_ctx=nullptr };
    httpd_uri_t mqtt_get   { .uri="/api/mqtt",     .method=HTTP_GET,   .handler=h_mqtt_get,   .user_ctx=nullptr };
//    httpd_uri_t mqtt_post  { .uri="/api/mqtt",     .method=HTTP_POST, .handler=h_mqtt_post,  .user_ctx=nullptr };
    httpd_uri_t cmd_post   { .uri="/api/command",  .method=HTTP_POST,  .handler=h_command,    .user_ctx=nullptr };
    httpd_uri_t ota_post   { .uri="/api/ota",      .method=HTTP_POST,  .handler=h_ota,        .user_ctx=nullptr };
    httpd_uri_t favicon_get = { .uri="/favicon.ico", .method=HTTP_GET, .handler=h_favicon,    .user_ctx=nullptr };
    httpd_uri_t config_page = { .uri="/config", .method=HTTP_GET, .handler=h_config_page, .user_ctx=nullptr };
    httpd_uri_t state_get = { .uri="/api/state", .method=HTTP_GET, .handler=h_state_get, .user_ctx=nullptr };
    httpd_uri_t i2c_scan_uri = { .uri="/api/i2c/scan", .method=HTTP_POST, .handler=h_i2c_scan, .user_ctx=nullptr };
    httpd_uri_t can_history = { .uri="/api/can/history", .method=HTTP_GET, .handler=h_can_history, .user_ctx=nullptr };
    httpd_uri_t nodes_get = { .uri="/api/nodes", .method=HTTP_GET, .handler=h_nodes_get, .user_ctx=nullptr };
    httpd_uri_t node_config_post = { .uri="/api/node/config", .method=HTTP_POST, .handler=h_node_config_post, .user_ctx=nullptr };
    httpd_uri_t node_remove_post = { .uri="/api/node/remove", .method=HTTP_POST, .handler=h_node_remove_post, .user_ctx=nullptr };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &node_remove_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &node_config_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &can_history));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &nodes_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &i2c_scan_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &state_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_page));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &favicon_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &adopt_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &mqtt_get));
//  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &mqtt_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &cmd_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ota_post));

    httpd_uri_t ws = {
        .uri        = "/ws",
        .method     = HTTP_GET,
        .handler    = h_websocket,
        .user_ctx   = NULL,
        .is_websocket = true
    };
    httpd_register_uri_handler(server, &ws);

    ESP_LOGI(TAG, "API URIs registered");
    return ESP_OK;
}

// prune PCA9685 config to only detected devices
static void prune_pca9685_config_to_detected(const std::vector<uint8_t>& detected) {
    std::unordered_set<std::string> keep;
    for (uint8_t a : detected) {
        char k[6]; std::snprintf(k, sizeof(k), "0x%02X", a);
        keep.insert(k);
    }

    // Work with CONFIG object only
    std::string cfg_text = HSG::API::get_config_json();
    cJSON* config = cJSON_Parse(cfg_text.c_str());
    if (!config) config = cJSON_CreateObject();

    // ensure i2c.pca9685 object
    cJSON* i2c = cJSON_GetObjectItem(config, "i2c");
    if (!cJSON_IsObject(i2c)) {
        i2c = cJSON_CreateObject();
        cJSON_AddItemToObject(config, "i2c", i2c);
    }
    cJSON* pca = cJSON_GetObjectItem(i2c, "pca9685");
    if (!cJSON_IsObject(pca)) {
        pca = cJSON_CreateObject();
        cJSON_AddItemToObject(i2c, "pca9685", pca);
    }

    // prune
    for (cJSON* child = pca->child; child; ) {
        cJSON* next = child->next;
        if (!keep.count(child->string)) {
            ESP_LOGW(TAG, "Removing PCA9685 at %s (not detected)", child->string);
            cJSON_DeleteItemFromObject(pca, child->string);
        }
        child = next;
    }

    // add defaults for newly detected
    for (auto& k : keep) {
        if (!cJSON_GetObjectItem(pca, k.c_str())) {
            cJSON* arr = cJSON_CreateArray();
            for (int i = 0; i < 16; ++i) cJSON_AddItemToArray(arr, cJSON_CreateNumber(0));
            cJSON_AddItemToObject(pca, k.c_str(), arr);
            ESP_LOGI(TAG, "Added default map for PCA9685 at %s", k.c_str());
        }
    }

    char* out = cJSON_PrintUnformatted(config);
    HSG::API::set_config_json(out ? out : "{}"); // replaces the CONFIG object atomically
    if (out) free(out);
    cJSON_Delete(config);
}

EventGroupHandle_t get_event_group() {
    return g_event_group;
}

esp_err_t start(const Init& cfg) {
    g_init = cfg;
    if (!g_last_mutex) g_last_mutex = xSemaphoreCreateMutex();
    if (!g_history_mutex) g_history_mutex = xSemaphoreCreateMutex();
    if (!g_event_group) g_event_group = xEventGroupCreate(); // Create the event group

    // Ensure NVS is ready (idempotent)
    if (nvs_flash_init() != ESP_OK) {
        esp_err_t r = nvs_flash_erase();
        if (r == ESP_OK) nvs_flash_init();
    }

    xEventGroupSetBits(g_event_group, CONFIG_LOADED_BIT);
    ESP_LOGI(TAG, "Configuration loaded, signaling event group.");

    httpd_config_t conf = HTTPD_DEFAULT_CONFIG();
    conf.max_uri_handlers = 16;   // more routes
    conf.lru_purge_enable = true;
    conf.server_port = 80;
//    conf.max_open_sockets = 10; // by adding this config the web server will stop working.
    conf.stack_size = 8192;
    
    httpd_handle_t server = nullptr;
    auto err = httpd_start(&server, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP start failed: %s", esp_err_to_name(err));
        return err;
    }

    g_http = server;

    ESP_ERROR_CHECK(register_uris(server, cfg));
    ESP_LOGI(TAG, "HTTP API ready on :%d", conf.server_port);
 
    ESP_LOGI(TAG, "HTTPD max_open_sockets = %d", conf.max_open_sockets);
    ESP_LOGI("LWIP", "MAX_SOCKETS: %d", MEMP_NUM_NETCONN);
    ESP_LOGI("LWIP", "TCP sockets: %d", MEMP_NUM_TCP_PCB);
    ESP_LOGI("LWIP", "TCPIP thread stack: %d", TCPIP_THREAD_STACKSIZE);

    return ESP_OK;
}

void stop() {
    if (g_http) {
        httpd_stop(g_http);
        g_http = nullptr;
    }
    if (g_last_mutex) { vSemaphoreDelete(g_last_mutex); g_last_mutex = nullptr; }
}

void add_to_can_history(const HSG_CanFrame& f) {
    if (!g_history_mutex) return;
    if (xSemaphoreTake(g_history_mutex, portMAX_DELAY) == pdTRUE) {
        g_can_history.insert(g_can_history.begin(), f);
        if (g_can_history.size() > MAX_CAN_HISTORY) {
            g_can_history.pop_back();
        }
        xSemaphoreGive(g_history_mutex);
    }
}

// returns only the "config" object as JSON
std::string get_config_json() {
    cJSON* full = load_cfg_json();
    ensure_layout(full);
    cJSON* cfg = cJSON_GetObjectItem(full, "config");
    char* txt = cJSON_PrintUnformatted(cfg ? cfg : cJSON_CreateObject());
    std::string out = txt ? txt : "{}";
    if (txt) free(txt);
    cJSON_Delete(full);
    return out;
}

// replaces the "config" object atomically
esp_err_t set_config_json(const char* text) {
    if (!text) return ESP_ERR_INVALID_ARG;
    cJSON* cfg = cJSON_Parse(text);
    if (!cfg) return ESP_ERR_INVALID_ARG;

    cJSON* full = load_cfg_json();
    ensure_layout(full);
    cJSON_DeleteItemFromObject(full, "config");
    cJSON_AddItemToObject(full, "config", cfg); // takes ownership
    esp_err_t r = save_cfg_json(full);
    cJSON_Delete(full);
    return r;
}

// expose a call you can trigger AFTER i2c init
esp_err_t scan_and_prune_i2c(int i2c_port) {
    auto found = scan_pca9685_addrs((i2c_port_t)i2c_port);
    if (found.empty()) {
        ESP_LOGW(TAG, "No PCA9685 detected on I2C bus");
    } else {
//        ESP_LOGI(TAG, "Detected %d PCA9685 device(s)", (int)found.size());
    }
    prune_pca9685_config_to_detected(found);
    return ESP_OK;
}

std::string get_mqtt_json() {
    cJSON* full = load_cfg_json(); ensure_layout(full);
    cJSON* cfg  = cJSON_GetObjectItem(full, "config");
    cJSON* mqtt = cJSON_GetObjectItem(cfg, "mqtt");
    char* txt = cJSON_PrintUnformatted(mqtt ? mqtt : cJSON_CreateObject());
    std::string out = txt ? txt : "{}";
    if (txt) free(txt);
    cJSON_Delete(full);
    return out;
}

// --- MQTT event handler (now internal to the component) ---
static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            if (!g_mqtt_command_topic.empty()) {
                esp_mqtt_client_subscribe(client, g_mqtt_command_topic.c_str(), 0);
                ESP_LOGI(TAG, "Subscribed to command topic: %s", g_mqtt_command_topic.c_str());
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT Disconnected");
            break;
        case MQTT_EVENT_DATA: {
            // Check if the topic matches our command topic
            if (!g_mqtt_command_topic.empty() && 
                event->topic_len == g_mqtt_command_topic.length() && 
                strncmp(event->topic, g_mqtt_command_topic.c_str(), event->topic_len) == 0) {

                std::string data(event->data, event->data_len);
                ESP_LOGI(TAG, "MQTT Command received: %s", data.c_str());

                cJSON *json = cJSON_Parse(data.c_str());
                if (json == NULL) {
                    ESP_LOGE(TAG, "Error parsing MQTT JSON command.");
                    break;
                }

                const cJSON *output_json = cJSON_GetObjectItem(json, "output");
                const cJSON *group_json = cJSON_GetObjectItem(json, "group");

                // --- Check if it's an OUTPUT command ---
                if (cJSON_IsNumber(output_json)) {
                    int output_num = output_json->valueint;
                    int brightness = 0;
                    int fade_ms = 0;

                    const cJSON *brightness_json = cJSON_GetObjectItem(json, "brightness");
                    const cJSON *fade_json = cJSON_GetObjectItem(json, "fade");
                    const cJSON *state_json = cJSON_GetObjectItem(json, "state");
                    const char* state_str = cJSON_IsString(state_json) ? state_json->valuestring : nullptr;

                    if (cJSON_IsNumber(brightness_json)) brightness = brightness_json->valueint;
                    //else if (cJSON_IsString(state_json) && strcmp(state_json->valuestring, "ON") == 0) brightness = 100;
                    
                    if (cJSON_IsNumber(fade_json)) fade_ms = fade_json->valueint;
                    
                    if (g_init.output_cb) {
                        g_init.output_cb(output_num, brightness, fade_ms, state_str);
                    }
                } 
                // --- FIX: Add this block to handle GROUP commands ---
                else if (cJSON_IsString(group_json)) {
                    int brightness = 100; // Default to ON
                    int fade_ms = 0;

                    const cJSON *brightness_json = cJSON_GetObjectItem(json, "brightness");
                    const cJSON *fade_json = cJSON_GetObjectItem(json, "fade");
                    const cJSON *state_json = cJSON_GetObjectItem(json, "state");
                    const char* state_str = cJSON_IsString(state_json) ? state_json->valuestring : nullptr;

                    if (cJSON_IsNumber(brightness_json)) {
                        brightness = brightness_json->valueint;
//                    } else if (cJSON_IsString(state_json) && strcmp(state_json->valuestring, "OFF") == 0) {
//                        brightness = 0;
                    }

                    if (cJSON_IsNumber(fade_json)) {
                        fade_ms = fade_json->valueint;
                    }

                    if (g_init.group_cb) {
                        g_init.group_cb(group_json->valuestring, brightness, fade_ms, state_str);
                    }
                }
                // --- END FIX ---
                
                cJSON_Delete(json);
            }
            break;
        }    
        default:
            ESP_LOGD(TAG, "Unhandled MQTT event id: %d", (int)event_id);
            break;
    }
}

esp_err_t mqtt_start() {
    std::string mqtt_json_str = HSG::API::get_mqtt_json();
    cJSON* mqtt_json = cJSON_Parse(mqtt_json_str.c_str());

    if (!mqtt_json) {
        ESP_LOGE(TAG, "Failed to parse MQTT JSON from API");
        return ESP_FAIL;
    }

    const cJSON* broker = cJSON_GetObjectItem(mqtt_json, "broker");
    if (!cJSON_IsString(broker) || broker->valuestring == NULL || strlen(broker->valuestring) == 0) {
        ESP_LOGW(TAG, "MQTT broker not configured. MQTT not started.");
        cJSON_Delete(mqtt_json);
        return ESP_OK; // Not an error, just not configured
    }

    const cJSON* clientId_json = cJSON_GetObjectItem(mqtt_json, "clientId");
    if (!cJSON_IsString(clientId_json) || clientId_json->valuestring == NULL) {
        ESP_LOGE(TAG, "MQTT clientId not found in config. Cannot subscribe to command topic.");
        cJSON_Delete(mqtt_json);
        return ESP_FAIL;
    }

     // --- FIX: Get the topicPrefix and build the full topic ---
    const cJSON* prefix_json = cJSON_GetObjectItem(mqtt_json, "topicPrefix");
    std::string clientId(clientId_json->valuestring);

    if (cJSON_IsString(prefix_json) && prefix_json->valuestring != NULL && strlen(prefix_json->valuestring) > 0) {
        // If prefix exists, build topic as: prefix/clientId/cmnd
        std::string prefix(prefix_json->valuestring);
        g_mqtt_command_topic = prefix + clientId + "/cmnd";
    } else {
        // Fallback if no prefix is set: clientId/cmnd
        g_mqtt_command_topic = clientId + "/cmnd";
    }
    // --- END FIX ---


    std::string broker_uri = "mqtt://" + std::string(broker->valuestring);

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = broker_uri.c_str();

    const cJSON* port = cJSON_GetObjectItem(mqtt_json, "port");
    if (cJSON_IsNumber(port)) {
        mqtt_cfg.broker.address.port = port->valueint;
    }

    // Optional credentials
    const cJSON* user = cJSON_GetObjectItem(mqtt_json, "username");
    if(cJSON_IsString(user)) mqtt_cfg.credentials.username = user->valuestring;
    const cJSON* pass = cJSON_GetObjectItem(mqtt_json, "password");
    if(cJSON_IsString(pass)) mqtt_cfg.credentials.authentication.password = pass->valuestring;

    g_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(g_mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_err_t err = esp_mqtt_client_start(g_mqtt_client);

    cJSON_Delete(mqtt_json);
    return err;
}

void mqtt_stop() {
    if (g_mqtt_client) {
        esp_mqtt_client_stop(g_mqtt_client);
        esp_mqtt_client_destroy(g_mqtt_client);
        g_mqtt_client = nullptr;
    }
}

void send_ws_message(const std::string& msg) {
    if (g_ws_fd >= 0) {
        httpd_ws_frame_t ws_pkt;
        memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
        ws_pkt.payload = (uint8_t*)msg.c_str();
        ws_pkt.len = msg.length();
        ws_pkt.type = HTTPD_WS_TYPE_TEXT;
        
        // httpd_ws_send_frame_async is safe to call from any task
        httpd_ws_send_frame_async(g_http, g_ws_fd, &ws_pkt);
    }
}


} // namespace API
} // namespace HSG
