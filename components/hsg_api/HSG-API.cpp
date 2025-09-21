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
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/i2c_master.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char* TAG = "HSG-API";
static const char* NVS_NS = "cfg";
static const char* NVS_KEY_CONFIG = "app_cfg";

extern const char _binary_ESP32_POE_html_start[] asm("_binary_ESP32_POE_html_start");
extern const char _binary_ESP32_POE_html_end[]   asm("_binary_ESP32_POE_html_end");

extern const uint8_t _binary_favicon_ico_start[] asm("_binary_favicon_ico_start");
extern const uint8_t _binary_favicon_ico_end[]   asm("_binary_favicon_ico_end");

extern i2c_master_bus_handle_t i2c_bus_handle;

namespace {

httpd_handle_t g_http = nullptr;
HSG::API::Init g_init{};
HSG_CanFrame g_last{};
SemaphoreHandle_t g_last_mutex = nullptr;
esp_mqtt_client_handle_t g_mqtt_client = nullptr;
static std::string g_mqtt_command_topic;

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
}
/*
// I2C probe (7-bit)
static esp_err_t i2c_probe(i2c_port_t port, uint8_t addr7) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
    return ret;
}*/

static esp_err_t i2c_probe(uint8_t addr) {
    i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = 100000; // Probe at a safe 100KHz

    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &dev_handle);
    if (ret == ESP_OK) {
        i2c_master_bus_rm_device(dev_handle); // Clean up immediately after successful probe
    }
    return ret;
}


static std::vector<uint8_t> scan_pca9685_addrs(i2c_port_t port) {
    std::vector<uint8_t> found;
    for (uint8_t addr = 0x40; addr <= 0x7F; ++addr) {
        if (addr == 0x70) continue; // ignore All-Call
        if (i2c_probe(addr) == ESP_OK) {
//            ESP_LOGI(TAG, "Found I2C device at 0x%02X", addr);
            found.push_back(addr);
        }
    }
    return found;
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
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(def, &ip) == ESP_OK) {
            char ipstr[16]; 
            snprintf(ipstr, sizeof ipstr, IPSTR, IP2STR(&ip.ip));
            cJSON_AddStringToObject(net, "ip", ipstr);
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
    if (auto* running = esp_ota_get_running_partition()) {
        cJSON_AddNumberToObject(sys, "sketchSpaceUsedBytes", running->size);
    }
    cJSON_AddItemToObject(root, "system", sys);

    // i2c scan (only valid PCA9685 addresses)
    cJSON* i2c = cJSON_CreateObject();
    cJSON* pca = cJSON_CreateArray();

    // PCA9685 valid range: 0x40–0x47 (7-bit address space)
    for (uint8_t addr = 0x40; addr <= 0x47; ++addr) {
        if (i2c_probe(addr) == ESP_OK) {
            ESP_LOGI(TAG, "Adopt: Found PCA9685 at 0x%02X", addr);
            cJSON_AddItemToArray(pca, cJSON_CreateNumber(addr));
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    cJSON_AddItemToObject(i2c, "pca9685", pca);
    cJSON_AddItemToObject(root, "i2c", i2c);

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
    // (wifi, mqtt, i2c, groups) and updates them in the main config.
    for (cJSON* new_section = posted_config->child; new_section != NULL; new_section = new_section->next) {
        // Remove the old section if it exists
        if (cJSON_HasObjectItem(config_obj, new_section->string)) {
            cJSON_DeleteItemFromObject(config_obj, new_section->string);
        }
        // Add the new (or updated) section
        cJSON_AddItemToObject(config_obj, new_section->string, cJSON_Duplicate(new_section, 1));
        ESP_LOGI(TAG, "Updated config section: %s", new_section->string);
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

    if (auto* out = cJSON_GetObjectItem(cmd, "output"); cJSON_IsNumber(out)) {
        int brightness = 0;
        if (auto* b = cJSON_GetObjectItem(cmd, "brightness"); cJSON_IsNumber(b)) brightness = b->valueint;
        if (auto* s = cJSON_GetObjectItem(cmd, "state"); cJSON_IsString(s)) {
            std::string st = s->valuestring; for (auto& ch: st) ch = (char)toupper(ch);
            brightness = (st == "ON") ? 100 : 0;
        }
        if (g_init.output_cb) g_init.output_cb(out->valueint, brightness, fade);
    } else if (auto* grp = cJSON_GetObjectItem(cmd, "group"); cJSON_IsString(grp)) {
        int brightness = 100; // Default to ON
        int fade_ms = 0;
        
        const cJSON* brightness_json = cJSON_GetObjectItem(cmd, "brightness");
        const cJSON* state_json = cJSON_GetObjectItem(cmd, "state");
        const cJSON* fade_json = cJSON_GetObjectItem(cmd, "fade");

        if (cJSON_IsNumber(brightness_json)) {
            brightness = brightness_json->valueint;
        } else if (cJSON_IsString(state_json) && strcmp(state_json->valuestring, "OFF") == 0) {
            brightness = 0;
        }

        if (cJSON_IsNumber(fade_json)) {
            fade_ms = fade_json->valueint;
        }
        
//        if (auto* s = cJSON_GetObjectItem(cmd, "state"); cJSON_IsString(s)) state = s->valuestring;
        if (g_init.group_cb) g_init.group_cb(grp->valuestring, brightness, fade_ms);
    }

    cJSON_Delete(cmd);
    return httpd_resp_sendstr(req, "OK");
}

// GET /api/can/last
static esp_err_t h_can_last(httpd_req_t *req) {
    HSG_CanFrame f;
    if (g_last_mutex) xSemaphoreTake(g_last_mutex, portMAX_DELAY);
    f = g_last;
    if (g_last_mutex) xSemaphoreGive(g_last_mutex);

    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "id", (int)f.id);
    cJSON_AddNumberToObject(root, "dlc", f.dlc);
    char hex[3*8+1] = {0}; int p = 0;
    for (int i = 0; i < f.dlc && i < 8; ++i)
        p += snprintf(hex + p, sizeof hex - p, "%02X%s", f.data[i], (i+1<f.dlc)?" ":"");
    cJSON_AddStringToObject(root, "data", hex);
    return send_json(req, root);
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

EventGroupHandle_t g_event_group = nullptr;

} // namespace (anon)

// ------------------ Public API ------------------
namespace HSG {
namespace API {


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
    httpd_uri_t can_last   { .uri="/api/can/last", .method=HTTP_GET,   .handler=h_can_last,   .user_ctx=nullptr };
    httpd_uri_t ota_post   { .uri="/api/ota",      .method=HTTP_POST,  .handler=h_ota,        .user_ctx=nullptr };
    httpd_uri_t favicon_get = { .uri="/favicon.ico", .method=HTTP_GET, .handler=h_favicon,    .user_ctx=nullptr };

    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &favicon_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &adopt_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &mqtt_get));
//  ESP_ERROR_CHECK(httpd_register_uri_handler(server, &mqtt_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &cmd_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &can_last));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ota_post));

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

    httpd_handle_t server = nullptr;
    auto err = httpd_start(&server, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP start failed: %s", esp_err_to_name(err));
        return err;
    }
    g_http = server;

    ESP_ERROR_CHECK(register_uris(server, cfg));
    ESP_LOGI(TAG, "HTTP API ready on :%d", conf.server_port);

    // NOTE: I2C scan is NOT done here anymore. Call scan_and_prune_i2c()
    //       from app_main() AFTER i2c_master_init().
    return ESP_OK;
}

void stop() {
    if (g_http) {
        httpd_stop(g_http);
        g_http = nullptr;
    }
    if (g_last_mutex) { vSemaphoreDelete(g_last_mutex); g_last_mutex = nullptr; }
}

void update_last_can(const HSG_CanFrame& f) {
    if (!g_last_mutex) return;
    xSemaphoreTake(g_last_mutex, portMAX_DELAY);
    g_last = f;
    xSemaphoreGive(g_last_mutex);
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
                    const cJSON *state_json = cJSON_GetObjectItem(json, "state");
                    const cJSON *fade_json = cJSON_GetObjectItem(json, "fade");

                    if (cJSON_IsNumber(brightness_json)) brightness = brightness_json->valueint;
                    else if (cJSON_IsString(state_json) && strcmp(state_json->valuestring, "ON") == 0) brightness = 100;
                    
                    if (cJSON_IsNumber(fade_json)) fade_ms = fade_json->valueint;
                    
                    if (g_init.output_cb) {
                        g_init.output_cb(output_num, brightness, fade_ms);
                    }
                } 
                // --- FIX: Add this block to handle GROUP commands ---
                else if (cJSON_IsString(group_json)) {
                    int brightness = 100; // Default to ON
                    int fade_ms = 0;

                    const cJSON *brightness_json = cJSON_GetObjectItem(json, "brightness");
                    const cJSON *state_json = cJSON_GetObjectItem(json, "state");
                    const cJSON *fade_json = cJSON_GetObjectItem(json, "fade");

                    if (cJSON_IsNumber(brightness_json)) {
                        brightness = brightness_json->valueint;
                    } else if (cJSON_IsString(state_json) && strcmp(state_json->valuestring, "OFF") == 0) {
                        brightness = 0;
                    }

                    if (cJSON_IsNumber(fade_json)) {
                        fade_ms = fade_json->valueint;
                    }

                    if (g_init.group_cb) {
                        g_init.group_cb(group_json->valuestring, brightness, fade_ms);
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

} // namespace API
} // namespace HSG
