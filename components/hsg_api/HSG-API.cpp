#include "HSG-API.h"

#include <cstring>
#include <string>
#include <vector>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/i2c.h"
#include "cJSON.h"

static const char* TAG = "HSG-API";
static const char* NVS_NS = "cfg";
static const char* NVS_KEY_CONFIG = "app_cfg";

extern const char _binary_ESP32_POE_html_start[] asm("_binary_ESP32_POE_html_start");
extern const char _binary_ESP32_POE_html_end[]   asm("_binary_ESP32_POE_html_end");

namespace {

httpd_handle_t g_http = nullptr;
HSG::API::Init g_init{};
HSG_CanFrame g_last{};
SemaphoreHandle_t g_last_mutex = nullptr;

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

// I2C probe (7-bit)
static esp_err_t i2c_probe(int port, uint8_t addr7) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)port, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ------------------ HTTP handlers ------------------

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
            char ipstr[16]; snprintf(ipstr, sizeof ipstr, IPSTR, IP2STR(&ip.ip));
            cJSON_AddStringToObject(net, "ip", ipstr);
        }
    }
    uint8_t mac[6]; esp_read_mac(mac, ESP_MAC_ETH); // prefer ETH MAC
    char macstr[18]; snprintf(macstr, sizeof macstr, "%02X:%02X:%02X:%02X:%02X:%02X",
                              mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    cJSON_AddStringToObject(net, "mac", macstr);
    cJSON_AddStringToObject(net, "mode", "ETH/WIFI failover");
    cJSON_AddItemToObject(root, "network", net);

    // system
    cJSON* sys = cJSON_CreateObject();
    cJSON_AddNumberToObject(sys, "heapFreeBytes", esp_get_free_heap_size());
    if (auto* running = esp_ota_get_running_partition()) cJSON_AddNumberToObject(sys, "sketchSpaceUsedBytes", running->size);
    cJSON_AddItemToObject(root, "system", sys);

    // i2c scan
    cJSON* i2c = cJSON_CreateObject();
    cJSON* pca = cJSON_CreateArray();
    for (uint32_t a = g_init.i2c_scan_start; a <= g_init.i2c_scan_end; ++a) {
        if (i2c_probe(g_init.i2c_port, (uint8_t)a) == ESP_OK) cJSON_AddItemToArray(pca, cJSON_CreateNumber(a));
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    cJSON_AddItemToObject(i2c, "pca9685", pca);
    cJSON_AddItemToObject(root, "i2c", i2c);

    return send_json(req, root);
}

// GET /api/config
static esp_err_t h_config_get(httpd_req_t* req) {
    cJSON* full = load_cfg_json();
    ensure_layout(full);
    cJSON* out = cJSON_CreateObject();
    cJSON_AddItemToObject(out, "config", cJSON_Duplicate(cJSON_GetObjectItem(full, "config"), 1));
    cJSON_Delete(full);
    return send_json(req, out);
}

// POST /api/config
static esp_err_t h_config_post(httpd_req_t* req) {
    auto body = req_read_all(req);
    cJSON* posted = cJSON_Parse(body.c_str());
    if (!posted) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");

    cJSON* full = load_cfg_json();
    ensure_layout(full);
    cJSON* config = cJSON_GetObjectItem(full, "config");

    // Merge "i2c" & "groups" (replace)
    for (auto key : { "i2c", "groups" }) {
        cJSON* v = cJSON_GetObjectItem(posted, key);
        if (v) {
            cJSON_DeleteItemFromObject(config, key);
            cJSON_AddItemToObject(config, key, cJSON_Duplicate(v, 1));
        }
    }
    cJSON_Delete(posted);
    ensure_layout(full);
    save_cfg_json(full);
    cJSON_Delete(full);
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
    // App can reconfigure MQTT client when convenient
    return httpd_resp_sendstr(req, "OK");
}

// POST /api/command
static esp_err_t h_command(httpd_req_t* req) {
    auto body = req_read_all(req);
    cJSON* cmd = cJSON_Parse(body.c_str());
    if (!cmd) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad json");

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
        const char* state = "ON";
        if (auto* s = cJSON_GetObjectItem(cmd, "state"); cJSON_IsString(s)) state = s->valuestring;
        if (g_init.group_cb) g_init.group_cb(grp->valuestring, state, fade);
    }

    cJSON_Delete(cmd);
    return httpd_resp_sendstr(req, "OK");
}

// GET /api/can/last   (optional, handy for your UI)
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

// POST /api/ota  (raw bin upload)
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

static esp_err_t h_root(httpd_req_t* req) {
    size_t len = _binary_ESP32_POE_html_end - _binary_ESP32_POE_html_start;
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, _binary_ESP32_POE_html_start, len);
    return ESP_OK;
}


} // namespace (anon)

// ------------------ Public API ------------------
namespace HSG {
namespace API {

esp_err_t register_uris(httpd_handle_t server, const Init& init) {
    if (!server) {
        ESP_LOGE(TAG, "Server handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    g_init = init; // keep callbacks

    // existing URIs...
    httpd_uri_t root = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = h_root,
        .user_ctx  = nullptr
    };

    httpd_register_uri_handler(server, &root);

    // Define all URI handlers
    httpd_uri_t adopt_get  { .uri="/api/adopt",   .method=HTTP_GET,  .handler=h_adopt,     .user_ctx=nullptr };
    httpd_uri_t config_get { .uri="/api/config",  .method=HTTP_GET,  .handler=h_config_get,.user_ctx=nullptr };
    httpd_uri_t config_post{ .uri="/api/config",  .method=HTTP_POST, .handler=h_config_post,.user_ctx=nullptr };
    httpd_uri_t mqtt_get   { .uri="/api/mqtt",    .method=HTTP_GET,  .handler=h_mqtt_get,  .user_ctx=nullptr };
    httpd_uri_t mqtt_post  { .uri="/api/mqtt",    .method=HTTP_POST, .handler=h_mqtt_post, .user_ctx=nullptr };
    httpd_uri_t cmd_post   { .uri="/api/command", .method=HTTP_POST, .handler=h_command,   .user_ctx=nullptr };
    httpd_uri_t can_last   { .uri="/api/can/last",.method=HTTP_GET,  .handler=h_can_last,  .user_ctx=nullptr };
    httpd_uri_t ota_post   { .uri="/api/ota",     .method=HTTP_POST, .handler=h_ota,       .user_ctx=nullptr };

    // Register them
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &adopt_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &config_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &mqtt_get));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &mqtt_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &cmd_post));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &can_last));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &ota_post));

    ESP_LOGI(TAG, "API URIs registered");
    return ESP_OK;
}

esp_err_t start(const Init& cfg) {
    g_init = cfg;
    if (!g_last_mutex) g_last_mutex = xSemaphoreCreateMutex();

    // Ensure NVS is ready (idempotent)
    if (nvs_flash_init() != ESP_OK) {
        esp_err_t r = nvs_flash_erase();
        if (r == ESP_OK) nvs_flash_init();
    }

//    if (g_http) return ESP_OK; // already started

    httpd_config_t conf = HTTPD_DEFAULT_CONFIG();
    conf.max_uri_handlers = 16;   // allow more routes
    conf.lru_purge_enable = true; // optional: allows reusing slots if needed
    conf.server_port = 80;

    httpd_handle_t server = nullptr;
    auto err = httpd_start(&server, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "HTTP start failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_ERROR_CHECK(register_uris(server, cfg));
    ESP_LOGI(TAG, "HTTP API ready on :%d", conf.server_port);
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

std::string get_config_json() {
    cJSON* root = load_cfg_json();
    ensure_layout(root);
    char* txt = cJSON_Print(root);
    std::string out = txt ? txt : "{}";
    if (txt) free(txt);
    cJSON_Delete(root);
    return out;
}

esp_err_t set_config_json(const char* text) {
    if (!text) return ESP_ERR_INVALID_ARG;
    cJSON* root = cJSON_Parse(text);
    if (!root) return ESP_ERR_INVALID_ARG;
    ensure_layout(root);
    auto r = save_cfg_json(root);
    cJSON_Delete(root);
    return r;
}

std::string get_mqtt_json() {
    cJSON* full = load_cfg_json(); ensure_layout(full);
    cJSON* cfg  = cJSON_GetObjectItem(full, "config");
    cJSON* mqtt = cJSON_GetObjectItem(cfg, "mqtt");
    char* txt = cJSON_Print(mqtt ? mqtt : cJSON_CreateObject());
    std::string out = txt ? txt : "{}";
    if (txt) free(txt);
    cJSON_Delete(full);
    return out;
}

} // namespace API
} // namespace HSG
