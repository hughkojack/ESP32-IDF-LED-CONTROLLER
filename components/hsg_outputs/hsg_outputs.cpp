#include "hsg_outputs.h"
#include "HSG-API.h"
#include "hsg_pca9685.h"
#include "cJSON.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <map>
#include <string>

static const char* TAG = "HSG-OUTPUTS";

struct OutputMap {
    uint8_t addr;
    uint8_t channel;
};

static std::map<int, OutputMap> g_output_map;
static cJSON* g_cfg = nullptr;
static i2c_port_t g_i2c_port = I2C_NUM_0;  // default

// -----------------------------------------------------------------------------
// Helper: rebuild mapping from config JSON
// -----------------------------------------------------------------------------
static void rebuild_output_map(cJSON* cfg)
{
    g_output_map.clear();
    if (!cfg) return;

    cJSON* i2c = cJSON_GetObjectItem(cfg, "i2c");
    if (!i2c) return;

    cJSON* pca = cJSON_GetObjectItem(i2c, "pca9685");
    if (!pca) return;

    cJSON* dev = nullptr;
    cJSON_ArrayForEach(dev, pca) {
        if (!dev->string || !cJSON_IsArray(dev)) continue;

        // Key is like "0x40"
        uint8_t addr = strtol(dev->string, nullptr, 0);

        int idx = 0;
        cJSON* el = nullptr;
        cJSON_ArrayForEach(el, dev) {
            if (cJSON_IsNumber(el)) {
                int out_num = el->valueint;
                if (out_num > 0) {
                    g_output_map[out_num] = { addr, (uint8_t)idx };
                    ESP_LOGI(TAG, "Mapped OUT %d -> PCA9685@0x%02X ch%d",
                             out_num, addr, idx);
                }
            }
            idx++;
        }
    }
}

// -----------------------------------------------------------------------------
// API: initialize outputs from config
// -----------------------------------------------------------------------------
esp_err_t hsg_outputs_init(i2c_port_t port)
{
    g_i2c_port = port;
    if (g_cfg) {
        cJSON_Delete(g_cfg);
        g_cfg = nullptr;
    }

    g_cfg = HSG::API::get_config_json_obj();  // deep copy of stored config
    if (!g_cfg) {
        ESP_LOGW(TAG, "No config available, outputs not mapped");
        return ESP_FAIL;
    }

    cJSON* cfg = cJSON_GetObjectItem(g_cfg, "config");
    if (!cfg) {
        ESP_LOGW(TAG, "No 'config' object in JSON");
        return ESP_FAIL;
    }

    rebuild_output_map(cfg);
    ESP_LOGI(TAG, "Outputs initialized (%zu mapped)", g_output_map.size());
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// RELOAD CONFIG FUNCTION HERE, at file scope (not nested!)
// -----------------------------------------------------------------------------
void hsg_outputs_reload_config() {
    if (g_cfg) {
        cJSON_Delete(g_cfg);
        g_cfg = nullptr;
    }

    g_cfg = HSG::API::get_config_json_obj();
    if (!g_cfg) {
        ESP_LOGW(TAG, "reload_config: Failed to reload config");
        g_output_map.clear();
        return;
    }

    cJSON* cfg = cJSON_GetObjectItem(g_cfg, "config");
    if (!cfg) {
        ESP_LOGW(TAG, "reload_config: No 'config' object");
        g_output_map.clear();
        return;
    }

    rebuild_output_map(cfg);
    ESP_LOGI(TAG, "reload_config: Outputs rebuilt (%zu mapped)", g_output_map.size());
}

// -----------------------------------------------------------------------------
// API: set output (called from API output_cb)
// -----------------------------------------------------------------------------
esp_err_t hsg_outputs_set(int out, int brightness, int fade_ms)
{
    auto it = g_output_map.find(out);
    if (it == g_output_map.end()) {
        ESP_LOGW(TAG, "No mapping for OUT %d", out);
        return ESP_ERR_NOT_FOUND;
    }

    auto map = it->second;
    ESP_LOGI(TAG, "Set OUT %d -> PCA9685@0x%02X ch%d brightness=%d fade=%d",
             out, map.addr, map.channel, brightness, fade_ms);

    // Now actually drive the PCA9685
    int duty = brightness; // assume 0–100%
    esp_err_t err = pca9685_set_pwm(g_i2c_port, map.addr, map.channel, duty, fade_ms);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set PWM out=%d addr=0x%02X ch=%d", out, map.addr, map.channel);
    }
    return ESP_OK;
}

esp_err_t hsg_outputs_set_group(const char* name, const char* state, int fade_ms) {
    ESP_LOGI(TAG, "Set GROUP %s -> state %s fade %d", name, state, fade_ms);

    // TODO: lookup group in g_cfg->groups[name], call hsg_outputs_set_output() for each member

    return ESP_OK;
}