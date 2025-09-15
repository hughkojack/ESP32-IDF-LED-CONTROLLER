#include "hsg_outputs.h"
#include "HSG-API.h"
#include "cJSON.h"
#include "esp_log.h"
#include <map>
#include <string>

static const char* TAG = "HSG-OUTPUTS";

// This struct holds the physical location of a logical output
struct OutputMap {
    uint8_t addr;
    uint8_t channel;
};

static std::map<int, OutputMap> g_output_map;
static cJSON* g_cfg = nullptr;

// -----------------------------------------------------------------------------
// Helper: Rebuilds the internal mapping from the configuration JSON
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
// API: Initialize the outputs component and build the initial map
// -----------------------------------------------------------------------------
esp_err_t hsg_outputs_init(i2c_port_t port)
{
    if (g_cfg) {
        cJSON_Delete(g_cfg);
        g_cfg = nullptr;
    }

    g_cfg = HSG::API::get_config_json_obj();  // Get a copy of the stored config
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
// API: Gets the physical mapping for a logical output number.
// This is called by the animation engine in main.cpp.
// -----------------------------------------------------------------------------
bool hsg_outputs_get_mapping(int output, uint8_t *addr, uint8_t *channel)
{
    auto it = g_output_map.find(output);
    if (it == g_output_map.end()) {
        return false; // No mapping found for this output
    }

    *addr = it->second.addr;
    *channel = it->second.channel;
    return true;
}


// -----------------------------------------------------------------------------
// API: Reloads the configuration if it has been updated via the web UI
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

