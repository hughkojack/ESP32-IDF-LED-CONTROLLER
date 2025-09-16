#include "hsg_outputs.h"
#include "hsg_pca9685.h"
#include "esp_log.h"
#include <vector>
#include <string>

static const char* TAG = "HSG-OUTPUTS";

struct OutputMapping {
    int logical_output;
    uint8_t i2c_addr;
    uint8_t channel;
};

static std::vector<OutputMapping> g_mappings;
static int g_i2c_port = 0;

// This is the main logic that parses the config. It's now a helper function.
static void parse_config(cJSON* config_json) {
    g_mappings.clear();

    if (!config_json) {
        ESP_LOGE(TAG, "parse_config received a null JSON object.");
        return;
    }

    cJSON* i2c = cJSON_GetObjectItem(config_json, "i2c");
    if (!i2c) return;

    cJSON* pca9685 = cJSON_GetObjectItem(i2c, "pca9685");
    if (!pca9685) return;

    // Iterate over each PCA9685 device address (e.g., "0x40", "0x41")
    cJSON* device = NULL;
    cJSON_ArrayForEach(device, pca9685) {
        uint8_t addr = (uint8_t)strtol(device->string, NULL, 16);
        int channel_index = 0;
        cJSON* channel_map = NULL;
        cJSON_ArrayForEach(channel_map, device) {
            if (cJSON_IsNumber(channel_map) && channel_map->valueint > 0) {
                g_mappings.push_back({channel_map->valueint, addr, (uint8_t)channel_index});
                ESP_LOGI(TAG, "Mapped OUT %d -> PCA9685@0x%02X ch%d", channel_map->valueint, addr, channel_index);
            }
            channel_index++;
        }
    }
    ESP_LOGI(TAG, "Outputs initialized (%d mapped)", g_mappings.size());
}


// --- PUBLIC FUNCTIONS ---

// Init now receives the config from main
esp_err_t hsg_outputs_init(int i2c_port, cJSON* config_json) {
    g_i2c_port = i2c_port;
    if (!config_json) {
        ESP_LOGE(TAG, "hsg_outputs_init called with null config!");
        return ESP_ERR_INVALID_ARG;
    }
    parse_config(config_json);
    return ESP_OK;
}

// Reload also receives the config from main
esp_err_t hsg_outputs_reload_config(cJSON* config_json) {
    ESP_LOGI(TAG, "Reloading outputs configuration...");
     if (!config_json) {
        ESP_LOGE(TAG, "hsg_outputs_reload_config called with null config!");
        return ESP_ERR_INVALID_ARG;
    }
    parse_config(config_json);
    return ESP_OK;
}

bool hsg_outputs_get_mapping(int output_num, uint8_t* addr, uint8_t* channel) {
    for (const auto& map : g_mappings) {
        if (map.logical_output == output_num) {
            *addr = map.i2c_addr;
            *channel = map.channel;
            return true;
        }
    }
    return false;
}