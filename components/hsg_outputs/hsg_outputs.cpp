#include "hsg_outputs.h"
#include "hsg_pca9685.h"
#include "i2c_bus_lock.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include <vector>
#include <string>
#include <set>
#include <algorithm>
#include <cstdio>

static const char* TAG = "HSG-OUTPUTS";
static const int kPcaFreqHz = 1000;

struct OutputMapping {
    int logical_output;
    uint8_t i2c_addr;
    uint8_t channel;
};

static std::vector<OutputMapping> g_mappings;
static int g_i2c_port = 0;

extern i2c_master_bus_handle_t i2c_bus_handle;

static void collect_pca_addresses_from_config(cJSON* config_json, std::set<uint8_t>& out) {
    if (!config_json) return;
    cJSON* i2c = cJSON_GetObjectItem(config_json, "i2c");
    cJSON* pca9685 = i2c ? cJSON_GetObjectItem(i2c, "pca9685") : nullptr;
    if (!cJSON_IsObject(pca9685)) return;
    for (cJSON* child = pca9685->child; child; child = child->next) {
        if (child->string)
            out.insert((uint8_t)strtol(child->string, nullptr, 16));
    }
}

static esp_err_t init_chip_with_retry(uint8_t addr, int attempts) {
    esp_err_t last = ESP_FAIL;
    for (int i = 0; i < attempts; ++i) {
        if (i > 0) {
            i2c_bus_recover(i2c_bus_handle);
            vTaskDelay(pdMS_TO_TICKS(50 + 250 * (unsigned)i));
        }
        last = hsg_pca9685::pca9685_init(addr, kPcaFreqHz);
        if (last == ESP_OK) {
            ESP_LOGI(TAG, "PCA9685 @0x%02X initialized", addr);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "PCA9685 @0x%02X init attempt %d failed: %s", addr, i + 1, esp_err_to_name(last));
        taskYIELD();
    }
    ESP_LOGE(TAG, "PCA9685 @0x%02X init failed after %d attempts", addr, attempts);
    return last;
}

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

    // Iterate PCA9685 devices in ascending I2C address order (stable log + init order)
    std::vector<std::pair<uint8_t, cJSON*>> pca_devices;
    for (cJSON* device = pca9685->child; device; device = device->next) {
        if (device->string)
            pca_devices.push_back({ (uint8_t)strtol(device->string, nullptr, 16), device });
    }
    std::sort(pca_devices.begin(), pca_devices.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    for (const auto& entry : pca_devices) {
        cJSON* device = entry.second;
        uint8_t addr = entry.first;
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
    ESP_LOGI(TAG, "Outputs initialized (%d mapped)", (int)g_mappings.size());
}

static esp_err_t init_pca_addresses(const std::set<uint8_t>& addresses, int retry_attempts,
                                    hsg_pca_init_stats_t* stats_out) {
    hsg_pca_init_stats_t stats = {0, 0};
    if (addresses.empty()) {
        ESP_LOGW(TAG, "No PCA9685 addresses to initialize.");
        if (stats_out) *stats_out = stats;
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing %d PCA9685 chip(s)...", (int)addresses.size());

    hsg_pca9685::pca9685_invalidate_bus_devices();
    esp_err_t rr = i2c_bus_recover(i2c_bus_handle);
    if (rr != ESP_OK) {
        ESP_LOGW(TAG, "I2C bus recover before PCA init failed: %s", esp_err_to_name(rr));
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    for (uint8_t addr : addresses) {
        ESP_LOGI(TAG, "PCA9685 @0x%02X: starting init...", addr);
        esp_err_t result = init_chip_with_retry(addr, retry_attempts);
        if (result == ESP_OK)
            stats.ok++;
        else
            stats.failed++;
        taskYIELD();
    }

    ESP_LOGI(TAG, "PCA init complete: %d ok, %d failed", stats.ok, stats.failed);
    if (stats_out) *stats_out = stats;
    return stats.failed > 0 ? ESP_FAIL : ESP_OK;
}

// --- PUBLIC FUNCTIONS ---

esp_err_t hsg_outputs_init(int i2c_port, cJSON* config_json) {
    g_i2c_port = i2c_port;
    if (!config_json) {
        ESP_LOGE(TAG, "hsg_outputs_init called with null config!");
        return ESP_ERR_INVALID_ARG;
    }
    parse_config(config_json);

    std::set<uint8_t> addresses;
    collect_pca_addresses_from_config(config_json, addresses);
    for (const auto& map : g_mappings)
        addresses.insert(map.i2c_addr);

    hsg_pca_init_stats_t stats = {0, 0};
    init_pca_addresses(addresses, 5, &stats);
    return ESP_OK;
}

esp_err_t hsg_outputs_reload_config(cJSON* config_json) {
    ESP_LOGI(TAG, "Reloading outputs configuration...");
    if (!config_json) {
        ESP_LOGE(TAG, "hsg_outputs_reload_config called with null config!");
        return ESP_ERR_INVALID_ARG;
    }
    parse_config(config_json);
    return ESP_OK;
}

int hsg_outputs_get_mapped_count(void) {
    return (int)g_mappings.size();
}

esp_err_t hsg_outputs_init_chips(const uint8_t* addrs, size_t count, int retry_attempts,
                                 hsg_pca_init_stats_t* stats_out) {
    std::set<uint8_t> addresses;
    for (size_t i = 0; i < count; ++i)
        addresses.insert(addrs[i]);
    return init_pca_addresses(addresses, retry_attempts, stats_out);
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

void hsg_outputs_clear_all() {
    ESP_LOGI(TAG, "Clearing all physical PWM outputs...");
    std::set<uint8_t> addresses;
    for (const auto& map : g_mappings)
        addresses.insert(map.i2c_addr);

    char addrs_buf[64] = "(none)";
    int n = 0;
    if (!addresses.empty()) {
        addrs_buf[0] = '\0';
        for (uint8_t a : addresses) {
            n += snprintf(addrs_buf + n, sizeof(addrs_buf) - n, "0x%02X ", a);
            if (n >= (int)sizeof(addrs_buf) - 8) break;
        }
    }
    ESP_LOGI(TAG, "Clearing only configured PCA9685 addrs: %s (not 0x68/0x69 RTC)", addrs_buf);
    for (uint8_t addr : addresses) {
        for (uint8_t channel = 0; channel < 16; ++channel) {
            hsg_pca9685::pca9685_write_pwm_value(addr, channel, 0);
            if ((channel & 3) == 3)
                taskYIELD();
        }
    }
}
