/**
 * HSG Light Controller Firmware - ESP-IDF Version
 *
 * This version uses a centralized, non-blocking animation engine in main.cpp
 * to provide smooth, simultaneous fades for all outputs.
 */

extern "C" {
#include "mcp2515.h"
}

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <map>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
//#include "esp_eth_driver.h"

#include "hardware_config.h"
#include "i2c_bus_lock.h"
#if defined(BOARD_RACK32)
    #include "esp_mac.h"
#endif

#include "can_protocol.h"
#include "can_tunnel.h"
#include "HSG-API.h"
#include "hsg_outputs.h"
#include "hsg_pca9685.h"
#include "ds3231.h"
#include "mcp9808.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include <ctime>
#include <sys/time.h>
#include "esp_netif_sntp.h"
//#include "sdkconfig.h"

static const char *TAG = "MAIN";

static bool s_sntp_started = false;

/*--------------------------- Constants ----------------------------------*/
#define MAX_OUTPUTS 160
#define DEFAULT_FADE_MS 1000

// --- Global State for Network Failover ---
static EventGroupHandle_t s_net_event_group;
static const int ETH_CONNECTED_BIT = BIT0;
static const int WIFI_CONNECTED_BIT = BIT1;
static bool s_eth_connected = false;
static bool s_wifi_connected = false;
static bool s_outputs_init_done = false;
static bool s_wifi_failover_pending = false;
static esp_netif_t *s_eth_netif = NULL;
static esp_netif_t *s_wifi_netif = NULL;
i2c_master_bus_handle_t i2c_bus_handle;
/* Cache last known-good RTC time (from boot or callback); used when live read from HTTP task returns stale */
static std::string g_rtc_cache;
static TickType_t g_rtc_cache_ticks = 0;
static const TickType_t g_rtc_cache_max_age_ticks = pdMS_TO_TICKS(120000); /* 120s */
/* Automatic RTC sync configuration: interval and per-boot flag */
static const time_t RTC_AUTO_SYNC_INTERVAL_SEC = 24 * 3600; /* 24 hours */
static bool s_rtc_auto_synced_this_boot = false;
static spi_device_handle_t can_spi_handle;
static MCP2515* mcp2515_ptr = nullptr;
static QueueHandle_t gpio_evt_queue = nullptr;
static QueueHandle_t can_message_queue = nullptr;
static SemaphoreHandle_t g_can_send_mutex = nullptr;

// --- Application Structs & Data ---
enum class TargetType { OUTPUT, GROUP };
struct Command {
    TargetType type;
    int output_id = 0;
    std::string group_name;
    int brightness = 0;
    int fade_ms = 0;
    std::string state;
};

struct Binding {
    int switchId = 0;
    int button = 0;
    std::string onAction;
    TargetType targetType = TargetType::OUTPUT;
    int outputId = 0;
    std::string groupName;
    std::string state;
    int brightness = -1; // -1 indicates not set
    int fade_ms = 0;
};

// A vector to hold all the rules loaded from config
static std::vector<Binding> g_bindings;

// Node tracking structure
#define MAX_INPUTS_PER_NODE 6  // inputs per node 1..6
#define MAX_INPUT_LABEL_LEN 24

struct Ws2812EffectParams {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint16_t timing_ms;
};

static void ws2812_effect_params_set_defaults(Ws2812EffectParams* p, uint8_t effect_id) {
    if (!p) return;
    switch (effect_id) {
        case 0: *p = {220, 180, 80, 45}; break;
        case 1: *p = {80, 60, 24, 50}; break;
        case 2: *p = {180, 120, 20, 150}; break;
        default: *p = {0, 0, 0, 100}; break;
    }
}

static void load_ws2812_effect_from_json(Ws2812EffectParams* out, cJSON* obj, uint8_t effect_id) {
    ws2812_effect_params_set_defaults(out, effect_id);
    if (!obj || !cJSON_IsObject(obj)) return;
    cJSON* item = cJSON_GetObjectItem(obj, "r");
    if (item && cJSON_IsNumber(item)) out->r = (uint8_t)(item->valueint & 0xFF);
    item = cJSON_GetObjectItem(obj, "g");
    if (item && cJSON_IsNumber(item)) out->g = (uint8_t)(item->valueint & 0xFF);
    item = cJSON_GetObjectItem(obj, "b");
    if (item && cJSON_IsNumber(item)) out->b = (uint8_t)(item->valueint & 0xFF);
    item = cJSON_GetObjectItem(obj, "timing_ms");
    if (item && cJSON_IsNumber(item)) {
        int t = item->valueint;
        if (t < 10) t = 10;
        if (t > 2000) t = 2000;
        out->timing_ms = (uint16_t)t;
    }
}

static void add_ws2812_effect_to_json(cJSON* parent, const char* name, const Ws2812EffectParams& p) {
    cJSON* obj = cJSON_CreateObject();
    if (!obj) return;
    cJSON_AddNumberToObject(obj, "r", p.r);
    cJSON_AddNumberToObject(obj, "g", p.g);
    cJSON_AddNumberToObject(obj, "b", p.b);
    cJSON_AddNumberToObject(obj, "timing_ms", p.timing_ms);
    cJSON_AddItemToObject(parent, name, obj);
}

struct NodeInfo {
    uint8_t node_id;
    uint8_t node_type;  // NODE_TYPE_LCD or NODE_TYPE_MECHANICAL
    uint8_t input_count;
    int64_t last_seen_timestamp_us;  // microseconds since boot
    bool is_configured;  // true if node_id != NODE_ID_UNCONFIGURED
    // Per-input switch type: 0 = momentary, 1 = toggle, 0xFF = unset (default momentary)
    uint8_t input_modes[MAX_INPUTS_PER_NODE];
    // Per-input label from web UI (for LCD display)
    char input_labels[MAX_INPUTS_PER_NODE][MAX_INPUT_LABEL_LEN + 1];
    // Per-input GPIO (mechanical node); 0xFF = not assigned
    uint8_t input_gpio[MAX_INPUTS_PER_NODE];
    // Per-input electrical sense: 0 = active low (pull-up, switch to GND), 1 = active high (pull-down)
    uint8_t input_active_high[MAX_INPUTS_PER_NODE];
    // Output index (0-255) used by node for Find Me blink; 0xFF = unset (node default)
    uint8_t find_me_output_index;
    // WS2812 night light (mechanical node)
    bool night_light_on;
    uint8_t night_light_brightness;
    uint8_t ws2812_click_effect;  // 0=strobe, 1=chase
    Ws2812EffectParams ws2812_strobe;
    Ws2812EffectParams ws2812_chase;
    Ws2812EffectParams ws2812_find_me;
    uint16_t fw_version;
    uint8_t ota_capable;
    char last_ota_result[32];

    NodeInfo() : node_id(0), node_type(0), input_count(0), last_seen_timestamp_us(0), is_configured(false), find_me_output_index(0xFF), night_light_on(false), night_light_brightness(0), ws2812_click_effect(0), fw_version(0), ota_capable(0) {
        last_ota_result[0] = '\0';
        ws2812_effect_params_set_defaults(&ws2812_strobe, 0);
        ws2812_effect_params_set_defaults(&ws2812_chase, 1);
        ws2812_effect_params_set_defaults(&ws2812_find_me, 2);
        for (int i = 0; i < MAX_INPUTS_PER_NODE; i++) {
            input_modes[i] = 0xFF;
            input_labels[i][0] = '\0';
            input_gpio[i] = 0xFF;
            input_active_high[i] = 0;
        }
    }
};

static void load_ws2812_effects_from_json(NodeInfo& node, cJSON* node_json) {
    ws2812_effect_params_set_defaults(&node.ws2812_strobe, 0);
    ws2812_effect_params_set_defaults(&node.ws2812_chase, 1);
    ws2812_effect_params_set_defaults(&node.ws2812_find_me, 2);
    cJSON* fx = cJSON_GetObjectItem(node_json, "ws2812_effects");
    if (!fx || !cJSON_IsObject(fx)) return;
    load_ws2812_effect_from_json(&node.ws2812_strobe, cJSON_GetObjectItem(fx, "strobe"), 0);
    load_ws2812_effect_from_json(&node.ws2812_chase, cJSON_GetObjectItem(fx, "chase"), 1);
    load_ws2812_effect_from_json(&node.ws2812_find_me, cJSON_GetObjectItem(fx, "find_me"), 2);
}

static void add_ws2812_effects_json(cJSON* node_json, const NodeInfo& node) {
    cJSON* fx = cJSON_CreateObject();
    if (!fx) return;
    add_ws2812_effect_to_json(fx, "strobe", node.ws2812_strobe);
    add_ws2812_effect_to_json(fx, "chase", node.ws2812_chase);
    add_ws2812_effect_to_json(fx, "find_me", node.ws2812_find_me);
    cJSON_AddItemToObject(node_json, "ws2812_effects", fx);
}

// Node registry: map from node_id to NodeInfo
static std::map<uint8_t, NodeInfo> g_nodes;
static SemaphoreHandle_t g_nodes_mutex = nullptr;
// When user sends set_node_id(old_id, new_id), we record here; when HEARTBEAT from new_id is seen, remove old_id permanently.
static std::map<uint8_t, uint8_t> g_pending_reconfig_old_id;  // key = new_id, value = old_id
static SemaphoreHandle_t g_pending_reconfig_mutex = nullptr;

struct OutputState {
    int startPwmValue = 0;
    int currentPwmValue = 0;
    int targetPwmValue = 0;
    unsigned long fadeStartTime = 0;
    unsigned long fadeDuration = DEFAULT_FADE_MS;
};
OutputState outputs[MAX_OUTPUTS];

//------------------------------------------------------------------------
int outputBrightness[MAX_OUTPUTS] = {0}; // Last "ON" brightness (0-100)
static can_frame g_last_frame = {};
static SemaphoreHandle_t g_bindings_mutex;

// Node registry functions (forward declarations)
static void load_nodes_from_config(void);
static void save_nodes_to_config(void);
// Run save in a one-shot task so CAN task does not block on NVS (avoids watchdog trigger)
static void defer_save_nodes_to_config(void);

//-----------------------------------------------------------------------

/*--------------------------- Function Declarations ---------------------------*/
static void reinit_task_wdt_no_idle(void)
{
    esp_task_wdt_deinit();
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 30000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
}

static esp_err_t i2c_master_init(void);
void main_task(void *pvParameter);
void animation_task(void *pvParameter);
void can_processing_task(void *pvParameter);
static void IRAM_ATTR gpio_isr_handler(void* arg);
static void wifi_start(void);
static void rtc_auto_sync_task(void *pvParameter);
void setOutput(int output, int brightness, int fadeMs, unsigned long startTimeOffset);
static void send_node_config(uint8_t target_node_id, uint8_t cmd, const uint8_t* data, size_t len);
static void set_node_input_label_impl(uint8_t node_id, uint8_t input_index, const char* label);
// Remove node from registry and persist; returns true if node was present and removed.
static bool remove_node_from_registry(uint8_t node_id);
static void w5500_hardware_reset();
void set_esp32_mac_on_w5500(esp_eth_handle_t eth_handle);
static std::string sync_rtc_from_system_time();
static void format_fw_version(uint16_t ver, char* out, size_t out_len);

static void reset_output_pwm_states() {
    for (int i = 0; i < MAX_OUTPUTS; ++i) {
        outputs[i].startPwmValue = 0;
        outputs[i].currentPwmValue = 0;
        outputs[i].targetPwmValue = 0;
        outputs[i].fadeStartTime = 0;
        outputs[i].fadeDuration = DEFAULT_FADE_MS;
    }
}

static esp_err_t apply_output_pwm(int logical_output, int pwm_value,
                                  uint8_t* out_addr, uint8_t* out_channel) {
    uint8_t addr = 0;
    uint8_t channel = 0;
    if (!hsg_outputs_get_mapping(logical_output, &addr, &channel)) {
        ESP_LOGW(TAG, "No PCA mapping for output %d", logical_output);
        return ESP_ERR_NOT_FOUND;
    }
    if (out_addr) *out_addr = addr;
    if (out_channel) *out_channel = channel;
    esp_err_t result = hsg_pca9685::pca9685_write_pwm_value(addr, channel, (uint16_t)pwm_value);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write PWM %d to PCA@0x%02X ch%d: %s",
                 pwm_value, addr, channel, esp_err_to_name(result));
        return result;
    }
    ESP_LOGI(TAG, "Output %d -> PCA@0x%02X ch%d PWM=%d", logical_output, addr, channel, pwm_value);
    return ESP_OK;
}

static esp_err_t setOutputEx(int output, int brightness, int fadeMs, unsigned long startTimeOffset = 0) {
    int outputIndex = output - 1;
    if (outputIndex < 0 || outputIndex >= MAX_OUTPUTS) return ESP_ERR_INVALID_ARG;

    const int newTarget = (int)roundf(brightness / 100.0f * 4095.0f);
    outputs[outputIndex].startPwmValue = outputs[outputIndex].currentPwmValue;
    outputs[outputIndex].targetPwmValue = newTarget;
    outputs[outputIndex].fadeStartTime = esp_log_timestamp() + startTimeOffset;
    outputs[outputIndex].fadeDuration = (fadeMs > 0) ? fadeMs : 1;

    if (brightness > 0)
        outputBrightness[outputIndex] = brightness;

    if (fadeMs <= 0) {
        outputs[outputIndex].currentPwmValue = newTarget;
        outputs[outputIndex].startPwmValue = newTarget;
        return apply_output_pwm(output, newTarget, nullptr, nullptr);
    }
    return ESP_OK;
}

void setOutput(int output, int brightness, int fadeMs, unsigned long startTimeOffset) {
    setOutputEx(output, brightness, fadeMs, startTimeOffset);
}

// Background task to automatically keep the DS3231 RTC in sync with system time.
// Behaviour:
//  - After boot, waits until system time is valid (NTP has synced) and then performs
//    a one-time RTC sync for this boot (if not already done).
//  - After that, re-syncs the RTC periodically every RTC_AUTO_SYNC_INTERVAL_SEC.
static void rtc_auto_sync_task(void *pvParameter)
{
    (void)pvParameter;

    if (!ds3231_is_installed()) {
        vTaskDelete(NULL);
        return;
    }

    time_t last_sync_time = 0;

    for (;;) {
        time_t now = time(nullptr);

        if (now >= 1600000000) { // system time considered valid
            // One-time sync per boot after first valid system time
            if (!s_rtc_auto_synced_this_boot) {
                std::string err = sync_rtc_from_system_time();
                if (err.empty()) {
                    s_rtc_auto_synced_this_boot = true;
                    last_sync_time = now;
                    ESP_LOGI(TAG, "RTC auto-sync: completed initial sync after NTP.");
                } else {
                    ESP_LOGW(TAG, "RTC auto-sync (initial) failed: %s", err.c_str());
                }
            } else {
                // Periodic sync while running
                if (last_sync_time == 0 || (now - last_sync_time) >= RTC_AUTO_SYNC_INTERVAL_SEC) {
                    std::string err = sync_rtc_from_system_time();
                    if (err.empty()) {
                        last_sync_time = now;
                        ESP_LOGI(TAG, "RTC auto-sync: completed periodic sync.");
                    } else {
                        ESP_LOGW(TAG, "RTC auto-sync (periodic) failed: %s", err.c_str());
                    }
                }
            }
        }

        // Check once per minute; this is sufficient for "after NTP" detection and 24h interval.
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}

void processFades() {
    unsigned long now = esp_log_timestamp(); // Get the current time once per loop

    for (int i = 0; i < MAX_OUTPUTS; i++) {
        // Check if a fade is active for this output
        if (outputs[i].currentPwmValue != outputs[i].targetPwmValue) {
            
            if (now < outputs[i].fadeStartTime) {
                continue; // Not time to start, skip to the next output
            }
            
            unsigned long elapsedTime = now - outputs[i].fadeStartTime;
            int newPwmValue;

            if (elapsedTime >= outputs[i].fadeDuration) {
                // Fade is complete, snap to the target value
                newPwmValue = outputs[i].targetPwmValue;
            } else {
                // Fade is in progress, calculate the intermediate value (linear interpolation)
                float progress = (float)elapsedTime / (float)outputs[i].fadeDuration;
                newPwmValue = outputs[i].startPwmValue + (progress * (outputs[i].targetPwmValue - outputs[i].startPwmValue));
            }

            // Only update the physical PWM chip if the value has actually changed
            if (newPwmValue != outputs[i].currentPwmValue) {
                uint8_t addr;
                uint8_t channel;
                if (hsg_outputs_get_mapping(i + 1, &addr, &channel)) {
                    esp_err_t result = hsg_pca9685::pca9685_write_pwm_value(addr, channel, (uint16_t)newPwmValue);
                    if (result != ESP_OK) {
                        ESP_LOGE(TAG, "Failed to write PWM value to PCA@0x%02X ch%d. Error: %s", addr, channel, esp_err_to_name(result));
                    } else {
                        outputs[i].currentPwmValue = newPwmValue;
                    }
                } else {
                    outputs[i].currentPwmValue = newPwmValue;
                }
            }
        }
    }
}

static HSG::API::CommandFeedback processCommand(const Command& cmd) {
    HSG::API::CommandFeedback fb;
    fb.mapped_outputs_total = hsg_outputs_get_mapped_count();
    fb.state = cmd.state;
    fb.fade_ms = cmd.fade_ms;

    if (cmd.type == TargetType::OUTPUT) {
        fb.target_type = "output";
        fb.output = cmd.output_id;
        ESP_LOGI(TAG, "Processing command for output: %d, state=%s, brightness=%d, fade=%dms",
                cmd.output_id, cmd.state.c_str(), cmd.brightness, cmd.fade_ms);
    } else {
        fb.target_type = "group";
        fb.group = cmd.group_name;
        ESP_LOGI(TAG, "Processing command for group: '%s', state=%s, brightness=%d, fade=%dms",
                cmd.group_name.c_str(), cmd.state.c_str(), cmd.brightness, cmd.fade_ms);
    }
    
    int final_brightness = 0;
    if (cmd.state == "OFF") {
        final_brightness = 0;
    } else if (cmd.state == "TOGGLE") {
        bool should_turn_on = true;

        if (cmd.type == TargetType::GROUP) {
            cJSON* config = HSG::API::get_config_json_obj();
            if (config) {
                cJSON* groups = cJSON_GetObjectItem(config, "groups");
                cJSON* specific_group = cJSON_GetObjectItem(groups, cmd.group_name.c_str());

                if (cJSON_IsObject(specific_group)) {
                    cJSON* group_outputs = cJSON_GetObjectItem(specific_group, "outputs");

                    if (cJSON_IsArray(group_outputs)) {
                        cJSON* output_item = NULL;
                        cJSON_ArrayForEach(output_item, group_outputs) {
                            int output_id_to_check = output_item->valueint;
                            if (output_id_to_check > 0 && output_id_to_check <= MAX_OUTPUTS) {
                                if (outputs[output_id_to_check - 1].targetPwmValue > 0) {
                                    should_turn_on = false;
                                    break;
                                }
                            }
                        }
                    }
                }
                cJSON_Delete(config);
            }
        } else if (cmd.output_id > 0 && cmd.output_id <= MAX_OUTPUTS) {
            if (outputs[cmd.output_id - 1].targetPwmValue > 0) {
                should_turn_on = false;
            }
        }

        if (should_turn_on) {
            final_brightness = (cmd.brightness > 0) ? cmd.brightness : 50;
        } else {
            final_brightness = 0;
        }
    } else if (cmd.state == "ON") {
        final_brightness = (cmd.brightness > 0) ? cmd.brightness : 50;
    } else if (cmd.state.empty()) {
        final_brightness = cmd.brightness;
    } else {
        final_brightness = 0;
    }
    ESP_LOGI(TAG, "Final brightness=%d", final_brightness);
    
    if (final_brightness < 0) final_brightness = 0;
    if (final_brightness > 100) final_brightness = 100; 

    if (cmd.type == TargetType::OUTPUT) {
        fb.brightness_pct = final_brightness;
        fb.pwm = (int)roundf(final_brightness / 100.0f * 4095.0f);

        uint8_t addr = 0, channel = 0;
        fb.mapping_found = hsg_outputs_get_mapping(cmd.output_id, &addr, &channel);
        if (fb.mapping_found) {
            fb.pca_addr = addr;
            fb.pca_channel = channel;
        }

        esp_err_t write_result = setOutputEx(cmd.output_id, final_brightness, cmd.fade_ms, 0);
        fb.outputs_affected = 1;

        if (!fb.mapping_found) {
            fb.ok = false;
            fb.message = "No PCA9685 mapping for output " + std::to_string(cmd.output_id);
        } else if (cmd.fade_ms > 0) {
            fb.deferred_fade = true;
            fb.ok = true;
            fb.message = "Command accepted; PWM will update during fade";
        } else if (write_result == ESP_OK) {
            fb.i2c_written = true;
            fb.ok = true;
            fb.message = "I2C PWM write OK";
        } else {
            fb.ok = false;
            fb.i2c_error = esp_err_to_name(write_result);
            fb.message = std::string("I2C PWM write failed: ") + fb.i2c_error;
        }
        return fb;
    } else if (cmd.type == TargetType::GROUP) {
        cJSON* config = HSG::API::get_config_json_obj();
        if (!config) {
            ESP_LOGE(TAG, "Failed to get config to process group command.");
            fb.ok = false;
            fb.message = "Failed to load configuration";
            return fb;
        }

        fb.brightness_pct = final_brightness;
        fb.pwm = (int)roundf(final_brightness / 100.0f * 4095.0f);

        int global_stagger_ms = 0;
        cJSON* settings = cJSON_GetObjectItem(config, "settings");
        if (settings) {
            global_stagger_ms = cJSON_GetObjectItem(settings, "groupStaggerMs") ? cJSON_GetObjectItem(settings, "groupStaggerMs")->valueint : 0;
        }
        cJSON* groups = cJSON_GetObjectItem(config, "groups");
        cJSON* specific_group = cJSON_GetObjectItem(groups, cmd.group_name.c_str());

        if (cJSON_IsObject(specific_group)) {
            bool stagger_is_enabled = cJSON_IsTrue(cJSON_GetObjectItem(specific_group, "staggerEnabled"));
            cJSON* group_outputs = cJSON_GetObjectItem(specific_group, "outputs");

            if (cJSON_IsArray(group_outputs)) {
                int channel_index = 0;
                int i2c_ok_count = 0;
                int i2c_fail_count = 0;
                int unmapped_count = 0;
                cJSON* output_item = NULL;
                cJSON_ArrayForEach(output_item, group_outputs) {
                    if (cJSON_IsNumber(output_item)) {
                        int output_id = output_item->valueint;
                        unsigned long delay_offset = 0;

                        if (stagger_is_enabled && global_stagger_ms > 0) {
                            delay_offset = channel_index * global_stagger_ms;
                        }
                        uint8_t addr = 0, ch = 0;
                        if (!hsg_outputs_get_mapping(output_id, &addr, &ch))
                            unmapped_count++;
                        esp_err_t wr = setOutputEx(output_id, final_brightness, cmd.fade_ms, delay_offset);
                        if (cmd.fade_ms <= 0) {
                            if (wr == ESP_OK) i2c_ok_count++;
                            else if (wr != ESP_ERR_NOT_FOUND) i2c_fail_count++;
                        }
                        fb.outputs_affected++;
                        channel_index++;
                    }
                }
                if (cmd.fade_ms > 0) {
                    fb.deferred_fade = true;
                    fb.ok = fb.outputs_affected > 0;
                    fb.message = "Group command accepted; fade in progress for " +
                        std::to_string(fb.outputs_affected) + " output(s)";
                } else if (i2c_fail_count > 0) {
                    fb.ok = false;
                    fb.message = "Group: " + std::to_string(i2c_ok_count) + " I2C OK, " +
                        std::to_string(i2c_fail_count) + " failed, " +
                        std::to_string(unmapped_count) + " unmapped";
                } else if (unmapped_count > 0 && i2c_ok_count == 0) {
                    fb.ok = false;
                    fb.message = "Group: no mapped outputs (" + std::to_string(unmapped_count) + " unmapped)";
                } else {
                    fb.ok = true;
                    fb.i2c_written = i2c_ok_count > 0;
                    fb.message = "Group: " + std::to_string(i2c_ok_count) + " I2C write(s) OK";
                    if (unmapped_count > 0)
                        fb.message += ", " + std::to_string(unmapped_count) + " unmapped skipped";
                }
            } else {
                fb.ok = false;
                fb.message = "Group has no outputs configured";
            }
        } else {
            ESP_LOGW(TAG, "Group '%s' not found in configuration.", cmd.group_name.c_str());
            fb.ok = false;
            fb.message = "Group '" + cmd.group_name + "' not found";
        }
        cJSON_Delete(config);
    }
    return fb;
}
/* ---------------------- Network Logic ---------------------- */
static void phy_power_set(bool on) {
    #if defined(BOARD_OLIMEX_POE)
        static bool inited = false;
        if (!inited) {
            gpio_config_t io = {};
            io.pin_bit_mask = 1ULL << ETH_PHY_PWR_GPIO;
            io.mode = GPIO_MODE_OUTPUT;
            gpio_config(&io);
            inited = true;
        }
        gpio_set_level((gpio_num_t)ETH_PHY_PWR_GPIO, on ? 1 : 0);
    #endif    
}

static void net_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    if (event_base == ETH_EVENT) {
        if (event_id == ETHERNET_EVENT_CONNECTED) {
            ESP_LOGI(TAG, "Ethernet Link Up");
            if (s_wifi_netif != NULL) {
                ESP_ERROR_CHECK(esp_wifi_stop());
            }
            s_wifi_connected = false;
        } else if (event_id == ETHERNET_EVENT_DISCONNECTED) {
            ESP_LOGI(TAG, "Ethernet Link Down");
            s_eth_connected = false;
            if (s_outputs_init_done) {
                wifi_start();
            } else {
                s_wifi_failover_pending = true;
                ESP_LOGI(TAG, "Wi-Fi failover deferred until PCA/output init completes");
            }
        }
    }
    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_STA_START) {
            // FIX: This is the critical missing piece.
            // This event means the Wi-Fi hardware is ready, now we can connect.
            esp_wifi_connect();
            ESP_LOGI(TAG, "Wi-Fi connecting...");
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            if (!s_eth_connected) {
                ESP_LOGI(TAG, "Wi-Fi disconnected, retrying...");
                esp_wifi_connect();
            }
        }
    }
    if (event_base == IP_EVENT) {
        esp_mqtt_client_handle_t mqtt_client = HSG::API::get_mqtt_client();
        if (event_id == IP_EVENT_ETH_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Ethernet Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            esp_netif_set_default_netif(s_eth_netif);
            s_eth_connected = true;
            if (mqtt_client) esp_mqtt_client_reconnect(mqtt_client);
            xEventGroupSetBits(s_net_event_group, ETH_CONNECTED_BIT);
        } else if (event_id == IP_EVENT_STA_GOT_IP) {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
            ESP_LOGI(TAG, "Wi-Fi Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            if (!s_eth_connected) {
                esp_netif_set_default_netif(s_wifi_netif);
            }
            s_wifi_connected = true;
            if (mqtt_client) esp_mqtt_client_reconnect(mqtt_client);
            xEventGroupSetBits(s_net_event_group, WIFI_CONNECTED_BIT);
        }
        if (!s_sntp_started) {
            setenv("TZ", "UTC0", 1);
            tzset();
            esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
            sntp_cfg.wait_for_sync = false;
            sntp_cfg.start = true;
            if (esp_netif_sntp_init(&sntp_cfg) == ESP_OK) {
                s_sntp_started = true;
                ESP_LOGI(TAG, "SNTP started (pool.ntp.org), system time will sync from internet");
            } else {
                ESP_LOGW(TAG, "SNTP init failed");
            }
        }
    }
}

// Helper used by both manual "Sync RTC" API callback and automatic RTC sync task.
// Returns empty string on success, or a human-readable error message on failure.
static std::string sync_rtc_from_system_time()
{
    if (!ds3231_is_installed()) {
        return "DS3231 RTC not installed";
    }

    time_t t = time(nullptr);
    if (t < 1600000000) {  // system time not set or unreasonable (before Sep 2020)
        return "System time not set or too old. Configure NTP or set time first.";
    }

    struct tm tm;
    if (!gmtime_r(&t, &tm)) {
        return "System time invalid.";
    }

    if (ds3231_set_time(i2c_bus_handle, &tm) != ESP_OK) {
        return "RTC write failed (check I2C/DS3231).";
    }

    struct tm readback;
    if (ds3231_get_time(i2c_bus_handle, &readback) == ESP_OK) {
        char buf[32];
        if (strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC", &readback) > 0) {
            ESP_LOGI(TAG, "RTC readback after sync: %s", buf);
            g_rtc_cache = buf;
            g_rtc_cache_ticks = xTaskGetTickCount();
        }
    }

    return "";
}

#if defined(BOARD_OLIMEX_POE)
static void eth_start(void)
{
    // Create Ethernet network interface
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    s_eth_netif = eth_netif;

     // Power up PHY first (GPIO12)
    phy_power_set(true);
    vTaskDelay(pdMS_TO_TICKS(50));  // let 3V3 & XO settle

    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_mdc_gpio_num = ETH_MDC_GPIO;
    esp32_emac_config.smi_mdio_gpio_num = ETH_MDIO_GPIO;
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_RST_GPIO;
    
    // Power on the PHY
    gpio_set_direction((gpio_num_t)ETH_PHY_PWR_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)ETH_PHY_PWR_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // New MAC creation API (no more esp32_emac_config struct)
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
      
    // Install Ethernet driver
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));
    void *glue = esp_eth_new_netif_glue(eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, glue));
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}
#elif defined(BOARD_RACK32)
static void eth_start(void) {
    ESP_LOGI(TAG, "Initializing W5500 with hardware reset...");
    vTaskDelay(pdMS_TO_TICKS(100));
    w5500_hardware_reset();

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << (gpio_num_t)W5500_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf);

    spi_bus_config_t buscfg = {
        .mosi_io_num = W5500_MOSI_GPIO,
        .miso_io_num = W5500_MISO_GPIO,
        .sclk_io_num = W5500_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096
    };
    ESP_ERROR_CHECK(spi_bus_initialize(W5500_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);
    s_eth_netif = eth_netif;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = -1;              // No SMI address
    phy_config.reset_gpio_num = -1;        // No reset GPIO via PHY API
    phy_config.autonego_timeout_ms = 0;
    phy_config.reset_timeout_ms = 0;

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 20 * 1000 * 1000,
        .spics_io_num = W5500_CS_GPIO,
        .queue_size = 20
    };

    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(W5500_HOST, &devcfg);
    w5500_config.int_gpio_num = W5500_INT_GPIO;

    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);

    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    set_esp32_mac_on_w5500(eth_handle);
    
    void *glue = esp_eth_new_netif_glue(eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(s_eth_netif, glue));

    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}
#endif

#if defined(BOARD_RACK32)
void set_esp32_mac_on_w5500(esp_eth_handle_t eth_handle) {
    uint8_t esp_mac[6];
    ESP_ERROR_CHECK(esp_read_mac(esp_mac, ESP_MAC_ETH));
    
    // Convert to locally administered address
    esp_mac[0] = (esp_mac[0] & 0xFE) | 0x02;  // Set bit 1, clear bit 0
    
    esp_err_t ret = esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, esp_mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ESP32 MAC on W5500: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "ESP32 MAC applied to W5500: %02x:%02x:%02x:%02x:%02x:%02x", 
                 esp_mac[0], esp_mac[1], esp_mac[2],
                 esp_mac[3], esp_mac[4], esp_mac[5]);
    }
}
#endif

#if defined(BOARD_RACK32)
static void w5500_hardware_reset() {
    // Configure reset GPIO
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << (gpio_num_t)W5500_RST_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&rst_conf);
    
    // Perform reset sequence
    gpio_set_level((gpio_num_t)W5500_RST_GPIO, 0);  // Assert reset
    vTaskDelay(pdMS_TO_TICKS(10));      // Hold for 10ms
    gpio_set_level((gpio_num_t)W5500_RST_GPIO, 1);  // De-assert reset
    vTaskDelay(pdMS_TO_TICKS(100));     // Wait for W5500 to stabilize
}
#endif

// --- Wi-Fi Start Function ---
static void wifi_start(void)
{
    ESP_LOGI(TAG, "Attempting to start Wi-Fi as failover...");
    
    // Create Wi-Fi station interface if it doesn't exist
    if (s_wifi_netif == NULL) {
        s_wifi_netif = esp_netif_create_default_wifi_sta();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    }

    
    // 1. Get the entire config object from your API
    cJSON* config_json = HSG::API::get_config_json_obj();
    if (!config_json) {
        ESP_LOGE(TAG, "Failed to get config from NVS. Cannot start Wi-Fi.");
        return;
    }

    // 2. Extract the 'wifi' section
    cJSON* wifi_json = cJSON_GetObjectItem(config_json, "wifi");
    if (!wifi_json) {
        ESP_LOGW(TAG, "No 'wifi' section in config. Wi-Fi not started.");
        cJSON_Delete(config_json);
        return;
    }
    // FIX: Add a check for the "ssid" item
    const cJSON* ssid_json = cJSON_GetObjectItem(wifi_json, "ssid");
    if (!cJSON_IsString(ssid_json) || ssid_json->valuestring == NULL || strlen(ssid_json->valuestring) == 0) {
        ESP_LOGW(TAG, "Wi-Fi SSID not configured or is empty. Wi-Fi not started.");
        cJSON_Delete(config_json);
        return;
    }

    // --- FIX: Add these log messages for verification ---
    ESP_LOGI(TAG, "Found Wi-Fi credentials in NVS:");
    ESP_LOGI(TAG, "  SSID: %s", ssid_json->valuestring);

    const cJSON* pass_json = cJSON_GetObjectItem(wifi_json, "password");
    if (cJSON_IsString(pass_json) && pass_json->valuestring != NULL && strlen(pass_json->valuestring) > 0) {
        ESP_LOGI(TAG, "  Password: [***]"); // We log stars for security, not the actual password
    } else {
        ESP_LOGI(TAG, "  Password: [NONE]");
    }
    // --- END FIX ---

    // 3. Configure Wi-Fi with credentials from NVS
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid_json->valuestring, sizeof(wifi_config.sta.ssid) - 1);
    if (cJSON_IsString(pass_json) && pass_json->valuestring != NULL) {
        strncpy((char*)wifi_config.sta.password, pass_json->valuestring, sizeof(wifi_config.sta.password) - 1);
    }
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    cJSON_Delete(config_json); // Clean up JSON object

    // Create Wi-Fi station interface if it doesn't exist
    if (esp_netif_get_handle_from_ifkey("WIFI_STA_DEF") == NULL) {
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    }
    
    // 4. Set config and start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Wi-Fi configured to connect to SSID: %s", wifi_config.sta.ssid);
}


// --- Main Network Initialization ---
static void initialize_network_interfaces(void) {
    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
  
    // Register our unified event handler for all network events
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &net_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &net_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &net_event_handler, NULL));

    // Start with Ethernet by default
    eth_start();
}


httpd_handle_t web_server_start(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.max_uri_handlers = 20; // Default is usually around 8

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

// This function can be called by the API to get the current state
cJSON* get_current_outputs_json() {
    cJSON* outputs_json = cJSON_CreateObject();
    for (int i = 0; i < MAX_OUTPUTS; ++i) {
        // Only include outputs that are on to keep the payload small
        if (outputs[i].currentPwmValue > 0) {
            char key[5];
            snprintf(key, sizeof(key), "%d", i + 1);
            cJSON_AddNumberToObject(outputs_json, key, outputs[i].currentPwmValue);
        }
    }
    return outputs_json; // The caller is responsible for deleting this object
}

/* ---------------------- Hardware & Network Init ---------------------- */
static esp_err_t can_module_init(void) {

    spi_bus_config_t can_buscfg = {
        .mosi_io_num = CAN_MOSI_GPIO,
        .miso_io_num = CAN_MISO_GPIO,
        .sclk_io_num = CAN_CLK_GPIO,
        .quadwp_io_num = -1, // Not used
        .quadhd_io_num = -1, // Not used
        .max_transfer_sz = 32
    };
    esp_err_t ret = spi_bus_initialize(CAN_HOST, &can_buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN SPI bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    spi_device_interface_config_t can_devcfg = {
        .mode = 0,                         // SPI mode 0
        .clock_speed_hz = 5 * 1000 * 1000, // 5 MHz — MCP2515 margin on shared bus
        .input_delay_ns = 50,
        .spics_io_num = CAN_CS_GPIO,
        .queue_size = 10,
    };

    //Attach the MCP2515 to the SPI bus
    ret = spi_bus_add_device(CAN_HOST, &can_devcfg, &can_spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add CAN SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "CAN SPI initialized successfully on SPI3_HOST");
    return ESP_OK;
}


/*--------------------------- Fading and PWM Logic ---------------------------*/

void reload_bindings() {
    if (xSemaphoreTake(g_bindings_mutex, portMAX_DELAY) == pdTRUE) {
        g_bindings.clear();
        cJSON* config = HSG::API::get_config_json_obj();
        if (!config) {
            xSemaphoreGive(g_bindings_mutex);
            return;
        }

        // --- ADD THIS DIAGNOSTIC BLOCK ---
    /*    char* json_string = cJSON_Print(config); // cJSON_Print makes it nicely formatted
        if (json_string) {
            printf("\n=============== START of Stored bindings.json ===============\n");
            printf("%s\n", json_string);
            printf("================ END of Stored bindings.json =================\n\n");
            free(json_string); // IMPORTANT: Free the memory allocated by cJSON_Print
        }
        // --- END DIAGNOSTIC BLOCK ---
    */

        cJSON* bindings_json = cJSON_GetObjectItem(config, "bindings");
        if (cJSON_IsArray(bindings_json)) {
            cJSON* rule_json = NULL;
            cJSON_ArrayForEach(rule_json, bindings_json) {
                Binding b;
                // Parse Trigger
                cJSON* trigger = cJSON_GetObjectItem(rule_json, "trigger");
                b.switchId = cJSON_GetObjectItem(trigger, "switchId")->valueint;
                b.button = cJSON_GetObjectItem(trigger, "button")->valueint;
                b.onAction = cJSON_GetObjectItem(trigger, "action")->valuestring;
                if (!isEventActionString(b.onAction)) {
                    ESP_LOGW(TAG, "Binding has unsupported CAN trigger action '%s' (supported: CLICK/HOLD/DOUBLE_CLICK/TRIPLE_CLICK/LONG_HOLD/HOLD_REPEAT)",
                             b.onAction.c_str());
                }

                // Parse Action
                cJSON* action = cJSON_GetObjectItem(rule_json, "action");
                if (cJSON_HasObjectItem(action, "output")) {
                    b.targetType = TargetType::OUTPUT;
                    b.outputId = cJSON_GetObjectItem(action, "output")->valueint;
                } else if (cJSON_HasObjectItem(action, "group")) {
                    b.targetType = TargetType::GROUP;
                    b.groupName = cJSON_GetObjectItem(action, "group")->valuestring;
                }
                b.state = cJSON_GetObjectItem(action, "state") ? cJSON_GetObjectItem(action, "state")->valuestring : "";
                b.brightness = cJSON_GetObjectItem(action, "brightness") ? cJSON_GetObjectItem(action, "brightness")->valueint : -1; // -1 indicates not set
                b.fade_ms = cJSON_GetObjectItem(action, "fade") ? cJSON_GetObjectItem(action, "fade")->valueint : 0;
                
                g_bindings.push_back(b);
            }
        }

        cJSON_Delete(config);
        ESP_LOGI(TAG, "Loaded %d CAN bindings", g_bindings.size());
        xSemaphoreGive(g_bindings_mutex);
    }
}

static void load_nodes_from_config(void) {
    if (!g_nodes_mutex) {
        ESP_LOGE(TAG, "load_nodes_from_config: g_nodes_mutex not initialized");
        return;
    }
    
    if (xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
        g_nodes.clear();
        cJSON* config = HSG::API::get_config_json_obj();
        if (!config) {
            xSemaphoreGive(g_nodes_mutex);
            return;
        }

        cJSON* nodes_json = cJSON_GetObjectItem(config, "nodes");
        if (cJSON_IsArray(nodes_json)) {
            cJSON* node_json = NULL;
            cJSON_ArrayForEach(node_json, nodes_json) {
                NodeInfo node;
                cJSON* item = cJSON_GetObjectItem(node_json, "node_id");
                if (!item || !cJSON_IsNumber(item)) continue;
                node.node_id = (uint8_t)item->valueint;
                
                item = cJSON_GetObjectItem(node_json, "node_type");
                node.node_type = (item && cJSON_IsNumber(item)) ? (uint8_t)item->valueint : 0;
                
                item = cJSON_GetObjectItem(node_json, "input_count");
                node.input_count = (item && cJSON_IsNumber(item)) ? (uint8_t)item->valueint : 0;
                
                // Do not restore last_seen from config: it was "microseconds since boot" and is
                // meaningless after reboot. Nodes show offline until they send a HEARTBEAT.
                node.last_seen_timestamp_us = 0;
                
                item = cJSON_GetObjectItem(node_json, "is_configured");
                node.is_configured = (item && cJSON_IsBool(item)) ? cJSON_IsTrue(item) : (node.node_id != NODE_ID_UNCONFIGURED);

                cJSON* input_config = cJSON_GetObjectItem(node_json, "input_config");
                if (cJSON_IsArray(input_config)) {
                    cJSON* entry = NULL;
                    cJSON_ArrayForEach(entry, input_config) {
                        cJSON* idx = cJSON_GetObjectItem(entry, "input_index");
                        cJSON* mode_item = cJSON_GetObjectItem(entry, "mode");
                        cJSON* label_item = cJSON_GetObjectItem(entry, "label");
                        cJSON* gpio_item = cJSON_GetObjectItem(entry, "gpio");
                        cJSON* active_high_item = cJSON_GetObjectItem(entry, "active_high");
                        if (cJSON_IsNumber(idx) && idx->valueint >= 0 && idx->valueint < MAX_INPUTS_PER_NODE) {
                            if (cJSON_IsNumber(mode_item))
                                node.input_modes[idx->valueint] = (uint8_t)(mode_item->valueint & 1);
                            if (cJSON_IsString(label_item) && label_item->valuestring) {
                                strncpy(node.input_labels[idx->valueint], label_item->valuestring, MAX_INPUT_LABEL_LEN);
                                node.input_labels[idx->valueint][MAX_INPUT_LABEL_LEN] = '\0';
                            }
                            if (cJSON_IsNumber(gpio_item) && ((gpio_item->valueint >= 0 && gpio_item->valueint <= 48) || gpio_item->valueint == 255))
                                node.input_gpio[idx->valueint] = (uint8_t)(gpio_item->valueint & 0xFF);
                            if (cJSON_IsBool(active_high_item))
                                node.input_active_high[idx->valueint] = cJSON_IsTrue(active_high_item) ? 1 : 0;
                            else if (cJSON_IsNumber(active_high_item))
                                node.input_active_high[idx->valueint] = (uint8_t)(active_high_item->valueint & 1);
                        }
                    }
                }
                item = cJSON_GetObjectItem(node_json, "find_me_output_index");
                if (node.node_type == NODE_TYPE_MECHANICAL) {
                    node.find_me_output_index = 0xFF;
                } else if (item && cJSON_IsNumber(item)) {
                    node.find_me_output_index = (uint8_t)(item->valueint & 0xFF);
                }
                item = cJSON_GetObjectItem(node_json, "night_light_on");
                if (item && cJSON_IsBool(item)) node.night_light_on = cJSON_IsTrue(item);
                item = cJSON_GetObjectItem(node_json, "night_light_brightness");
                if (item && cJSON_IsNumber(item)) node.night_light_brightness = (uint8_t)(item->valueint & 0xFF);
                item = cJSON_GetObjectItem(node_json, "ws2812_click_effect");
                if (item && cJSON_IsNumber(item)) node.ws2812_click_effect = (item->valueint != 0) ? 1 : 0;
                load_ws2812_effects_from_json(node, node_json);
                item = cJSON_GetObjectItem(node_json, "fw_version");
                if (item && cJSON_IsNumber(item)) node.fw_version = (uint16_t)item->valueint;
                item = cJSON_GetObjectItem(node_json, "ota_capable");
                if (item && cJSON_IsNumber(item)) node.ota_capable = (uint8_t)(item->valueint & 1);
                item = cJSON_GetObjectItem(node_json, "last_ota_result");
                if (item && cJSON_IsString(item) && item->valuestring)
                    strncpy(node.last_ota_result, item->valuestring, sizeof(node.last_ota_result) - 1);

                g_nodes[node.node_id] = node;
            }
        }

        cJSON_Delete(config);
        ESP_LOGI(TAG, "Loaded %d nodes from config", (int)g_nodes.size());
        xSemaphoreGive(g_nodes_mutex);
    }
}

static void save_nodes_to_config(void) {
    if (!g_nodes_mutex) {
        ESP_LOGE(TAG, "save_nodes_to_config: g_nodes_mutex not initialized");
        return;
    }
    
    if (xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
        // Get the config object (this returns just the "config" part, not the root)
        cJSON* config = HSG::API::get_config_json_obj();
        if (!config) {
            xSemaphoreGive(g_nodes_mutex);
            return;
        }

        // Get or create nodes array
        cJSON* nodes_json = cJSON_GetObjectItem(config, "nodes");
        if (!nodes_json || !cJSON_IsArray(nodes_json)) {
            cJSON_DeleteItemFromObject(config, "nodes");
            nodes_json = cJSON_CreateArray();
            cJSON_AddItemToObject(config, "nodes", nodes_json);
        } else {
            // Clear existing array by deleting all items
            while (cJSON_GetArraySize(nodes_json) > 0) {
                cJSON_DeleteItemFromArray(nodes_json, 0);
            }
        }

        // Add all nodes to array
        for (const auto& pair : g_nodes) {
            const NodeInfo& node = pair.second;
            cJSON* node_json = cJSON_CreateObject();
            cJSON_AddNumberToObject(node_json, "node_id", node.node_id);
            cJSON_AddNumberToObject(node_json, "node_type", node.node_type);
            cJSON_AddNumberToObject(node_json, "input_count", node.input_count);
            cJSON_AddNumberToObject(node_json, "last_seen", (double)node.last_seen_timestamp_us);
            cJSON_AddBoolToObject(node_json, "is_configured", node.is_configured);
            if (node.node_type != NODE_TYPE_MECHANICAL && node.find_me_output_index != 0xFF)
                cJSON_AddNumberToObject(node_json, "find_me_output_index", node.find_me_output_index);
            if (node.node_type == NODE_TYPE_MECHANICAL) {
                cJSON_AddBoolToObject(node_json, "night_light_on", node.night_light_on);
                cJSON_AddNumberToObject(node_json, "night_light_brightness", node.night_light_brightness);
                cJSON_AddNumberToObject(node_json, "ws2812_click_effect", node.ws2812_click_effect);
                add_ws2812_effects_json(node_json, node);
                cJSON_AddNumberToObject(node_json, "fw_version", node.fw_version);
                cJSON_AddNumberToObject(node_json, "ota_capable", node.ota_capable);
                if (node.last_ota_result[0] != '\0')
                    cJSON_AddStringToObject(node_json, "last_ota_result", node.last_ota_result);
            }
            int n_in = (node.input_count < MAX_INPUTS_PER_NODE) ? node.input_count : MAX_INPUTS_PER_NODE;
            if (n_in > 0) {
                cJSON* input_config = cJSON_CreateArray();
                for (int i = 0; i < n_in; i++) {
                    cJSON* entry = cJSON_CreateObject();
                    cJSON_AddNumberToObject(entry, "input_index", i);
                    cJSON_AddNumberToObject(entry, "input_id", i + 1);
                    uint8_t m = (node.input_modes[i] == 0xFF) ? 0 : (node.input_modes[i] & 1);
                    cJSON_AddNumberToObject(entry, "mode", m);
                    if (node.input_labels[i][0] != '\0')
                        cJSON_AddStringToObject(entry, "label", node.input_labels[i]);
                    if (node.input_gpio[i] != 0xFF)
                        cJSON_AddNumberToObject(entry, "gpio", node.input_gpio[i]);
                    cJSON_AddBoolToObject(entry, "active_high", node.input_active_high[i] != 0);
                    cJSON_AddItemToArray(input_config, entry);
                }
                cJSON_AddItemToObject(node_json, "input_config", input_config);
            }
            cJSON_AddItemToArray(nodes_json, node_json);
        }

        // Convert to string and save using public API
        char* json_string = cJSON_PrintUnformatted(config);
        if (json_string) {
            HSG::API::set_config_json(json_string);
            free(json_string);
        }

        cJSON_Delete(config);
        xSemaphoreGive(g_nodes_mutex);
    }
}

static void save_nodes_task_fn(void* pv) {
    save_nodes_to_config();
    vTaskDelete(NULL);
}

static void defer_save_nodes_to_config(void) {
    xTaskCreate(save_nodes_task_fn, "save_nodes", 4096, NULL, 1, NULL);
}

// Returns JSON array of g_nodes only (persistable format). Used by config POST so we never re-save removed nodes from stale NVS.
static cJSON* get_nodes_json_for_config_save(void) {
    cJSON* root = cJSON_CreateArray();
    if (!root) return nullptr;
    if (!g_nodes_mutex) return root;
    if (xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) != pdTRUE) return root;
    for (const auto& pair : g_nodes) {
        const NodeInfo& node = pair.second;
        cJSON* node_json = cJSON_CreateObject();
        cJSON_AddNumberToObject(node_json, "node_id", node.node_id);
        cJSON_AddNumberToObject(node_json, "node_type", node.node_type);
        cJSON_AddNumberToObject(node_json, "input_count", node.input_count);
        cJSON_AddNumberToObject(node_json, "last_seen", (double)node.last_seen_timestamp_us);
        cJSON_AddBoolToObject(node_json, "is_configured", node.is_configured);
        if (node.node_type != NODE_TYPE_MECHANICAL && node.find_me_output_index != 0xFF)
            cJSON_AddNumberToObject(node_json, "find_me_output_index", node.find_me_output_index);
        if (node.node_type == NODE_TYPE_MECHANICAL) {
            cJSON_AddBoolToObject(node_json, "night_light_on", node.night_light_on);
            cJSON_AddNumberToObject(node_json, "night_light_brightness", node.night_light_brightness);
            cJSON_AddNumberToObject(node_json, "ws2812_click_effect", node.ws2812_click_effect);
            add_ws2812_effects_json(node_json, node);
            cJSON_AddNumberToObject(node_json, "fw_version", node.fw_version);
            cJSON_AddNumberToObject(node_json, "ota_capable", node.ota_capable);
            if (node.last_ota_result[0] != '\0')
                cJSON_AddStringToObject(node_json, "last_ota_result", node.last_ota_result);
        }
        int n_in = (node.input_count < MAX_INPUTS_PER_NODE) ? node.input_count : MAX_INPUTS_PER_NODE;
        if (n_in > 0) {
            cJSON* input_config = cJSON_CreateArray();
            for (int i = 0; i < n_in; i++) {
                cJSON* entry = cJSON_CreateObject();
                cJSON_AddNumberToObject(entry, "input_index", i);
                cJSON_AddNumberToObject(entry, "input_id", i + 1);
                uint8_t m = (node.input_modes[i] == 0xFF) ? 0 : (node.input_modes[i] & 1);
                cJSON_AddNumberToObject(entry, "mode", m);
                if (node.input_labels[i][0] != '\0')
                    cJSON_AddStringToObject(entry, "label", node.input_labels[i]);
                if (node.input_gpio[i] != 0xFF)
                    cJSON_AddNumberToObject(entry, "gpio", node.input_gpio[i]);
                cJSON_AddBoolToObject(entry, "active_high", node.input_active_high[i] != 0);
                cJSON_AddItemToArray(input_config, entry);
            }
            cJSON_AddItemToObject(node_json, "input_config", input_config);
        }
        cJSON_AddItemToArray(root, node_json);
    }
    xSemaphoreGive(g_nodes_mutex);
    return root;
}

// Function to get nodes as JSON array (for API access)
// Returns union of nodes from config (NVS) and live g_nodes so the list is never missing a node that was saved or recently seen.
static cJSON* get_nodes_json(void) {
    cJSON* root = cJSON_CreateArray();
    if (!root) return nullptr;
    
    std::map<uint8_t, NodeInfo> merged;
    
    // 1) Load nodes from config so we include any node that was saved (e.g. after reconfig) even if not yet seen this boot
    cJSON* config = HSG::API::get_config_json_obj();
    if (config) {
        cJSON* nodes_json = cJSON_GetObjectItem(config, "nodes");
        if (cJSON_IsArray(nodes_json)) {
            cJSON* node_json = NULL;
            cJSON_ArrayForEach(node_json, nodes_json) {
                NodeInfo node;
                cJSON* item = cJSON_GetObjectItem(node_json, "node_id");
                if (!item || !cJSON_IsNumber(item)) continue;
                node.node_id = (uint8_t)item->valueint;
                item = cJSON_GetObjectItem(node_json, "node_type");
                node.node_type = (item && cJSON_IsNumber(item)) ? (uint8_t)item->valueint : 0;
                item = cJSON_GetObjectItem(node_json, "input_count");
                node.input_count = (item && cJSON_IsNumber(item)) ? (uint8_t)item->valueint : 0;
                node.last_seen_timestamp_us = 0;
                item = cJSON_GetObjectItem(node_json, "is_configured");
                node.is_configured = (item && cJSON_IsBool(item)) ? cJSON_IsTrue(item) : (node.node_id != NODE_ID_UNCONFIGURED);
                cJSON* input_config = cJSON_GetObjectItem(node_json, "input_config");
                if (cJSON_IsArray(input_config)) {
                    cJSON* entry = NULL;
                    cJSON_ArrayForEach(entry, input_config) {
                        cJSON* idx = cJSON_GetObjectItem(entry, "input_index");
                        cJSON* mode_item = cJSON_GetObjectItem(entry, "mode");
                        cJSON* label_item = cJSON_GetObjectItem(entry, "label");
                        cJSON* gpio_item = cJSON_GetObjectItem(entry, "gpio");
                        cJSON* active_high_item = cJSON_GetObjectItem(entry, "active_high");
                        if (cJSON_IsNumber(idx) && idx->valueint >= 0 && idx->valueint < MAX_INPUTS_PER_NODE) {
                            if (cJSON_IsNumber(mode_item))
                                node.input_modes[idx->valueint] = (uint8_t)(mode_item->valueint & 1);
                            if (cJSON_IsString(label_item) && label_item->valuestring) {
                                strncpy(node.input_labels[idx->valueint], label_item->valuestring, MAX_INPUT_LABEL_LEN);
                                node.input_labels[idx->valueint][MAX_INPUT_LABEL_LEN] = '\0';
                            }
                            if (cJSON_IsNumber(gpio_item) && ((gpio_item->valueint >= 0 && gpio_item->valueint <= 48) || gpio_item->valueint == 255))
                                node.input_gpio[idx->valueint] = (uint8_t)(gpio_item->valueint & 0xFF);
                            if (cJSON_IsBool(active_high_item))
                                node.input_active_high[idx->valueint] = cJSON_IsTrue(active_high_item) ? 1 : 0;
                            else if (cJSON_IsNumber(active_high_item))
                                node.input_active_high[idx->valueint] = (uint8_t)(active_high_item->valueint & 1);
                        }
                    }
                }
                item = cJSON_GetObjectItem(node_json, "find_me_output_index");
                if (node.node_type == NODE_TYPE_MECHANICAL) {
                    node.find_me_output_index = 0xFF;
                } else if (item && cJSON_IsNumber(item)) {
                    node.find_me_output_index = (uint8_t)(item->valueint & 0xFF);
                }
                item = cJSON_GetObjectItem(node_json, "night_light_on");
                if (item && cJSON_IsBool(item)) node.night_light_on = cJSON_IsTrue(item);
                item = cJSON_GetObjectItem(node_json, "night_light_brightness");
                if (item && cJSON_IsNumber(item)) node.night_light_brightness = (uint8_t)(item->valueint & 0xFF);
                item = cJSON_GetObjectItem(node_json, "ws2812_click_effect");
                if (item && cJSON_IsNumber(item)) node.ws2812_click_effect = (item->valueint != 0) ? 1 : 0;
                load_ws2812_effects_from_json(node, node_json);
                item = cJSON_GetObjectItem(node_json, "fw_version");
                if (item && cJSON_IsNumber(item)) node.fw_version = (uint16_t)item->valueint;
                item = cJSON_GetObjectItem(node_json, "ota_capable");
                if (item && cJSON_IsNumber(item)) node.ota_capable = (uint8_t)(item->valueint & 1);
                item = cJSON_GetObjectItem(node_json, "last_ota_result");
                if (item && cJSON_IsString(item) && item->valuestring)
                    strncpy(node.last_ota_result, item->valuestring, sizeof(node.last_ota_result) - 1);
                merged[node.node_id] = node;
            }
        }
        cJSON_Delete(config);
    }
    
    // 2) Overlay live g_nodes (so recently seen nodes have correct last_seen and status)
    if (g_nodes_mutex && xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
        for (const auto& pair : g_nodes)
            merged[pair.first] = pair.second;
        xSemaphoreGive(g_nodes_mutex);
    }
    
    int64_t current_time_us = esp_timer_get_time();
    const int64_t ONLINE_THRESHOLD_US = 30 * 1000000; // 30 seconds
    const int64_t STALE_UNCONFIGURED_THRESHOLD_US = 60 * 1000000; // 60 seconds
    
    for (const auto& pair : merged) {
        const NodeInfo& node = pair.second;
        int64_t time_since_seen = current_time_us - node.last_seen_timestamp_us;
        
        // Only skip stale unconfigured nodes (127) offline > 60s. Include all configured nodes so the list never hides an active node.
        if (node.node_id == NODE_ID_UNCONFIGURED) {
            if (node.last_seen_timestamp_us > 0 && time_since_seen > STALE_UNCONFIGURED_THRESHOLD_US)
                continue;
        }
        // Configured nodes: always include (no 1-hour filter). UI shows online/offline from last_seen.
        
        cJSON* node_json = cJSON_CreateObject();
        if (!node_json) continue;
        
        cJSON_AddNumberToObject(node_json, "node_id", node.node_id);
        cJSON_AddNumberToObject(node_json, "node_type", node.node_type);
        cJSON_AddNumberToObject(node_json, "input_count", node.input_count);
        cJSON_AddNumberToObject(node_json, "last_seen", (double)node.last_seen_timestamp_us);
        int64_t ago_ms = (time_since_seen > 0) ? (time_since_seen / 1000) : 0;
        cJSON_AddNumberToObject(node_json, "last_seen_ago_ms", (double)ago_ms);
        cJSON_AddBoolToObject(node_json, "is_configured", node.is_configured);
        bool is_online = time_since_seen < ONLINE_THRESHOLD_US;
        cJSON_AddStringToObject(node_json, "status", is_online ? "online" : "offline");
        const char* type_str = (node.node_type == NODE_TYPE_LCD) ? "LCD" : 
                               (node.node_type == NODE_TYPE_MECHANICAL) ? "mechanical" : "unknown";
        cJSON_AddStringToObject(node_json, "type_string", type_str);
        if (node.node_type != NODE_TYPE_MECHANICAL && node.find_me_output_index != 0xFF)
            cJSON_AddNumberToObject(node_json, "find_me_output_index", node.find_me_output_index);
        if (node.node_type == NODE_TYPE_MECHANICAL) {
            cJSON_AddBoolToObject(node_json, "night_light_on", node.night_light_on);
            cJSON_AddNumberToObject(node_json, "night_light_brightness", node.night_light_brightness);
            cJSON_AddNumberToObject(node_json, "ws2812_click_effect", node.ws2812_click_effect);
            add_ws2812_effects_json(node_json, node);
            cJSON_AddNumberToObject(node_json, "fw_version", node.fw_version);
            cJSON_AddNumberToObject(node_json, "ota_capable", node.ota_capable);
            if (node.last_ota_result[0] != '\0')
                cJSON_AddStringToObject(node_json, "last_ota_result", node.last_ota_result);
            char fw_str[16];
            format_fw_version(node.fw_version, fw_str, sizeof(fw_str));
            cJSON_AddStringToObject(node_json, "fw_version_string", fw_str);
        }

        int n_in = (node.input_count < MAX_INPUTS_PER_NODE) ? node.input_count : MAX_INPUTS_PER_NODE;
        if (n_in > 0) {
            cJSON* input_config = cJSON_CreateArray();
            for (int i = 0; i < n_in; i++) {
                cJSON* entry = cJSON_CreateObject();
                cJSON_AddNumberToObject(entry, "input_index", i);
                cJSON_AddNumberToObject(entry, "input_id", i + 1);
                uint8_t m = (node.input_modes[i] == 0xFF) ? 0 : (node.input_modes[i] & 1);
                cJSON_AddNumberToObject(entry, "mode", m);
                if (node.input_labels[i][0] != '\0')
                    cJSON_AddStringToObject(entry, "label", node.input_labels[i]);
                if (node.input_gpio[i] != 0xFF)
                    cJSON_AddNumberToObject(entry, "gpio", node.input_gpio[i]);
                cJSON_AddBoolToObject(entry, "active_high", node.input_active_high[i] != 0);
                cJSON_AddItemToArray(input_config, entry);
            }
            cJSON_AddItemToObject(node_json, "input_config", input_config);
        }
        
        cJSON_AddItemToArray(root, node_json);
    }
    
    return root;
}


/*--------------------------- Main Application Entry Point --------------------*/
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting up...");

    // Initialize NVS (must be first)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    // Initialize Networking
    s_net_event_group = xEventGroupCreate();
  
    initialize_network_interfaces();
    ESP_LOGI(TAG, "Waiting for network connection...");
    xEventGroupWaitBits(s_net_event_group, ETH_CONNECTED_BIT | WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Network connected.");

    // Initialize the core API component and its services
    HSG::API::Init api_init;
    api_init.i2c_port = I2C_MASTER_NUM;
    api_init.output_cb = [](int out, int brightness, int fade_ms, const char* state_str) -> HSG::API::CommandFeedback {
        Command cmd = {TargetType::OUTPUT, out, "", brightness, fade_ms, state_str ? state_str : ""};
        return processCommand(cmd);
    };
    api_init.group_cb = [](const char* name, int brightness, int fade_ms, const char* state_str) -> HSG::API::CommandFeedback {
        Command cmd = {TargetType::GROUP, 0, std::string(name), brightness, fade_ms, state_str ? state_str : ""};
        return processCommand(cmd);
    };
    api_init.get_outputs_json_cb = [](){
        return get_current_outputs_json();
    };
    api_init.get_nodes_json_cb = [](){
        return get_nodes_json();
    };
    api_init.get_nodes_json_for_config_save_cb = [](){
        return get_nodes_json_for_config_save();
    };
    api_init.config_updated_cb = reload_bindings;
    api_init.send_node_config_cb = [](uint8_t target_node_id, uint8_t cmd, const uint8_t* data, size_t len) {
        send_node_config(target_node_id, cmd, data, len);
    };
    api_init.remove_node_cb = [](uint8_t node_id) {
        return remove_node_from_registry(node_id);
    };
    api_init.set_node_input_label_cb = [](uint8_t node_id, uint8_t input_index, const char* label) {
        set_node_input_label_impl(node_id, input_index, label ? label : "");
    };
    api_init.sync_time_to_rtc_cb = []() -> std::string {
        return sync_rtc_from_system_time();
    };
    api_init.get_rtc_time_cb = []() {
        if (!ds3231_is_installed()) return std::string("");
        struct tm rtc_tm;
        char buf[32];
        std::string first_stale;
        for (int attempt = 0; attempt < 3; attempt++) {
            if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(100));
            if (ds3231_get_time(i2c_bus_handle, &rtc_tm) != ESP_OK) break;
            if (strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC", &rtc_tm) <= 0) break;
            int year = rtc_tm.tm_year + 1900;
            if (year >= 2020) {
                g_rtc_cache = buf;
                g_rtc_cache_ticks = xTaskGetTickCount();
                return std::string(buf);
            }
            if (first_stale.empty()) first_stale = buf;
        }
        /* Live read was stale; return recent cache from boot if available so UI shows correct time */
        if (!g_rtc_cache.empty() && (xTaskGetTickCount() - g_rtc_cache_ticks) < g_rtc_cache_max_age_ticks)
            return g_rtc_cache;
        return first_stale.empty() ? std::string("") : first_stale;
    };
    api_init.get_system_time_cb = []() {
        time_t t = time(nullptr);
        if (t < 0) return std::string("");
        struct tm tm;
        if (!gmtime_r(&t, &tm)) return std::string("");
        char buf[32];
        if (strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC", &tm) <= 0) return std::string("");
        return std::string(buf);
    };
    api_init.get_temperature_cb = [](float* celsius) -> bool {
        if (!celsius) return false;
        return mcp9808_read_celsius(i2c_bus_handle, celsius) == ESP_OK;
    };
    api_init.i2c_after_scan_cb = [](const std::vector<uint8_t>& detected) -> HSG::API::I2cScanFollowup {
        HSG::API::I2cScanFollowup result;
        result.mapped_outputs = hsg_outputs_get_mapped_count();
        if (detected.empty()) {
            result.message = "No PCA9685 on bus.";
            return result;
        }
        hsg_pca9685::pca9685_invalidate_bus_devices();
        i2c_bus_recover(i2c_bus_handle);
        hsg_pca_init_stats_t stats = {0, 0};
        esp_err_t err = hsg_outputs_init_chips(detected.data(), detected.size(), 3, &stats);
        result.chips_initialized = stats.ok;
        result.chips_failed = stats.failed;
        if (err == ESP_OK) {
            hsg_outputs_clear_all();
            reset_output_pwm_states();
            result.message = "PCA9685 boards initialized.";
        } else {
            result.message = "PCA init failed for " + std::to_string(stats.failed) + " chip(s).";
        }
        if (result.mapped_outputs == 0)
            result.message += " Warning: no output mappings loaded.";
        return result;
    };
    api_init.i2c_config_applied_cb = [](const std::vector<uint8_t>& applied) {
        cJSON* cfg = HSG::API::get_config_json_obj();
        if (!cfg) return;
        hsg_outputs_reload_config(cfg);
        cJSON_Delete(cfg);
        hsg_pca_init_stats_t stats = {0, 0};
        if (!applied.empty()) {
            hsg_pca9685::pca9685_invalidate_bus_devices();
            i2c_bus_recover(i2c_bus_handle);
            hsg_outputs_init_chips(applied.data(), applied.size(), 3, &stats);
        }
        hsg_outputs_clear_all();
        reset_output_pwm_states();
    };

    ESP_ERROR_CHECK(i2c_master_init());
    i2c_bus_set_post_recover_cb(hsg_pca9685_invalidate_bus_devices);
    ESP_LOGI(TAG, "I2C ready: SDA=%d SCL=%d @%dHz", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    /* Don't monitor IDLE during multi-second I2C init (but keep TWDT initialized). */
    reinit_task_wdt_no_idle();

    /* Let I2C lines settle. Do not bus-reset immediately before probe — that
     * races the ESP-IDF I2C ISR (StoreProhibited in i2c_ll_read_rxfifo). */
    vTaskDelay(pdMS_TO_TICKS(500));

    const bool ds3231_present = ds3231_probe(i2c_bus_handle);
    ds3231_mark_installed(ds3231_present);
    if (ds3231_present) {
        struct tm rtc_first, rtc_second;
        int rtc_reads_consistent = 0;
        if (ds3231_get_time(i2c_bus_handle, &rtc_first) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if (ds3231_get_time(i2c_bus_handle, &rtc_second) == ESP_OK) {
                time_t t1 = mktime(&rtc_first), t2 = mktime(&rtc_second);
                rtc_reads_consistent = (t1 != (time_t)-1 && t2 != (time_t)-1 && (t1 >= t2 ? t1 - t2 : t2 - t1) <= 2);
            }
            uint8_t status = 0;
            ds3231_get_status(i2c_bus_handle, &status);
            int osf_set = (status & 0x80) ? 1 : 0;
            char rtc_buf[32];
            if (strftime(rtc_buf, sizeof(rtc_buf), "%Y-%m-%d %H:%M:%S UTC", &rtc_first) > 0)
                ESP_LOGI(TAG, "RTC time at boot: %s (addr=0x%02X, OSF=%d)", rtc_buf, ds3231_get_i2c_addr(), osf_set);
            setenv("TZ", "UTC0", 1);
            tzset();
            int rtc_year = rtc_first.tm_year + 1900;
            if (rtc_year < 2020)
                ESP_LOGI(TAG, "RTC time stale (year=%d), skipping set system time; use NTP then Sync RTC", rtc_year);
            else if (!rtc_reads_consistent)
                ESP_LOGW(TAG, "RTC read inconsistent, not setting system time");
            else {
                time_t t = mktime(&rtc_first);
                if (t != (time_t)-1) {
                    struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
                    if (settimeofday(&tv, NULL) == 0) {
                        ESP_LOGI(TAG, "System time set from DS3231 RTC");
                        g_rtc_cache = rtc_buf;
                        g_rtc_cache_ticks = xTaskGetTickCount();
                    } else
                        ESP_LOGW(TAG, "settimeofday failed");
                }
            }
        } else
            ESP_LOGW(TAG, "DS3231 read failed");
    } else
        ESP_LOGI(TAG, "DS3231 not found — RTC disabled (no I2C traffic to 0x68/0x69)");

    if (i2c_bus_recover_pending()) {
        esp_err_t bus_rr = i2c_bus_recover(i2c_bus_handle);
        if (bus_rr != ESP_OK) {
            ESP_LOGW(TAG, "I2C bus recover after RTC probe failed: %s", esp_err_to_name(bus_rr));
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    ESP_LOGI(TAG, "Waiting for PCA9685 boards to stabilize...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    esp_err_t i2c_rr = i2c_bus_recover(i2c_bus_handle);
    if (i2c_rr != ESP_OK) {
        ESP_LOGW(TAG, "I2C bus recover before output init failed: %s", esp_err_to_name(i2c_rr));
    }

    ESP_ERROR_CHECK(HSG::API::restore_rack32_pca_if_needed(I2C_MASTER_NUM));

    cJSON* initial_config = HSG::API::get_config_json_obj();
    ESP_ERROR_CHECK(hsg_outputs_init(I2C_MASTER_NUM, initial_config));
    hsg_outputs_clear_all();
    reset_output_pwm_states();
    cJSON_Delete(initial_config);
    ESP_LOGI(TAG, "Outputs ready: %d mapped", hsg_outputs_get_mapped_count());

    i2c_bus_recover(i2c_bus_handle);
    vTaskDelay(pdMS_TO_TICKS(20));

    s_outputs_init_done = true;
    if (s_wifi_failover_pending) {
        ESP_LOGI(TAG, "Starting deferred Wi-Fi failover");
        s_wifi_failover_pending = false;
        wifi_start();
    }

    // MCP9808 probed lazily on first temperature read (avoids boot crash on shared I2C bus).

    // HTTP/MQTT after I2C outputs are initialized so /api/adopt cannot race clear_all.
    HSG::API::start(api_init);
    HSG::API::mqtt_start();

    g_bindings_mutex = xSemaphoreCreateMutex();
    g_nodes_mutex = xSemaphoreCreateMutex();
    g_pending_reconfig_mutex = xSemaphoreCreateMutex();
    g_can_send_mutex = xSemaphoreCreateMutex();
    can_message_queue = xQueueCreate(30, sizeof(can_frame)); // Buffer for 30 CAN frames

    // Load nodes from config
    load_nodes_from_config();

    ESP_LOGI(TAG, "app_main() Initialization complete.");

    reinit_task_wdt_no_idle();

    //xTaskCreate(main_task, "can_gateway", 4096, NULL, 15, NULL); // High priority
    xTaskCreatePinnedToCore(main_task, "can_gateway", 4096, NULL, 15, NULL, 1);
    xTaskCreatePinnedToCore(can_processing_task, "can_logic", 4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(animation_task, "animation_task", 4096, NULL, 5, NULL, 0);

    // Background RTC auto-sync only when a DS3231 is present.
    if (ds3231_is_installed()) {
        xTaskCreatePinnedToCore(rtc_auto_sync_task, "rtc_auto_sync_task", 4096, NULL, 4, NULL, 1);
    }
}

bool initialize_mcp2515_with_retry() {
    const int max_retries = 3;
    
    for (int attempt = 1; attempt <= max_retries; attempt++) {
        ESP_LOGI(TAG, "MCP2515 initialization attempt %d/%d", attempt, max_retries);
        
        // Reset the controller
        if (mcp2515_ptr->reset() != MCP2515::ERROR_OK) {
            ESP_LOGW(TAG, "MCP2515 reset failed on attempt %d", attempt);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (mcp2515_ptr->setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
            ESP_LOGW(TAG, "MCP2515 bitrate setting failed on attempt %d", attempt);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (mcp2515_ptr->setNormalMode() != MCP2515::ERROR_OK) {
            ESP_LOGW(TAG, "MCP2515 normal mode setting failed on attempt %d", attempt);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        } else {
            ESP_LOGI(TAG, "MCP2515 set to Normal Mode successfully");
            return true;
        }  
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    
    ESP_LOGE(TAG, "MCP2515 initialization failed after %d attempts", max_retries);
    return false;
}

// Remove node from registry and persist; returns true if node was present and removed.
static bool remove_node_from_registry(uint8_t node_id) {
    if (node_id < 1 || node_id > 127) return false;
    if (!g_nodes_mutex) return false;
    bool removed = false;
    if (xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
        auto it = g_nodes.find(node_id);
        if (it != g_nodes.end()) {
            g_nodes.erase(it);
            removed = true;
            ESP_LOGI(TAG, "remove_node: removed node %u from registry", (unsigned)node_id);
        }
        xSemaphoreGive(g_nodes_mutex);
        if (removed)
            save_nodes_to_config();
    }
    return removed;
}

// Update node input label in registry, persist, and send CMD_SET_INPUT_LABEL to node (multi-frame if len>5).
// CAN frame is cmd(1) + payload(7) = 8 bytes max, so we send at most 5 chars per frame (idx, total_len|0xFF, 5 chars).
static void set_node_input_label_impl(uint8_t node_id, uint8_t input_index, const char* label) {
    if (input_index >= MAX_INPUTS_PER_NODE) return;
    size_t label_len = label ? strnlen(label, MAX_INPUT_LABEL_LEN + 1) : 0;
    if (label_len > MAX_INPUT_LABEL_LEN) label_len = MAX_INPUT_LABEL_LEN;

    if (g_nodes_mutex && xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
        auto it = g_nodes.find(node_id);
        if (it != g_nodes.end()) {
            memset(it->second.input_labels[input_index], 0, MAX_INPUT_LABEL_LEN + 1);
            if (label_len > 0 && label)
                memcpy(it->second.input_labels[input_index], label, label_len);
        }
        xSemaphoreGive(g_nodes_mutex);
        defer_save_nodes_to_config();
    }

    if (!mcp2515_ptr || !g_can_send_mutex) return;
    const size_t chunk = 5;  // 5 chars per frame (payload 7 bytes: idx, seg, c0..c4)
    uint8_t payload[7];
    for (size_t offset = 0; offset < label_len || offset == 0; offset += chunk) {
        payload[0] = input_index;
        payload[1] = (offset == 0) ? (uint8_t)label_len : (uint8_t)0xFF;
        size_t n = 0;
        for (; n < chunk && offset + n < label_len && label; n++)
            payload[2 + n] = (uint8_t)label[offset + n];
        for (; n < chunk; n++)
            payload[2 + n] = 0;
        send_node_config(node_id, CMD_SET_INPUT_LABEL, payload, sizeof(payload));
        if (label_len == 0) break;
    }
}

// Send NODE_STATE_FEEDBACK to one LCD node (payload: 4 bytes brightness 0-100 or 0xFF per button).
static void send_node_state_feedback(uint8_t node_id, const uint8_t payload[4]) {
    if (!mcp2515_ptr || !g_can_send_mutex) return;
    can_frame frame = {};
    frame.can_id = createCanId(NODE_STATE_FEEDBACK, node_id);
    frame.can_dlc = 4;
    memcpy(frame.data, payload, 4);
    if (xSemaphoreTake(g_can_send_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        mcp2515_ptr->sendMessage(&frame);
        xSemaphoreGive(g_can_send_mutex);
    }
}

// Send SENSOR_DATA (0x2) link keepalive to one node (mechanical nodes use this for CAN link LED).
static void send_sensor_data_link_to_node(uint8_t node_id) {
    if (!mcp2515_ptr || !g_can_send_mutex) return;
    can_frame frame = {};
    frame.can_id = createCanId(SENSOR_DATA, node_id);
    frame.can_dlc = 1;
    frame.data[0] = 0x00;
    if (xSemaphoreTake(g_can_send_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        mcp2515_ptr->sendMessage(&frame);
        xSemaphoreGive(g_can_send_mutex);
    }
}

// Build per-button brightness for one LCD node from bindings and output state; send NODE_STATE_FEEDBACK (throttled).
static void maybe_send_feedback_to_node(uint8_t node_id) {
    static std::map<uint8_t, int64_t> s_last_feedback_us;
    const int64_t throttle_us = 200 * 1000;
    int64_t now = esp_timer_get_time();
    auto it = s_last_feedback_us.find(node_id);
    if (it != s_last_feedback_us.end() && (now - it->second) < throttle_us)
        return;
    s_last_feedback_us[node_id] = now;

    uint8_t payload[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    if (xSemaphoreTake(g_bindings_mutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
    for (int btn = 1; btn <= 4; btn++) {
        for (const auto& rule : g_bindings) {
            if (rule.switchId == (int)node_id && rule.button == btn && rule.onAction == "CLICK" && rule.targetType == TargetType::OUTPUT) {
                int oid = rule.outputId;
                if (oid >= 1 && oid <= MAX_OUTPUTS) {
                    int idx = oid - 1;
                    if (outputs[idx].targetPwmValue == 0)
                        payload[btn - 1] = 0;  // off or fading to off: show 0 so node is not overwritten
                    else
                        payload[btn - 1] = (uint8_t)(outputBrightness[idx] <= 100 ? outputBrightness[idx] : (int)(outputs[idx].currentPwmValue * 100 / 4095));
                }
                break;
            }
        }
    }
    xSemaphoreGive(g_bindings_mutex);
    send_node_state_feedback(node_id, payload);
}

// Get configured timezone string from hub config (for sending to LCD nodes).
static std::string get_hub_timezone_string(void) {
    cJSON* config = HSG::API::get_config_json_obj();
    if (!config) return "UTC";
    cJSON* sys = cJSON_GetObjectItem(config, "system");
    cJSON* tz_item = sys ? cJSON_GetObjectItem(sys, "timezone") : nullptr;
    std::string out = "UTC";
    if (tz_item && cJSON_IsString(tz_item) && tz_item->valuestring && tz_item->valuestring[0])
        out = tz_item->valuestring;
    cJSON_Delete(config);
    return out;
}

// Send CMD_SET_TIMEZONE to one node (multi-frame: 6 chars per frame, same pattern as labels).
static void send_timezone_to_node(uint8_t node_id, const char* tz_str) {
    if (!tz_str) return;
    size_t tz_len = strnlen(tz_str, 41);
    if (tz_len > 40) tz_len = 40;
    const size_t chunk = 6;
    uint8_t payload[7];
    for (size_t offset = 0; offset < tz_len || offset == 0; offset += chunk) {
        payload[0] = (offset == 0) ? (uint8_t)tz_len : (uint8_t)0xFF;
        size_t n = 0;
        for (; n < chunk && offset + n < tz_len; n++)
            payload[1 + n] = (uint8_t)tz_str[offset + n];
        for (; n < chunk; n++)
            payload[1 + n] = 0;
        send_node_config(node_id, CMD_SET_TIMEZONE, payload, sizeof(payload));
        if (tz_len == 0) break;
    }
}

static void format_fw_version(uint16_t ver, char* out, size_t out_len) {
    if (!out || out_len == 0) return;
    snprintf(out, out_len, "%u.%u", (unsigned)(ver >> 8), (unsigned)(ver & 0xFF));
}

static const char* mcp2515_err_str(MCP2515::ERROR err) {
    switch (err) {
        case MCP2515::ERROR_OK: return "OK";
        case MCP2515::ERROR_FAIL: return "FAIL";
        case MCP2515::ERROR_ALLTXBUSY: return "ALLTXBUSY";
        case MCP2515::ERROR_FAILINIT: return "FAILINIT";
        case MCP2515::ERROR_FAILTX: return "FAILTX";
        case MCP2515::ERROR_NOMSG: return "NOMSG";
        default: return "?";
    }
}

static void send_node_config(uint8_t target_node_id, uint8_t cmd, const uint8_t* data, size_t len) {
    if (!mcp2515_ptr) {
        ESP_LOGW(TAG, "send_node_config: CAN not available");
        return;
    }
    if (can_tunnel_is_active()) {
        ESP_LOGD(TAG, "send_node_config: deferred during CAN tunnel (target=%u cmd=0x%02X)",
                 (unsigned)target_node_id, (unsigned)cmd);
        return;
    }
    size_t payload_len = (len > 7) ? 7 : len;
    can_frame frame = {};
    frame.can_id = createCanId(NODE_CONFIG, target_node_id);
    frame.can_dlc = (uint8_t)(1 + payload_len);
    frame.data[0] = cmd;
    if (data && payload_len > 0)
        memcpy(&frame.data[1], data, payload_len);
    if (g_can_send_mutex && xSemaphoreTake(g_can_send_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        MCP2515::ERROR err = mcp2515_ptr->sendMessage(&frame);
        xSemaphoreGive(g_can_send_mutex);
        if (err != MCP2515::ERROR_OK) {
            ESP_LOGE(TAG, "send_node_config: send failed (target=%u cmd=0x%02X)", (unsigned)target_node_id, (unsigned)cmd);
        } else {
            ESP_LOGI(TAG, "send_node_config: sent target=%u cmd=0x%02X", (unsigned)target_node_id, (unsigned)cmd);

            if (cmd == CMD_SET_INPUT_CFG && len >= 3 && data && data[0] < MAX_INPUTS_PER_NODE) {
                if (g_nodes_mutex && xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
                    auto it = g_nodes.find(target_node_id);
                    if (it != g_nodes.end()) {
                        it->second.input_modes[data[0]] = data[2] & 1;
                        if (len >= 4)
                            it->second.input_gpio[data[0]] = data[3];
                        if (len >= 5)
                            it->second.input_active_high[data[0]] = data[4] & 1;
                        ESP_LOGI(TAG, "input_modes[%u]=%u for node %u", (unsigned)data[0], (unsigned)(data[2] & 1), (unsigned)target_node_id);
                    }
                    xSemaphoreGive(g_nodes_mutex);
                    defer_save_nodes_to_config();
                }
            }

            if (cmd == CMD_SET_INPUT_COUNT && len >= 1 && data) {
                uint8_t count = data[0];
                if (count >= 1 && count <= MAX_INPUTS_PER_NODE) {
                    if (g_nodes_mutex && xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
                        auto it = g_nodes.find(target_node_id);
                        if (it != g_nodes.end()) {
                            it->second.input_count = count;
                            ESP_LOGI(TAG, "input_count=%u for node %u", (unsigned)count, (unsigned)target_node_id);
                        }
                        xSemaphoreGive(g_nodes_mutex);
                        save_nodes_to_config();
                    }
                }
            }

            if (cmd == CMD_SET_FIND_ME_OUTPUT && len >= 1 && data) {
                if (g_nodes_mutex && xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
                    auto it = g_nodes.find(target_node_id);
                    if (it != g_nodes.end() && it->second.node_type != NODE_TYPE_MECHANICAL) {
                        it->second.find_me_output_index = data[0];
                        ESP_LOGI(TAG, "find_me_output_index=%u for node %u", (unsigned)data[0], (unsigned)target_node_id);
                        xSemaphoreGive(g_nodes_mutex);
                        save_nodes_to_config();
                    } else {
                        if (it != g_nodes.end())
                            ESP_LOGI(TAG, "find_me_output_index ignored for mechanical node %u (WS2812 strip)", (unsigned)target_node_id);
                        xSemaphoreGive(g_nodes_mutex);
                    }
                }
            }
            if (cmd == CMD_SET_NIGHT_LIGHT && len >= 2 && data) {
                uint8_t enabled = data[0] ? 1 : 0;
                uint8_t brightness = data[1];
                if (brightness > 100) brightness = 100;
                if (g_nodes_mutex && xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
                    auto it = g_nodes.find(target_node_id);
                    if (it != g_nodes.end()) {
                        it->second.night_light_on = (enabled != 0);
                        it->second.night_light_brightness = brightness;
                        ESP_LOGI(TAG, "night_light %s brightness=%u for node %u", enabled ? "on" : "off", (unsigned)brightness, (unsigned)target_node_id);
                    }
                    xSemaphoreGive(g_nodes_mutex);
                    save_nodes_to_config();
                }
            }
            if (cmd == CMD_SET_WS2812_CLICK_EFFECT && len >= 1 && data) {
                uint8_t effect = data[0] ? 1 : 0;
                if (g_nodes_mutex && xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
                    auto it = g_nodes.find(target_node_id);
                    if (it != g_nodes.end()) {
                        it->second.ws2812_click_effect = effect;
                        ESP_LOGI(TAG, "ws2812_click_effect=%u for node %u", (unsigned)effect, (unsigned)target_node_id);
                    }
                    xSemaphoreGive(g_nodes_mutex);
                    save_nodes_to_config();
                }
            }
            if (cmd == CMD_SET_WS2812_EFFECT_PARAMS && len >= 6 && data && data[0] <= 2) {
                uint8_t effect_id = data[0];
                uint8_t r = data[1];
                uint8_t g = data[2];
                uint8_t b = data[3];
                uint16_t timing_ms = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
                if (timing_ms < 10) timing_ms = 10;
                if (timing_ms > 2000) timing_ms = 2000;
                Ws2812EffectParams params = {r, g, b, timing_ms};
                if (g_nodes_mutex && xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
                    auto it = g_nodes.find(target_node_id);
                    if (it != g_nodes.end()) {
                        if (effect_id == 0) it->second.ws2812_strobe = params;
                        else if (effect_id == 1) it->second.ws2812_chase = params;
                        else it->second.ws2812_find_me = params;
                        ESP_LOGI(TAG, "ws2812 effect params id=%u rgb=%u,%u,%u timing=%u for node %u",
                                 (unsigned)effect_id, (unsigned)r, (unsigned)g, (unsigned)b,
                                 (unsigned)timing_ms, (unsigned)target_node_id);
                    }
                    xSemaphoreGive(g_nodes_mutex);
                    save_nodes_to_config();
                }
            }

            // Record pending reconfig: remove old_id only after we receive HEARTBEAT with new_id
            if (cmd == CMD_SET_NODE_ID && len >= 1) {
                uint8_t new_id = data[0];
                if (new_id >= 1 && new_id <= 126 && new_id != target_node_id) {
                    if (g_pending_reconfig_mutex && xSemaphoreTake(g_pending_reconfig_mutex, portMAX_DELAY) == pdTRUE) {
                        g_pending_reconfig_old_id[new_id] = target_node_id;
                        ESP_LOGI(TAG, "Pending reconfig: will remove node %u when HEARTBEAT from node %u is received", (unsigned)target_node_id, (unsigned)new_id);
                        xSemaphoreGive(g_pending_reconfig_mutex);
                    }
                }
            }
        }
    }
}

void can_processing_task(void *pvParameter) {
    ESP_LOGI(TAG, "CAN Processing Task started.");
    
    // Load the bindings once at the start
    reload_bindings();
    vTaskDelay(pdMS_TO_TICKS(100));

    can_frame frame;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (1) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());

        if (can_tunnel_is_active()) {
            if (xQueueReceive(can_message_queue, &frame, pdMS_TO_TICKS(50))) {
                /* process below */
            } else {
                continue;
            }
        } else if (xQueueReceive(can_message_queue, &frame, pdMS_TO_TICKS(5000))) {
            /* process below */
        } else {
            continue;
        }

        {
            // Protocol guard: this project only supports standard 11-bit data frames.
            if (!isStandardDataFrame(frame.can_id)) {
                ESP_LOGW(TAG, "Ignoring non-standard CAN frame (can_id=0x%lX)", frame.can_id);
                continue;
            }

            const uint16_t raw_sid = getStandardId(frame.can_id);
            CanMessageType msgType = resolveMessageType(raw_sid, frame.data, frame.can_dlc);
            const uint16_t sid = canonicalSid(raw_sid, msgType);
            uint8_t msgTypeValue = (uint8_t)msgType;

            HSG_CanFrame api_frame;
            api_frame.id = sid;
            api_frame.dlc = frame.can_dlc;
            memcpy(api_frame.data, frame.data, frame.can_dlc);
            HSG::API::add_to_can_history(api_frame);

            if (msgType == LIGHTING_COMMAND) {
                if (frame.can_dlc >= 2) {
                    int switchId = getNodeId(sid);
                    int button = frame.data[0];

                    uint8_t evt = frame.data[1];
                    const char* action = decodeEventAction(evt);
                    bool matchFound = false;
                    Command cmd;

                    // EVT_DIM: payload byte 2 = brightness 0..100; use same binding as CLICK for (node_id, button)
                    if (evt == EVT_DIM && frame.can_dlc >= 3) {
                        uint8_t brightness = frame.data[2];
                        if (brightness > 100) brightness = 100;
                        if (xSemaphoreTake(g_bindings_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                            for (const auto& rule : g_bindings) {
                                if (rule.switchId == switchId && rule.button == button && strcmp(rule.onAction.c_str(), "CLICK") == 0) {
                                    ESP_LOGI(TAG, "CAN DIM Match: Switch %d, Button %d, brightness=%u", switchId, button, (unsigned)brightness);
                                    matchFound = true;
                                    cmd.type = rule.targetType;
                                    cmd.output_id = rule.outputId;
                                    cmd.group_name = rule.groupName;
                                    cmd.fade_ms = rule.fade_ms;
                                    cmd.state = (brightness == 0) ? "OFF" : "ON";
                                    cmd.brightness = (int)brightness;
                                    break;
                                }
                            }
                            xSemaphoreGive(g_bindings_mutex);
                        }
                        if (matchFound) {
                            processCommand(cmd);
                        } else {
                            ESP_LOGW(TAG, "EVT_DIM: no CLICK binding for Switch %d Button %d", switchId, button);
                        }
                    } else {
                    if (xSemaphoreTake(g_bindings_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                        for (const auto& rule : g_bindings) {
                            if (rule.switchId == switchId && rule.button == button && strcmp(rule.onAction.c_str(), action) == 0) {
                                ESP_LOGI(TAG, "CAN Match Found: Switch %d, Button %d", switchId, button);
                                matchFound = true;

                                cmd.type = rule.targetType;
                                cmd.output_id = rule.outputId;
                                cmd.group_name = rule.groupName;
                                cmd.fade_ms = rule.fade_ms;
                                cmd.state = rule.state;
                                cmd.brightness = rule.brightness;
                                
                                break; 
                            }
                        }
                        xSemaphoreGive(g_bindings_mutex);
                    }    
                    if (matchFound) {
                        processCommand(cmd);
                    } else {
                        ESP_LOGW(TAG, "Received a valid lighting command for Switch ID %d (Button: %d), but NO matching binding was found in the configuration.", switchId, button);
                    }
                    }
                }
            } else if (msgType == HEARTBEAT || msgTypeValue == 0x08) {
                uint8_t nodeId = getNodeId(sid);
                int64_t current_time_us = esp_timer_get_time();
                ESP_LOGD(TAG, "HEARTBEAT received: nodeId=%u, DLC=%u, data[0]=0x%02X, data[1]=0x%02X, msgTypeValue=0x%02X", 
                         (unsigned)nodeId, (unsigned)frame.can_dlc, frame.data[0], frame.data[1], msgTypeValue);
                
                if (frame.can_dlc >= 2) {
                    uint8_t node_type = frame.data[0];
                    uint8_t input_count = frame.data[1];
                    const char* type_str = (node_type == NODE_TYPE_LCD) ? "LCD" : (node_type == NODE_TYPE_MECHANICAL) ? "mechanical" : "unknown";
                    ESP_LOGD(TAG, "HEARTBEAT processing: nodeId=%u, type=%s (%u), input_count=%u", 
                             (unsigned)nodeId, type_str, (unsigned)node_type, (unsigned)input_count);
                    
                    // Update node registry
                    bool removed_stale = false;
                    bool is_new = false;
                    bool lcd_just_came_online = false;  // true if LCD node was offline and we should push time
                    if (g_nodes_mutex && xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
                        is_new = (g_nodes.find(nodeId) == g_nodes.end());
                        NodeInfo& node = g_nodes[nodeId];
                        int64_t prev_seen = node.last_seen_timestamp_us;
                        node.node_id = nodeId;
                        node.node_type = node_type;
                        node.input_count = input_count;
                        node.last_seen_timestamp_us = current_time_us;
                        if (node_type == NODE_TYPE_LCD && nodeId != NODE_ID_UNCONFIGURED &&
                            (is_new || prev_seen == 0 || (current_time_us - prev_seen) > 30000000 ||
                             current_time_us < 120000000))  // 120 s: always push time for first 2 min after hub boot
                            lcd_just_came_online = true;
                        node.is_configured = (nodeId != NODE_ID_UNCONFIGURED);
                        if (node_type == NODE_TYPE_MECHANICAL)
                            node.find_me_output_index = 0xFF;
                        if (frame.can_dlc >= 5) {
                            node.fw_version = (uint16_t)frame.data[2] | ((uint16_t)frame.data[3] << 8);
                            node.ota_capable = frame.data[4] ? 1 : 0;
                        }
                        
                        if (nodeId == NODE_ID_UNCONFIGURED) {
                            if (is_new) {
                                ESP_LOGI(TAG, "New unconfigured node on bus: type=%s (%u), input_count=%u (assign node ID via web)", type_str, (unsigned)node_type, (unsigned)input_count);
                            }
                        } else {
                            // Configured node (not 127)
                            if (is_new) {
                                ESP_LOGI(TAG, "New configured node discovered: node_id=%u type=%s input_count=%u", (unsigned)nodeId, type_str, (unsigned)input_count);
                            } else {
                                ESP_LOGD(TAG, "Node announce: node_id=%u type=%s input_count=%u", (unsigned)nodeId, type_str, (unsigned)input_count);
                            }
                            
                            // ALWAYS check for stale unconfigured entry when receiving HEARTBEAT from configured node
                            auto unconf_it = g_nodes.find(NODE_ID_UNCONFIGURED);
                            if (unconf_it != g_nodes.end()) {
                                const NodeInfo& unconf_node = unconf_it->second;
                                // Remove if type/count matches (likely same physical node)
                                if (unconf_node.node_type == node_type && 
                                    unconf_node.input_count == input_count) {
                                    ESP_LOGI(TAG, "Removing stale unconfigured node entry (same as configured node %u)", (unsigned)nodeId);
                                    g_nodes.erase(unconf_it);
                                    removed_stale = true;
                                }
                            }
                        }
                        
                        xSemaphoreGive(g_nodes_mutex);
                        
                        if (lcd_just_came_online) {
                            std::string tz = get_hub_timezone_string();
                            send_timezone_to_node(nodeId, tz.c_str());
                            time_t t = time(nullptr);
                            uint32_t ts = (uint32_t)t;
                            uint8_t datetime_payload[4] = {
                                (uint8_t)(ts & 0xFF),
                                (uint8_t)((ts >> 8) & 0xFF),
                                (uint8_t)((ts >> 16) & 0xFF),
                                (uint8_t)((ts >> 24) & 0xFF)
                            };
                            send_node_config(nodeId, CMD_SET_DATETIME, datetime_payload, sizeof(datetime_payload));
                        }
                        
                        // If this HEARTBEAT is from a node we just reconfigured (old_id -> nodeId), remove old_id permanently (outside g_nodes_mutex to avoid deadlock)
                        if (nodeId != NODE_ID_UNCONFIGURED && g_pending_reconfig_mutex && xSemaphoreTake(g_pending_reconfig_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                            auto pit = g_pending_reconfig_old_id.find(nodeId);
                            if (pit != g_pending_reconfig_old_id.end()) {
                                uint8_t old_id = pit->second;
                                g_pending_reconfig_old_id.erase(pit);
                                xSemaphoreGive(g_pending_reconfig_mutex);
                                if (g_nodes_mutex && xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
                                    auto oit = g_nodes.find(old_id);
                                    if (oit != g_nodes.end()) {
                                        ESP_LOGI(TAG, "Removing old node ID %u permanently (node now sending as %u)", (unsigned)old_id, (unsigned)nodeId);
                                        g_nodes.erase(oit);
                                        defer_save_nodes_to_config();
                                    }
                                    xSemaphoreGive(g_nodes_mutex);
                                }
                            } else {
                                xSemaphoreGive(g_pending_reconfig_mutex);
                            }
                        }
                        
                        // Save to config (throttle: only save if it's a new node, stale entry was removed, or every 10 seconds)
                        static int64_t last_save_time_us = 0;
                        if (is_new || removed_stale || (current_time_us - last_save_time_us) > 10000000) { // 10 seconds
                            defer_save_nodes_to_config();
                            last_save_time_us = current_time_us;
                        }
                    }
                } else {
                    // Minimal heartbeat without type/input_count - just update timestamp
                    if (g_nodes_mutex && xSemaphoreTake(g_nodes_mutex, portMAX_DELAY) == pdTRUE) {
                        auto it = g_nodes.find(nodeId);
                        if (it != g_nodes.end()) {
                            it->second.last_seen_timestamp_us = current_time_us;
                        }
                        xSemaphoreGive(g_nodes_mutex);
                    }
                    ESP_LOGD(TAG, "Heartbeat from node 0x%02X", nodeId);
                }
            } else if (isCanOtaNodeMessage(msgType)) {
                uint8_t nodeId = getNodeId(sid);
                if (frame.can_dlc >= 3 && g_nodes_mutex
                    && xSemaphoreTake(g_nodes_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    auto it = g_nodes.find(nodeId);
                    if (it != g_nodes.end()) {
                        const uint8_t opcode = frame.data[2];
                        if (opcode == OTA_FLASH_COMPLETE)
                            strncpy(it->second.last_ota_result, "success", sizeof(it->second.last_ota_result) - 1);
                        else if (opcode == OTA_FLASH_ERROR)
                            strncpy(it->second.last_ota_result, "failed", sizeof(it->second.last_ota_result) - 1);
                        it->second.last_ota_result[sizeof(it->second.last_ota_result) - 1] = '\0';
                        if (opcode == OTA_FLASH_COMPLETE || opcode == OTA_FLASH_ERROR)
                            defer_save_nodes_to_config();
                    }
                    xSemaphoreGive(g_nodes_mutex);
                }
            } else {
                ESP_LOGW(TAG, "Unknown message type: 0x%02X (CAN ID: 0x%03X, SID: 0x%03X)", 
                         (unsigned)msgType, (unsigned)frame.can_id, sid);
            }
        }
    }
}

void test_can_hardware() {
    ESP_LOGI(TAG, "=== CAN Hardware Diagnostic ===");

    // Test SPI communication (use internal tx_data so transaction descriptor is valid for driver)
    spi_transaction_t t = {};
    t.length = 8;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = 0xC0;  // Reset command

    esp_err_t spi_ret = spi_device_transmit(can_spi_handle, &t);
    ESP_LOGI(TAG, "SPI transmit test: %s", esp_err_to_name(spi_ret));
    
    // Test GPIO levels
    ESP_LOGI(TAG, "CAN_CS level: %d", gpio_get_level((gpio_num_t)CAN_CS_GPIO));
    ESP_LOGI(TAG, "CAN_INT level: %d", gpio_get_level((gpio_num_t)CAN_INT_GPIO));
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Try to reset and reconfigure MCP2515
    if (mcp2515_ptr->reset() == MCP2515::ERROR_OK) {
        ESP_LOGI(TAG, "MCP2515 reset successful");
        if (mcp2515_ptr->setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
            ESP_LOGI(TAG, "MCP2515 bitrate set successfully");
            if (mcp2515_ptr->setNormalMode() == MCP2515::ERROR_OK) {
                ESP_LOGI(TAG, "MCP2515 normal mode set successfully");
            }
        }
    }
    
    ESP_LOGI(TAG, "=== End Diagnostic ===");
}

/*--------------------------- Main Application Task -------------------------*/
void main_task(void *pvParameter)
{
    ESP_LOGI(TAG, "CAN Gateway Task started.");
    
     if (can_module_init() != ESP_OK) {
        ESP_LOGE(TAG, "CAN module initialization failed. Halting task.");
        vTaskDelete(NULL);
    }
    mcp2515_ptr = new MCP2515(&can_spi_handle);
    
    bool can_available = initialize_mcp2515_with_retry();
    
    if (!can_available) {
        ESP_LOGW(TAG, "MCP2515 not available, running without CAN support.");
    } else {
        can_tunnel_set_hw(mcp2515_ptr, g_can_send_mutex);
        can_tunnel_set_hub_rx_queue(can_message_queue);
        can_tunnel_register_http(HSG::API::http_server());
    }

    // GPIO interrupt setup (keep your existing code)
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << CAN_INT_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(30, sizeof(uint32_t));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)CAN_INT_GPIO, gpio_isr_handler, (void*)CAN_INT_GPIO));
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    // Let SPI/CAN settle before first transaction to avoid spi_master assert on early use
    if (can_available)
        vTaskDelay(pdMS_TO_TICKS(500));
    while (1) {
        // "Pet the dog" at the start of every loop
        ESP_ERROR_CHECK(esp_task_wdt_reset());

        if (can_available) {
            if (can_tunnel_is_active()) {
                vTaskDelay(pdMS_TO_TICKS(10));
            } else {
            const TickType_t irq_wait = pdMS_TO_TICKS(1000);
            uint32_t io_num;
            if (xQueueReceive(gpio_evt_queue, &io_num, irq_wait) == pdTRUE) {
                if (g_can_send_mutex && xSemaphoreTake(g_can_send_mutex, pdMS_TO_TICKS(200)) == pdTRUE) {
                    int messages_processed = 0;
                    can_frame received_frame;
                    do {
                        if (mcp2515_ptr->readMessage(&received_frame) == MCP2515::ERROR_OK) {
                            messages_processed++;
                            if (isStandardDataFrame(received_frame.can_id)) {
                                if (xQueueSend(can_message_queue, &received_frame, 0) != pdPASS) {
                                    ESP_LOGW(TAG, "CAN processing queue full! Dropped message ID: 0x%lX", received_frame.can_id);
                                }
                            }
                        } else {
                            break;
                        }
                        if (messages_processed >= 32)
                            break;
                    } while (true);
                    if (mcp2515_ptr->checkError()) {
                        static int64_t s_last_can_err_log_us = 0;
                        static uint8_t s_last_can_err_flags = 0;
                        uint8_t err_flags = mcp2515_ptr->getErrorFlags();
                        mcp2515_ptr->clearRXnOVRFlags();
                        mcp2515_ptr->clearInterrupts();
                        int64_t now_us = esp_timer_get_time();
                        if (err_flags != s_last_can_err_flags
                            || (now_us - s_last_can_err_log_us) > 5000000LL) {
                            ESP_LOGW(TAG, "CAN error (flags: 0x%02X), cleared", err_flags);
                            s_last_can_err_flags = err_flags;
                            s_last_can_err_log_us = now_us;
                        }
                    }
                    xSemaphoreGive(g_can_send_mutex);
                }
            }
            }
        }else {
            // If CAN is not available, just delay to avoid a tight loop
            // Just idle so other tasks (MQTT/HTTP/animation) keep running
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void animation_task(void *pvParameter)
{
    uint32_t last_update_time = 0;
    const uint32_t update_interval_ms = 100; // Send updates to UI 10 times/sec
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    while (1) {
        ESP_ERROR_CHECK(esp_task_wdt_reset());
        processFades();

        uint32_t now = esp_log_timestamp();
        if (now - last_update_time >= update_interval_ms) {
            last_update_time = now;

            // --- NEW: Build and send a single state update ---
            cJSON* root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "type", "state");

            // Create an object to hold the brightness of all "ON" outputs
            cJSON* outputs_json = cJSON_CreateObject();
            for (int i = 0; i < MAX_OUTPUTS; i++) {
                // Only include outputs that have a non-zero brightness
                if (outputs[i].currentPwmValue != 0 || outputs[i].targetPwmValue != 0) {
                    char key[5];
                    snprintf(key, sizeof(key), "%d", i + 1); // Key is the logical output ID
                    // We send the raw PWM value
                    cJSON_AddNumberToObject(outputs_json, key, outputs[i].currentPwmValue);
                }
            }
            cJSON_AddItemToObject(root, "outputs", outputs_json);

            char* json_str = cJSON_PrintUnformatted(root);
            if (json_str) {
                HSG::API::send_ws_message(std::string(json_str));
                free(json_str);
            }
            cJSON_Delete(root);

            // Defer all CAN (SPI) sends for a few seconds after boot to avoid spi_master assert on first transactions.
            const uint32_t CAN_SEND_DEFER_MS = 5000;
            if (now >= CAN_SEND_DEFER_MS && g_nodes_mutex
                && xSemaphoreTake(g_nodes_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                std::vector<uint8_t> lcd_nodes;
                std::vector<uint8_t> mechanical_nodes;
                for (const auto& pair : g_nodes) {
                    if (pair.second.node_type == NODE_TYPE_LCD && pair.second.node_id != NODE_ID_UNCONFIGURED)
                        lcd_nodes.push_back(pair.second.node_id);
                    if (pair.second.node_type == NODE_TYPE_MECHANICAL)
                        mechanical_nodes.push_back(pair.second.node_id);
                }
                xSemaphoreGive(g_nodes_mutex);

                if (!can_tunnel_is_active()) {
                    for (uint8_t nid : lcd_nodes)
                        maybe_send_feedback_to_node(nid);
                }

                // Send SENSOR_DATA (0x2) link keepalive to mechanical nodes every 7 s so their CAN link LED matches hub online state.
                // Keep this running during CAN tunnel so nodes stay visible on the bus.
                static uint32_t last_mechanical_link_ms = 0;
                const uint32_t MECHANICAL_LINK_INTERVAL_MS = 7000;
                if (now - last_mechanical_link_ms >= MECHANICAL_LINK_INTERVAL_MS && !mechanical_nodes.empty()) {
                    for (uint8_t nid : mechanical_nodes) {
                        send_sensor_data_link_to_node(nid);
                        vTaskDelay(pdMS_TO_TICKS(5));
                    }
                    last_mechanical_link_ms = now;
                }

                if (!can_tunnel_is_active()) {
                // Send date/time to LCD nodes: every 2s for first 60s (so node gets time soon after boot), then every hour
                static uint32_t datetime_send_counter = 0;
                const uint32_t loops_per_second = 1000 / 16;  // ~62
                const uint32_t loops_per_2s = 2 * loops_per_second;
                const uint32_t loops_60s = 60 * loops_per_second;
                datetime_send_counter++;
                bool send_now = false;
                if (datetime_send_counter < loops_60s && (datetime_send_counter % loops_per_2s) == 0)
                    send_now = true;
                if (datetime_send_counter >= 36000) {  // ~every hour
                    datetime_send_counter = 0;
                    send_now = true;
                }
                if (send_now && !lcd_nodes.empty()) {
                    std::string tz = get_hub_timezone_string();
                    time_t t = time(nullptr);
                    uint32_t ts = (uint32_t)t;
                    uint8_t datetime_payload[4] = {
                        (uint8_t)(ts & 0xFF),
                        (uint8_t)((ts >> 8) & 0xFF),
                        (uint8_t)((ts >> 16) & 0xFF),
                        (uint8_t)((ts >> 24) & 0xFF)
                    };
                    for (uint8_t nid : lcd_nodes) {
                        send_timezone_to_node(nid, tz.c_str());
                        send_node_config(nid, CMD_SET_DATETIME, datetime_payload, sizeof(datetime_payload));
                    }
                }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(16)); // ~60Hz loop for smooth fades
    }
}


static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, nullptr);
}

static esp_err_t i2c_master_init(void) {
    i2c_bus_lock_init();
    i2c_master_bus_config_t i2c_mst_config = {};
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_MASTER_NUM;
    i2c_mst_config.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    i2c_mst_config.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    i2c_mst_config.glitch_ignore_cnt = 7;
    //i2c_mst_config.glitch_filter_ns = -1;
    i2c_mst_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_bus_handle));
    return ESP_OK;
}


