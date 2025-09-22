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
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
//#include "driver/i2c.h"
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
#include "esp_eth_phy.h"
#include "esp_eth_mac.h"
#include "mqtt_client.h"

#include "HSG-API.h"
#include "hsg_outputs.h"
#include "hsg_pca9685.h"
#include "can_protocol.h"

static const char *TAG = "MAIN";

/*--------------------------- Constants ----------------------------------*/
#define MAX_OUTPUTS 160
#define DEFAULT_FADE_MS 1000

// --- Global State for Network Failover ---
static EventGroupHandle_t s_net_event_group;
static const int ETH_CONNECTED_BIT = BIT0;
static const int WIFI_CONNECTED_BIT = BIT1;
static bool s_eth_connected = false;
static bool s_wifi_connected = false;
static esp_mqtt_client_handle_t mqtt_client;
i2c_master_bus_handle_t i2c_bus_handle;
SemaphoreHandle_t i2c_mutex;

/* ---------------------- Global State for Commands and Fading ---------------------- */
enum class TargetType { OUTPUT, GROUP };

struct Binding {
    // The Trigger (what button was pressed)
    int switchId;
    int button;
    std::string onAction; // e.g., "CLICK", "HOLD"

    // The Action (what to do)
    TargetType targetType; // OUTPUT or GROUP
    int outputId;
    std::string groupName;
    std::string state; // "ON", "OFF", "TOGGLE"
    int brightness;
    int fade_ms;
};
// A vector to hold all the rules loaded from config
static std::vector<Binding> g_bindings;

struct Command {
    TargetType type = TargetType::OUTPUT;
    int output_id = 0;
    std::string group_name;
    int brightness = 0;
    int fade_ms = 0;
};

struct OutputState {
    int startPwmValue = 0;
    int currentPwmValue = 0;
    int targetPwmValue = 0;
    unsigned long fadeStartTime = 0;
    unsigned long fadeDuration = DEFAULT_FADE_MS;
};
OutputState outputs[MAX_OUTPUTS];
int outputBrightness[MAX_OUTPUTS] = {0}; // Last "ON" brightness (0-100)

// MQTT client handle
//static esp_mqtt_client_handle_t mqtt_client;


// --- Olimex ESP32-POE wiring ---
#define ETH_MDC_GPIO               23
#define ETH_MDIO_GPIO              18
#define ETH_PHY_ADDR               0
#define ETH_RST_GPIO               -1
#define ETH_PHY_PWR_GPIO           12
#define ETH_PHY_POWER_ACTIVE_HIGH  1

/* ---------------------- I2C Configuration ---------------------- */
#define I2C_MASTER_SCL_IO   16
#define I2C_MASTER_SDA_IO   13
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  400000


/* ---------------------- SPI/MCP2515 Configuration ---------------------- */
#define SPI_HOST SPI2_HOST
#define PIN_NUM_MISO 33
#define PIN_NUM_MOSI 32
#define PIN_NUM_CLK  4
#define PIN_NUM_CS   5
#define PIN_NUM_INT  35

static QueueHandle_t gpio_evt_queue = nullptr;
static spi_device_handle_t mcp_spi_handle;
static can_frame g_last_frame = {};
static SemaphoreHandle_t g_last_mutex;

static esp_netif_t *s_eth_netif = NULL;
static esp_netif_t *s_wifi_netif = NULL;

/*--------------------------- Function Prototypes ---------------------------*/
static esp_err_t i2c_master_init(void);
void main_task(void *pvParameter);
void animation_task(void *pvParameter);
static void IRAM_ATTR gpio_isr_handler(void* arg);
//static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
//static void ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void wifi_start(void);
static void eth_start(void);
void setOutput(int output, int brightness, int fadeMs);


void processFades();

static void phy_power_set(bool on) {
    static bool inited = false;
    if (!inited) {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << ETH_PHY_PWR_GPIO;
        io.mode = GPIO_MODE_OUTPUT;
        gpio_config(&io);
        inited = true;
    }
    gpio_set_level((gpio_num_t)ETH_PHY_PWR_GPIO, on ? 1 : 0);
}

/* ---------------------- Network Initialization ---------------------- */
// --- Unified Network Event Handler with Failback Logic ---
static void net_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    // --- Ethernet Events ---
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
            // Ethernet lost, start Wi-Fi as a backup
            wifi_start();
        }
    }

    // --- Wi-Fi Events ---
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

    // --- IP Address Events ---
    if (event_base == IP_EVENT) {
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
    }
}



// --- Ethernet Start Function ---
static void eth_start(void)
{
    // Create Ethernet network interface
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *eth_netif = esp_netif_new(&cfg);

    s_eth_netif = eth_netif;

     // Power up PHY first (GPIO12)
    phy_power_set(true);
    vTaskDelay(pdMS_TO_TICKS(50));  // let 3V3 & XO settle

    // Configure Ethernet MAC and PHY
    eth_esp32_emac_config_t esp32_emac_config = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_config.smi_mdc_gpio_num = ETH_MDC_GPIO;
    esp32_emac_config.smi_mdio_gpio_num = ETH_MDIO_GPIO;

    // FIX: Create the generic MAC config
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();

    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = ETH_PHY_ADDR;
    phy_config.reset_gpio_num = ETH_RST_GPIO;

    // Power on the PHY
    gpio_set_direction((gpio_num_t)ETH_PHY_PWR_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)ETH_PHY_PWR_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Install Ethernet driver
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_config, &mac_config);

    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_config);
    esp_eth_config_t config = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = NULL;
    ESP_ERROR_CHECK(esp_eth_driver_install(&config, &eth_handle));

    // Attach Ethernet driver to TCP/IP stack
    // Create the glue layer between the MAC and the TCP/IP stack
    void *glue = esp_eth_new_netif_glue(eth_handle);

    // Attach the Ethernet driver to the TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, glue));
    
    
    // Start Ethernet driver state machine
    ESP_ERROR_CHECK(esp_eth_start(eth_handle));
}

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

/* ---------------------- Hardware & Network Init ---------------------- */
static esp_err_t spi_master_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1, // Not used
        .quadhd_io_num = -1, // Not used
        .max_transfer_sz = 32
    };

    spi_device_interface_config_t devcfg = {
        .mode = 0,                         // SPI mode 0
        .clock_speed_hz = 10 * 1000 * 1000, // Clock speed is 10 MHz
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,                   // We wish to be able to queue 7 transactions at a time
    };

    //Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    //Attach the MCP2515 to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &mcp_spi_handle));

    ESP_LOGI(TAG, "SPI master initialized successfully.");
    return ESP_OK;
}


/*--------------------------- Fading and PWM Logic ---------------------------*/
void processCommand(const Command& cmd) {
    ESP_LOGI(TAG, "Processing command for %s: %s%d, brightness=%d, fade=%dms",
             (cmd.type == TargetType::OUTPUT) ? "output" : "group",
             (cmd.type == TargetType::OUTPUT) ? "" : cmd.group_name.c_str(),
             cmd.output_id,
             cmd.brightness,
             cmd.fade_ms);

    if (cmd.type == TargetType::OUTPUT) {
        setOutput(cmd.output_id, cmd.brightness, cmd.fade_ms);
    } else if (cmd.type == TargetType::GROUP) {
        cJSON* config = HSG::API::get_config_json_obj();
        if (!config) {
            ESP_LOGE(TAG, "Failed to get config to process group command.");
            return;
        }

        // 2. Find the "groups" object and the specific group name
        cJSON* groups = cJSON_GetObjectItem(config, "groups");
        cJSON* group_outputs = cJSON_GetObjectItem(groups, cmd.group_name.c_str());

        // 3. If the group is found, iterate through its outputs
        if (cJSON_IsArray(group_outputs)) {
            cJSON* output_item = NULL;
            cJSON_ArrayForEach(output_item, group_outputs) {
                if (cJSON_IsNumber(output_item)) {
                    // For each output in the group, call setOutput()
                    int output_id = output_item->valueint;
                    setOutput(output_id, cmd.brightness, cmd.fade_ms);
                }
            }
        } else {
            ESP_LOGW(TAG, "Group '%s' not found in configuration.", cmd.group_name.c_str());
        }

        // 4. Clean up the cJSON object
        cJSON_Delete(config);
    }
}


void setOutput(int output, int brightness, int fadeMs) {
    int outputIndex = output - 1;
    if (outputIndex < 0 || outputIndex >= MAX_OUTPUTS) return;

    // Set up the parameters for the animation engine
    outputs[outputIndex].startPwmValue = outputs[outputIndex].currentPwmValue;
    outputs[outputIndex].targetPwmValue = (int)roundf(brightness / 100.0f * 4095.0f);
    outputs[outputIndex].fadeStartTime = esp_log_timestamp();
    outputs[outputIndex].fadeDuration = (fadeMs > 0) ? fadeMs : 1; // Prevent division by zero

    // Store the last "ON" brightness for stateful commands
    if (brightness > 0) {
        outputBrightness[outputIndex] = brightness;
    }
}

void processFades() {
    for (int i = 0; i < MAX_OUTPUTS; i++) {
        // Check if a fade is active for this output
        if (outputs[i].currentPwmValue != outputs[i].targetPwmValue) {
            unsigned long elapsedTime = esp_log_timestamp() - outputs[i].fadeStartTime;
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
                outputs[i].currentPwmValue = newPwmValue;
                
                uint8_t addr;
                uint8_t channel;
                // Get the physical address from our mapping component
                if (hsg_outputs_get_mapping(i + 1, &addr, &channel)) {
                    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
                        // Send the raw value to the hardware driver
                        esp_err_t result = hsg_pca9685::pca9685_write_pwm_value(addr, channel, newPwmValue);
                        xSemaphoreGive(i2c_mutex); // Release the lock

                        if (result != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to write PWM value to PCA@0x%02X ch%d. Error: %s", addr, channel, esp_err_to_name(result));
                        }
                    }
                }
            }
        }
    }
}

static void load_bindings() {
    g_bindings.clear();
    cJSON* config = HSG::API::get_config_json_obj();
    if (!config) return;

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
}


/*--------------------------- Main Application Entry Point --------------------*/
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting up...");

    i2c_mutex = xSemaphoreCreateMutex();

    // 1. Initialize NVS (must be first)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Initialize Networking
    s_net_event_group = xEventGroupCreate();
    initialize_network_interfaces();
    
    ESP_LOGI(TAG, "Waiting for network connection...");
    xEventGroupWaitBits(s_net_event_group, ETH_CONNECTED_BIT | WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "Network connected.");
    
    // 3. Initialize the core API component and its services
    HSG::API::Init api_init;
    api_init.i2c_port = I2C_MASTER_NUM;
    api_init.output_cb = [](int out, int brightness, int fade_ms){
        Command cmd = {TargetType::OUTPUT, out, "", brightness, fade_ms};
        processCommand(cmd);
    };
    api_init.group_cb = [](const char* name, int brightness, int fade_ms){
     //   int brightness = (strcmp(state, "ON") == 0) ? 100 : 0;
        Command cmd = {TargetType::GROUP, 0, std::string(name), brightness, fade_ms};
        processCommand(cmd);
    };
    
/*    api_init.config_updated_cb = [](){
        ESP_LOGI(TAG, "Configuration updated, reloading outputs...");
        cJSON* config = HSG::API::get_config_json_obj();
        hsg_outputs_reload_config(config);
        cJSON_Delete(config);
    };
*/
    HSG::API::start(api_init);
    HSG::API::mqtt_start();

    /*
    // 4. *** NEW: Wait for the API to confirm config is loaded ***
    ESP_LOGI(TAG, "Waiting for HSG-API configuration to be loaded...");
    xEventGroupWaitBits(HSG::API::get_event_group(), CONFIG_LOADED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "HSG-API configuration is ready.");
*/
    // 5. Initialize Hardware Drivers and Dependent Components
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C ready: SDA=%d SCL=%d @%dHz", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    
    // 6. Init Outputs by PASSING the config to it
    cJSON* initial_config = HSG::API::get_config_json_obj();
    ESP_ERROR_CHECK(hsg_outputs_init(I2C_MASTER_NUM, initial_config));
    hsg_outputs_clear_all();
    
    // Now that the API has loaded the config, we can init the outputs
    HSG::API::scan_and_prune_i2c(I2C_MASTER_NUM);
    cJSON_Delete(initial_config); // Clean up

    // 7. Start Application Tasks
    xTaskCreate(main_task, "can_task", 4096, NULL, 5, NULL);
    xTaskCreate(animation_task, "animation_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "app_main() Initialization complete.");    
}

/*--------------------------- Main Application Task -------------------------*/
void main_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Main task started.");
    
    if (spi_master_init() != ESP_OK) {
        ESP_LOGE(TAG, "SPI master initialization failed. Halting task.");
        vTaskDelete(NULL);
    }
    
    // 2. Create the MCP2515 object, passing the handle to the constructor
    MCP2515 mcp2515(&mcp_spi_handle);

    // 3. Set the interrupt pin (since the constructor didn't)
    //mcp2515.set_interrupt_pin((gpio_num_t)PIN_NUM_INT);

    // 5. Set the configuration
    mcp2515.reset();
//    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
//    mcp2515.setNormalMode()

    ESP_ERROR_CHECK(mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ));
    ESP_ERROR_CHECK(mcp2515.setNormalMode());
    
 //   ESP_ERROR_CHECK(mcp2515_enable_rx_interrupts(mcp_spi_handle));

    ESP_LOGI(TAG, "MCP2515 initialized successfully.");

    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_INT);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    g_last_mutex = xSemaphoreCreateMutex();
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
   // gpio_isr_handler_add((gpio_num_t)PIN_NUM_INT, gpio_isr_handler, (void*) (uint32_t) PIN_NUM_INT);
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)PIN_NUM_INT, gpio_isr_handler, (void*)PIN_NUM_INT));

    load_bindings();

    while (1) {
        if (mcp2515.checkError()) {
            uint8_t err_flags = mcp2515.getErrorFlags();
            // Check for the bus-off flag (TXBO)
            if (err_flags & MCP2515::EFLG_TXBO) {
                ESP_LOGE(TAG, "CAN bus-off error detected! Attempting to reset MCP2515...");
                // Attempt to reset the controller to clear the bus-off state
                if (mcp2515.reset() == MCP2515::ERROR_OK) {
                    // Re-apply the configuration after reset
                    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
                    mcp2515.setNormalMode();
                    ESP_LOGI(TAG, "MCP2515 reset and reconfigured successfully.");
                } else {
                    ESP_LOGE(TAG, "MCP2515 reset failed.");
                }
            } else {
                ESP_LOGW(TAG, "CAN error detected (flags: 0x%02X), clearing flags.", err_flags);
                mcp2515.clearRXnOVRFlags();
                mcp2515.clearInterrupts();
            }
             // Give the bus a moment to settle after an error
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue; // Skip trying to read a message this cycle
        }
        // --- Check for CAN messages ---
       uint32_t io_num;
        if (xQueueReceive(gpio_evt_queue, &io_num, pdMS_TO_TICKS(5000))) {
            can_frame can_frame;
            if (mcp2515.readMessage(&can_frame) == MCP2515::ERROR_OK) {
                ESP_LOGI(TAG, "CAN Frame Received! ID: 0x%lX", can_frame.can_id);
                // We need to copy the data to the HSG_CanFrame struct
                // 1. Create a temporary frame of the correct API type
                HSG_CanFrame api_frame;
                api_frame.id = can_frame.can_id;
                api_frame.dlc = can_frame.can_dlc;
                memcpy(api_frame.data, can_frame.data, can_frame.can_dlc);
                // 2. Call the public function to update the API's internal cache
                HSG::API::update_last_can(api_frame);
                
                // --- FIX: Decode the message using the new protocol ---
                CanMessageType msgType = getMessageType(can_frame.can_id);

                // --- NEW: Rule-matching engine ---
                // Assume switch sends: [button_num, action_type]
                if (msgType == LIGHTING_COMMAND) {
                    if (can_frame.can_dlc >= 2) {
                        int switchId = getNodeId(can_frame.can_id);
                        int button = can_frame.data[0];
                        const char* action = (can_frame.data[1] == 1) ? "CLICK" : "HOLD"; // Example action mapping

                        // Find a matching rule in our bindings
                        for (const auto& rule : g_bindings) {
                            if (rule.switchId == switchId && rule.button == button && rule.onAction == action) {
                                ESP_LOGI(TAG, "CAN Match Found: Switch %d, Button %d", switchId, button);
                                
                                Command cmd;
                                cmd.type = rule.targetType;
                                cmd.output_id = rule.outputId;
                                cmd.group_name = rule.groupName;
                                cmd.fade_ms = rule.fade_ms;

                                // Handle brightness/state
                                if (rule.brightness != -1) {
                                    cmd.brightness = rule.brightness;
                                } else if (rule.state == "ON") {
                                    cmd.brightness = 100;
                                } else if (rule.state == "OFF") {
                                    cmd.brightness = 0;
                                }
                                // TODO: Add "TOGGLE" logic if needed

                                processCommand(cmd);
                                break; // Stop after finding the first match
                            }
                        }
                    }
                }
                // You could add else if blocks here to handle SENSOR_DATA or HEARTBEAT
                else if (msgType == HEARTBEAT) {
                    uint8_t nodeId = getNodeId(can_frame.can_id);
                    ESP_LOGI(TAG, "Heartbeat received from node 0x%02X", nodeId);
                }
                // --- END FIX ---
            }
        }
    }
}

void animation_task(void *pvParameter)
{
    while (1) {
        processFades();
        vTaskDelay(pdMS_TO_TICKS(16)); // ~60Hz loop for smooth fades
    }
}
/* ---------------------- Hardware & Network Init (STUBS) ---------------------- */
// NOTE: You must provide the full implementations for these functions from your repository.
// The code below are stubs; you need to copy/paste your actual working functions.

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, nullptr);
}

/*
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}
*/
static esp_err_t i2c_master_init(void) {
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

httpd_handle_t web_start() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = nullptr;
    if (httpd_start(&server, &config) == ESP_OK) {
        return server;
    }
    return nullptr;
}


