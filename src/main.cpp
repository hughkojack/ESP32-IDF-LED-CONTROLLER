// main.cpp  (ESP-IDF)
// Wi-Fi <-> Ethernet failover + I2C + simple web + MCP2515

extern "C" {
#include "mcp2515.h"
#include "mcp2515_defs.h"
}

#include <cstdio>
#include <cstring>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
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

#include "HSG-API.h"

static const char *TAG = "MAIN";

/* ---------------------- Net failover state ---------------------- */
static EventGroupHandle_t s_net_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int ETH_CONNECTED_BIT  = BIT1;

static esp_netif_t *eth_netif  = nullptr;
static esp_netif_t *wifi_netif = nullptr;

static bool is_eth_active  = false;
static bool is_wifi_active = false;

#define CONFIG_USE_ETH 1

// --- Olimex ESP32-POE wiring ---
#define ETH_MDC_GPIO               23
#define ETH_MDIO_GPIO              18
#define ETH_PHY_ADDR               0      // try 1 only if 0 fails
#define ETH_RST_GPIO               -1     // no dedicated reset line
#define ETH_PHY_PWR_GPIO           12     // powers the PHY (strap pin! keep low at boot)
#define ETH_PHY_POWER_ACTIVE_HIGH  1

/* ---------------------- I2C (kept) ---------------------- */
#define I2C_MASTER_SCL_IO   16
#define I2C_MASTER_SDA_IO   13
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  400000

static void phy_power_set(bool on) {
    // Configure once
    static bool inited = false;
    if (!inited) {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << ETH_PHY_PWR_GPIO;
        io.mode = GPIO_MODE_OUTPUT;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;  // rely on external pulldown for boot strap!
        io.pull_up_en   = GPIO_PULLUP_DISABLE;
        gpio_config(&io);
        inited = true;
    }
#if ETH_PHY_POWER_ACTIVE_HIGH
    gpio_set_level((gpio_num_t)ETH_PHY_PWR_GPIO, on ? 1 : 0);
#else
    gpio_set_level((gpio_num_t)ETH_PHY_PWR_GPIO, on ? 0 : 1);
#endif
}

static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = 0;
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_probe(uint8_t addr7) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr7 << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(20));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ---------------------- MCP2515 pins ---------------------- */
#define MPC_SPI_HOST SPI2_HOST
#define PIN_NUM_MISO 33
#define PIN_NUM_MOSI 32
#define PIN_NUM_CLK  4
#define PIN_NUM_CS   5
#define PIN_NUM_INT  35    // input-only; needs external pull-up

static QueueHandle_t gpio_evt_queue = nullptr;
static spi_device_handle_t mcp_spi;
static volatile uint32_t g_int_count = 0;

/* ---------------------- Web state ---------------------- */
static can_frame g_last_frame = {};
static SemaphoreHandle_t g_last_mutex;
static httpd_handle_t g_http = nullptr;

/* ---------------------- Wi-Fi config ---------------------- */
#ifndef WIFI_SSID
#define WIFI_SSID "HSG"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "myhomesecurity9981"
#endif

/* ---------------------- Forward decls (failover) ---------------------- */
static void switch_to_ethernet(void);
static void switch_to_wifi(void);
static void initialize_network_interfaces(void);

/* ---------------------- HTTP helpers ---------------------- */
static esp_err_t send_json(httpd_req_t *req, cJSON *root) {
    char *out = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, out ? out : "{}");
    if (out) free(out);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t h_i2c_scan(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "bus", I2C_MASTER_NUM);
    cJSON *arr = cJSON_AddArrayToObject(root, "addrs");
    for (uint8_t a = 0x03; a <= 0x77; ++a) {
        if (i2c_probe(a) == ESP_OK) {
            char buf[6];
            snprintf(buf, sizeof(buf), "0x%02X", a);
            cJSON_AddItemToArray(arr, cJSON_CreateString(buf));
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return send_json(req, root);
}

static esp_err_t h_can_last(httpd_req_t *req) {
    can_frame f;
    xSemaphoreTake(g_last_mutex, portMAX_DELAY);
    f = g_last_frame;
    xSemaphoreGive(g_last_mutex);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "id", (int)f.can_id);
    cJSON_AddNumberToObject(root, "dlc", f.can_dlc);
    char hex[3*8+1] = {0};
    int p = 0;
    for (int i = 0; i < f.can_dlc; ++i)
        p += snprintf(hex + p, sizeof(hex) - p, "%02X%s", f.data[i], (i+1 < f.can_dlc) ? " " : "");
    cJSON_AddStringToObject(root, "data", hex);
    return send_json(req, root);
}

static esp_err_t h_can_send(httpd_req_t *req) {
    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad body");
    buf[len] = 0;

    cJSON *root = cJSON_Parse(buf);
    if (!root) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "json");

    can_frame f = {};
    cJSON *jid   = cJSON_GetObjectItem(root, "id");
    cJSON *jdata = cJSON_GetObjectItem(root, "data");
    if (!cJSON_IsNumber(jid) || !cJSON_IsString(jdata)) {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "fields");
    }

    f.can_id = (uint32_t)jid->valuedouble;
    const char *hex = jdata->valuestring;
    size_t n = strlen(hex);
    uint8_t out[8] = {0};
    int outlen = 0;
    for (size_t i = 0; i + 1 < n && outlen < 8; i += 2) {
        unsigned v;
        if (sscanf(hex + i, "%02x", &v) != 1) break;
        out[outlen++] = (uint8_t)v;
    }
    f.can_dlc = outlen;
    for (int i = 0; i < outlen; ++i) f.data[i] = out[i];

    esp_err_t r = mcp2515_send_message(mcp_spi, &f);
    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "status", (r == ESP_OK) ? "ok" : "err");
    cJSON_AddNumberToObject(resp, "sent_bytes", f.can_dlc);
    cJSON_AddNumberToObject(resp, "id", (int)f.can_id);
    cJSON_Delete(root);
    return send_json(req, resp);
}

httpd_handle_t web_start()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_handle_t server = nullptr;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: %s", esp_err_to_name(ret));
        return nullptr;
    }

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);

    // If you have static file handlers or UI endpoints, register them here

    return server;
}

/* ---------------------- MCP2515 IRQ ---------------------- */
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    g_int_count++;
    if (gpio_evt_queue) xQueueSendFromISR(gpio_evt_queue, &gpio_num, nullptr);
}

/* ---------------------- Wi-Fi + Ethernet init & failover ---------------------- */

// Prefer Ethernet when it has IP; otherwise Wi-Fi
static void switch_to_ethernet(void) {
    if (eth_netif) {
        esp_netif_set_default_netif(eth_netif);
        is_eth_active  = true;
        is_wifi_active = false;
        ESP_LOGI(TAG, "Default route -> ETHERNET");
    }
}
static void switch_to_wifi(void) {
    EventBits_t bits = xEventGroupGetBits(s_net_event_group);
    if ((bits & WIFI_CONNECTED_BIT) && wifi_netif) {
        esp_netif_set_default_netif(wifi_netif);
        is_eth_active  = false;
        is_wifi_active = true;
        ESP_LOGI(TAG, "Default route -> WIFI");
    } else {
        ESP_LOGW(TAG, "Cannot switch to Wi-Fi: no IP yet");
    }
}

static void eth_event_handler(void *, esp_event_base_t base, int32_t id, void *) {
    if (base != ETH_EVENT) return;
    switch (id) {
        case ETHERNET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Ethernet Link Up");
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Ethernet Link Down -> failover to Wi-Fi (if available)");
            is_eth_active = false;
            switch_to_wifi();
            break;
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "Ethernet Started");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGI(TAG, "Ethernet Stopped");
            is_eth_active = false;
            break;
        default: break;
    }
}

static void ip_event_handler(void *, esp_event_base_t base, int32_t id, void *data) {
    if (base != IP_EVENT) return;
    switch (id) {
        case IP_EVENT_ETH_GOT_IP: {
            auto *e = (ip_event_got_ip_t *)data;
            ESP_LOGI(TAG, "ETH IP: " IPSTR, IP2STR(&e->ip_info.ip));
            xEventGroupSetBits(s_net_event_group, ETH_CONNECTED_BIT);
            switch_to_ethernet();
            break;
        }
        case IP_EVENT_STA_GOT_IP: {
            auto *e = (ip_event_got_ip_t *)data;
            ESP_LOGI(TAG, "WIFI IP: " IPSTR, IP2STR(&e->ip_info.ip));
            xEventGroupSetBits(s_net_event_group, WIFI_CONNECTED_BIT);
            if (!is_eth_active) switch_to_wifi();
            break;
        }
        case IP_EVENT_STA_LOST_IP:
            ESP_LOGI(TAG, "Wi-Fi lost IP");
            xEventGroupClearBits(s_net_event_group, WIFI_CONNECTED_BIT);
            break;
        default: break;
    }
}

static void wifi_start_sta(void) {
    esp_netif_inherent_config_t base = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
    wifi_netif = esp_netif_create_default_wifi_sta();
    assert(wifi_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {};
    std::strncpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    std::strncpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

static void eth_start(void) {
#ifdef CONFIG_USE_ETH
    // Create netif
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    eth_netif = esp_netif_new(&netif_cfg);
    if (!eth_netif) { ESP_LOGE(TAG, "netif_new failed"); return; }

    // Power up PHY first (GPIO12)
    phy_power_set(true);
    vTaskDelay(pdMS_TO_TICKS(50));  // let 3V3 & XO settle

    // EMAC low-level pins (SMI)
    eth_esp32_emac_config_t esp32_emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    esp32_emac_cfg.smi_mdc_gpio_num  = ETH_MDC_GPIO;   // 23
    esp32_emac_cfg.smi_mdio_gpio_num = ETH_MDIO_GPIO;  // 18

    // MAC / PHY configs
    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr       = ETH_PHY_ADDR;  // 0 on most ESP32-POE
    phy_cfg.reset_gpio_num = ETH_RST_GPIO;  // -1 (no reset pin)

    // Create MAC and PHY (LAN87xx == LAN8710/8720 family)
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&esp32_emac_cfg, &mac_cfg);
    if (!mac) { ESP_LOGE(TAG, "esp_eth_mac_new_esp32 failed"); return; }

    esp_eth_phy_t *phy = esp_eth_phy_new_lan87xx(&phy_cfg);
    if (!phy) { ESP_LOGE(TAG, "esp_eth_phy_new_lan87xx failed"); return; }

    // Install, attach, start (fail-soft, do not reboot)
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    esp_eth_handle_t eth_handle = nullptr;

    esp_err_t err = esp_eth_driver_install(&eth_cfg, &eth_handle);
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_eth_driver_install: %s", esp_err_to_name(err)); return; }

    err = esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle));
    if (err != ESP_OK) { ESP_LOGE(TAG, "netif_attach: %s", esp_err_to_name(err)); return; }

    err = esp_eth_start(eth_handle);
    if (err != ESP_OK) { ESP_LOGE(TAG, "esp_eth_start: %s", esp_err_to_name(err)); return; }

    ESP_LOGI(TAG, "Ethernet start requested (Olimex ESP32-POE, LAN87xx)");
#else
    ESP_LOGW(TAG, "Ethernet disabled (CONFIG_USE_ETH not set)");
#endif
}

static void initialize_network_interfaces(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_net_event_group = xEventGroupCreate();

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,  ESP_EVENT_ANY_ID, &ip_event_handler,  nullptr));

    // Start interfaces
    wifi_start_sta();  // always start Wi-Fi
    eth_start();       // start ETH if enabled

    #ifdef CONFIG_USE_ETH
        ESP_LOGI(TAG, "Failover manager started (Wi-Fi enabled, ETH enabled)");
    #else
        ESP_LOGI(TAG, "Failover manager started (Wi-Fi enabled, ETH disabled)");
    #endif
}

/* ---------------------- app_main ---------------------- */
extern "C" void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    // 1) Start network failover stack first
    initialize_network_interfaces();
    ESP_LOGI(TAG, "Waiting for network...");
    EventBits_t bits = xEventGroupWaitBits(
        s_net_event_group, WIFI_CONNECTED_BIT | ETH_CONNECTED_BIT,
        pdFALSE, pdFALSE, portMAX_DELAY);
    (void)bits;

    //------------------------------------------------------------------------------------------
    // Pass API init callbacks
    HSG::API::Init api_init;
    api_init.i2c_port = I2C_NUM_0;        // your initialized I2C port
    api_init.output_cb = [](int out, int brightness, int fade_ms){
        // TODO: map logical 'out' to PCA9685 (addr/channel) & set duty
    };
    api_init.group_cb = [](const char* name, const char* state, int fade_ms){
        // TODO: look up group -> outputs in stored JSON (HSG::API::get_config_json()) and set each
    };

    ESP_ERROR_CHECK(HSG::API::start(api_init));
    
    // Register all API URIs on the existing server
    //ESP_ERROR_CHECK(HSG::API::register_uris(server, api_init));
    //------------------------------------------------------------------------------------------

    // 2) Bring up HTTP after at least one IP is available
 //   httpd_handle_t server = web_start();
 
    
    // 3) I2C ready
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C ready: SDA=%d SCL=%d @%dHz",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);

    // 4) MCP2515 + IRQ
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask  = 1ULL << PIN_NUM_INT;
    io_conf.mode          = GPIO_MODE_INPUT;
    io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;  // GPIO35 has no internal PU
    io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type     = GPIO_INTR_NEGEDGE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)PIN_NUM_INT, gpio_isr_handler, (void*)PIN_NUM_INT));

    ESP_ERROR_CHECK(mcp2515_init(MPC_SPI_HOST, PIN_NUM_MISO, PIN_NUM_MOSI,
                                 PIN_NUM_CLK, PIN_NUM_CS, &mcp_spi));
    ESP_ERROR_CHECK(mcp2515_reset(mcp_spi));
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_ERROR_CHECK(mcp2515_set_bitrate(mcp_spi, CAN_500KBPS, MCP_8MHZ)); // adjust for 16MHz modules
    ESP_ERROR_CHECK(mcp2515_enable_rx_interrupts(mcp_spi));
    ESP_ERROR_CHECK(mcp2515_set_normal_mode(mcp_spi));
    ESP_LOGI(TAG, "MCP2515 ready");

    gpio_evt_queue = xQueueCreate(16, sizeof(uint32_t));
    g_last_mutex   = xSemaphoreCreateMutex();

    // 5) Main loop (IRQ-driven CAN receive)
    while (true) {
        uint32_t io_num;
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            can_frame rx;
            for (;;) {
                esp_err_t r = mcp2515_read_message(mcp_spi, &rx);
                if (r == ESP_OK) {
                    ESP_LOGI(TAG, "CAN RX: ID=0x%lX LEN=%d", rx.can_id, rx.can_dlc);
                    for (int i = 0; i < rx.can_dlc; ++i) std::printf("%02X ", rx.data[i]);
                    std::printf("\n");
                    xSemaphoreTake(g_last_mutex, portMAX_DELAY);
                    g_last_frame = rx;
                    xSemaphoreGive(g_last_mutex);
                } else {
                    break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}