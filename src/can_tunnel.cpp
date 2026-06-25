#include "can_tunnel.h"

#include "mcp2515.h"
#include "can_protocol.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"

#include "lwip/sockets.h"
#include "lwip/netdb.h"

#include <string.h>
#include <errno.h>

static const char* TAG = "can_tunnel";

static MCP2515* s_mcp = nullptr;
static SemaphoreHandle_t s_mutex = nullptr;
static QueueHandle_t s_hub_rx_queue = nullptr;
static volatile bool s_exclusive = false;
static volatile bool s_client_connected = false;
static int s_listen_fd = -1;
static int s_client_fd = -1;
static TaskHandle_t s_task = nullptr;
static TickType_t s_exclusive_since_ticks = 0;
static const TickType_t kNoClientReleaseTicks = pdMS_TO_TICKS(60000);

static const uint8_t kMagic0 = 0xCA;
static const uint8_t kMagic1 = 0xFE;

static esp_err_t send_json(httpd_req_t* req, cJSON* root) {
    char* str = cJSON_PrintUnformatted(root);
    if (!str) {
        cJSON_Delete(root);
        return ESP_ERR_NO_MEM;
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t err = httpd_resp_sendstr(req, str);
    cJSON_free(str);
    cJSON_Delete(root);
    return err;
}

static void close_client_locked(void) {
    if (s_client_fd >= 0) {
        shutdown(s_client_fd, SHUT_RDWR);
        close(s_client_fd);
        s_client_fd = -1;
    }
    s_client_connected = false;
    if (s_exclusive) {
        ESP_LOGI(TAG, "tunnel client gone — releasing exclusive CAN mode");
        s_exclusive = false;
    }
}

static bool tunnel_frame_for_hub(CanMessageType mt) {
    return mt == HEARTBEAT || mt == INPUT_EVENT || mt == CAN_OTA_NODE;
}

static void tunnel_feed_hub_rx(const can_frame* rx) {
    if (!s_hub_rx_queue || !rx || !isStandardDataFrame(rx->can_id))
        return;
    const CanMessageType mt = peekMessageType(getStandardId(rx->can_id));
    if (!tunnel_frame_for_hub(mt))
        return;
    if (xQueueSend(s_hub_rx_queue, rx, 0) != pdPASS) {
        ESP_LOGW(TAG, "hub RX queue full, dropped 0x%03X", (unsigned)getStandardId(rx->can_id));
    }
}

static void close_listen(void) {
    if (s_listen_fd >= 0) {
        close(s_listen_fd);
        s_listen_fd = -1;
    }
}

static bool encode_frame(const can_frame* f, uint8_t out[CAN_TUNNEL_FRAME_SIZE]) {
    if (!f || !out)
        return false;
    const uint16_t sid = (uint16_t)(f->can_id & CAN_SFF_MASK);
    uint8_t dlc = f->can_dlc > 8 ? 8 : f->can_dlc;
    out[0] = kMagic0;
    out[1] = kMagic1;
    out[2] = (uint8_t)(sid & 0xFF);
    out[3] = (uint8_t)((sid >> 8) & 0xFF);
    out[4] = dlc;
    memset(&out[5], 0, 8);
    if (dlc)
        memcpy(&out[5], f->data, dlc);
    return true;
}

static bool decode_frame(const uint8_t in[CAN_TUNNEL_FRAME_SIZE], can_frame* f) {
    if (!in || !f)
        return false;
    if (in[0] != kMagic0 || in[1] != kMagic1)
        return false;
    memset(f, 0, sizeof(*f));
    const uint16_t sid = (uint16_t)((uint16_t)in[2] | ((uint16_t)in[3] << 8));
    uint8_t dlc = in[4];
    if (dlc > 8)
        return false;
    f->can_id = sid;
    f->can_dlc = dlc;
    memcpy(f->data, &in[5], dlc);
    return true;
}

static bool tunnel_send_can(const can_frame* frame) {
    if (!s_mcp || !s_mutex || !frame)
        return false;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(500)) != pdTRUE)
        return false;
    const MCP2515::ERROR err = s_mcp->sendMessage(frame);
    xSemaphoreGive(s_mutex);
    return err == MCP2515::ERROR_OK;
}

static int tunnel_drain_can_to_tcp(int client_fd) {
    if (!s_mcp || !s_mutex)
        return 0;
    const bool forward_tcp = client_fd >= 0;
    int sent = 0;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) != pdTRUE)
        return 0;
    can_frame rx;
    while (s_mcp->readMessage(&rx) == MCP2515::ERROR_OK) {
        if (!isStandardDataFrame(rx.can_id))
            continue;
        tunnel_feed_hub_rx(&rx);
        if (!forward_tcp)
            continue;
        uint8_t wire[CAN_TUNNEL_FRAME_SIZE];
        if (!encode_frame(&rx, wire))
            continue;
        ssize_t w = send(client_fd, wire, sizeof(wire), 0);
        if (w != (ssize_t)sizeof(wire)) {
            xSemaphoreGive(s_mutex);
            return -1;
        }
        sent++;
    }
    if (s_mcp->checkError()) {
        s_mcp->clearRXnOVRFlags();
        s_mcp->clearInterrupts();
    }
    xSemaphoreGive(s_mutex);
    return sent;
}

static bool tunnel_drain_can_burst(int client_fd) {
    for (int i = 0; i < 16; i++) {
        const int n = tunnel_drain_can_to_tcp(client_fd);
        if (n < 0)
            return false;
        if (n == 0)
            break;
    }
    return true;
}

static void tunnel_task(void* arg) {
    (void)arg;
    uint8_t rx_buf[512];
    size_t rx_len = 0;

    ESP_LOGI(TAG, "tunnel task started");

    while (1) {
        if (!s_exclusive) {
            vTaskDelay(pdMS_TO_TICKS(100));
            rx_len = 0;
            continue;
        }

        if (s_listen_fd < 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (s_client_fd < 0) {
            (void)tunnel_drain_can_burst(-1);
            if (s_exclusive_since_ticks != 0
                && (xTaskGetTickCount() - s_exclusive_since_ticks) >= kNoClientReleaseTicks) {
                ESP_LOGW(TAG, "no tunnel client for 60s — releasing exclusive CAN mode");
                s_exclusive = false;
                s_exclusive_since_ticks = 0;
                continue;
            }
            struct sockaddr_in addr;
            socklen_t alen = sizeof(addr);
            int fd = accept(s_listen_fd, (struct sockaddr*)&addr, &alen);
            if (fd < 0) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            int yes = 1;
            setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
            struct timeval tv = { .tv_sec = 0, .tv_usec = 10000 };
            setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            s_client_fd = fd;
            s_client_connected = true;
            rx_len = 0;
            ESP_LOGI(TAG, "tunnel client connected");
        }

        uint8_t chunk[256];
        (void)tunnel_drain_can_burst(s_client_fd);
        ssize_t n = recv(s_client_fd, chunk, sizeof(chunk), 0);
        if (n > 0) {
            if (rx_len + (size_t)n > sizeof(rx_buf)) {
                ESP_LOGW(TAG, "tunnel TCP RX buffer full (%u bytes), dropping oldest data",
                         (unsigned)rx_len);
                const size_t keep = sizeof(rx_buf) / 2;
                if (rx_len > keep)
                    memmove(rx_buf, rx_buf + (rx_len - keep), keep);
                rx_len = keep;
            }
            memcpy(rx_buf + rx_len, chunk, (size_t)n);
            rx_len += (size_t)n;
            while (rx_len >= CAN_TUNNEL_FRAME_SIZE) {
                if (rx_buf[0] != kMagic0 || rx_buf[1] != kMagic1) {
                    memmove(rx_buf, rx_buf + 1, rx_len - 1);
                    rx_len--;
                    continue;
                }
                can_frame tx;
                if (!decode_frame(rx_buf, &tx)) {
                    memmove(rx_buf, rx_buf + 1, rx_len - 1);
                    rx_len--;
                    continue;
                }
                if (!tunnel_send_can(&tx)) {
                    ESP_LOGW(TAG, "tunnel CAN TX failed");
                }
                if (!tunnel_drain_can_burst(s_client_fd)) {
                    ESP_LOGI(TAG, "tunnel TCP send failed, closing client");
                    close_client_locked();
                    rx_len = 0;
                    break;
                }
                memmove(rx_buf, rx_buf + CAN_TUNNEL_FRAME_SIZE, rx_len - CAN_TUNNEL_FRAME_SIZE);
                rx_len -= CAN_TUNNEL_FRAME_SIZE;
            }
        } else if (n == 0 || (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
            ESP_LOGI(TAG, "tunnel client disconnected");
            close_client_locked();
            rx_len = 0;
            continue;
        }

        if (!tunnel_drain_can_burst(s_client_fd)) {
            ESP_LOGI(TAG, "tunnel TCP send failed, closing client");
            close_client_locked();
            rx_len = 0;
        }
    }
}

static esp_err_t ensure_listen_socket(void) {
    if (s_listen_fd >= 0)
        return ESP_OK;
    int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (fd < 0)
        return ESP_FAIL;
    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(CAN_TUNNEL_TCP_PORT);
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        close(fd);
        ESP_LOGE(TAG, "bind failed errno=%d", errno);
        return ESP_FAIL;
    }
    if (listen(fd, 1) != 0) {
        close(fd);
        return ESP_FAIL;
    }
    s_listen_fd = fd;
    ESP_LOGI(TAG, "listening on TCP port %d", CAN_TUNNEL_TCP_PORT);
    return ESP_OK;
}

static esp_err_t h_can_tunnel_open(httpd_req_t* req) {
    if (req->method != HTTP_POST)
        return httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "POST only");
    if (!s_mcp || !s_mutex)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "CAN not ready");
    if (ensure_listen_socket() != ESP_OK)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "listen failed");
    s_exclusive = true;
    s_exclusive_since_ticks = xTaskGetTickCount();
    if (!s_task) {
        BaseType_t ok = xTaskCreatePinnedToCore(tunnel_task, "can_tunnel", 6144, nullptr, 16, &s_task, 1);
        if (ok != pdPASS) {
            s_exclusive = false;
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "task create failed");
        }
    }
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "active", true);
    cJSON_AddNumberToObject(root, "port", CAN_TUNNEL_TCP_PORT);
    cJSON_AddStringToObject(root, "frame_format", "CAFE + can_id_le16 + dlc + data[8]");
    return send_json(req, root);
}

static esp_err_t h_can_tunnel_close(httpd_req_t* req) {
    if (req->method != HTTP_POST)
        return httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "POST only");
    s_exclusive = false;
    s_exclusive_since_ticks = 0;
    close_client_locked();
    close_listen();
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "active", false);
    return send_json(req, root);
}

static esp_err_t h_can_tunnel_status(httpd_req_t* req) {
    cJSON* root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "active", s_exclusive);
    cJSON_AddNumberToObject(root, "port", CAN_TUNNEL_TCP_PORT);
    cJSON_AddBoolToObject(root, "client_connected", s_client_connected);
    return send_json(req, root);
}

void can_tunnel_set_hw(MCP2515* mcp, SemaphoreHandle_t mutex) {
    s_mcp = mcp;
    s_mutex = mutex;
}

void can_tunnel_set_hub_rx_queue(QueueHandle_t queue) {
    s_hub_rx_queue = queue;
}

bool can_tunnel_is_active(void) {
    return s_exclusive;
}

bool can_tunnel_client_connected(void) {
    return s_client_connected;
}

esp_err_t can_tunnel_register_http(httpd_handle_t server) {
    if (!server)
        return ESP_ERR_INVALID_ARG;
    httpd_uri_t open_uri = {
        .uri = "/api/can/tunnel/open",
        .method = HTTP_POST,
        .handler = h_can_tunnel_open,
        .user_ctx = nullptr,
    };
    httpd_uri_t close_uri = {
        .uri = "/api/can/tunnel/close",
        .method = HTTP_POST,
        .handler = h_can_tunnel_close,
        .user_ctx = nullptr,
    };
    httpd_uri_t status_uri = {
        .uri = "/api/can/tunnel/status",
        .method = HTTP_GET,
        .handler = h_can_tunnel_status,
        .user_ctx = nullptr,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &open_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &close_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &status_uri));
    return ESP_OK;
}
