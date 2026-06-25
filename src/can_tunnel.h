#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"
#include "esp_http_server.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

class MCP2515;
struct can_frame;

#define CAN_TUNNEL_FRAME_SIZE 13
#define CAN_TUNNEL_TCP_PORT   5250

void can_tunnel_set_hw(MCP2515* mcp, SemaphoreHandle_t mutex);
void can_tunnel_set_hub_rx_queue(QueueHandle_t queue);
bool can_tunnel_is_active(void);
bool can_tunnel_client_connected(void);
esp_err_t can_tunnel_register_http(httpd_handle_t server);
