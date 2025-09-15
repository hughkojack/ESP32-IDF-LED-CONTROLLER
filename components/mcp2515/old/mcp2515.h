#ifndef MCP2515_H
#define MCP2515_H

#include "driver/spi_master.h"
#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t can_id;  // 11-bit (0..0x7FF) used here
    uint8_t  can_dlc; // 0..8
    uint8_t  data[8];
} can_frame;

// API
esp_err_t mcp2515_init(spi_host_device_t host, int miso, int mosi, int sclk, int cs, spi_device_handle_t *handle);
esp_err_t mcp2515_reset(spi_device_handle_t spi);
esp_err_t mcp2515_set_bitrate(spi_device_handle_t spi, uint8_t bitrate, uint8_t clock);
esp_err_t mcp2515_enable_rx_interrupts(spi_device_handle_t spi);
esp_err_t mcp2515_set_normal_mode(spi_device_handle_t spi);

esp_err_t mcp2515_read_message(spi_device_handle_t spi, can_frame *frame);
esp_err_t mcp2515_send_message(spi_device_handle_t spi, const can_frame *frame);

#ifdef __cplusplus
}
#endif
#endif
