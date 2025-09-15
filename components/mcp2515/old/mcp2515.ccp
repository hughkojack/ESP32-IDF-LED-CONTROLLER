#include "mcp2515.h"
#include "mcp2515_defs.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

static const char* TAG = "MCP2515";

// --- low-level helpers ---
static esp_err_t spi_cmd_only(spi_device_handle_t spi, uint8_t cmd) {
    spi_transaction_t t = { .length = 8, .tx_buffer = &cmd };
    return spi_device_transmit(spi, &t);
}
static esp_err_t spi_write_register(spi_device_handle_t spi, uint8_t addr, uint8_t val) {
    uint8_t tx[3] = { MCP_WRITE, addr, val };
    spi_transaction_t t = { .length = 24, .tx_buffer = tx };
    return spi_device_transmit(spi, &t);
}
static uint8_t spi_read_register(spi_device_handle_t spi, uint8_t addr) {
    uint8_t tx[3] = { MCP_READ, addr, 0x00 }, rx[3] = {0};
    spi_transaction_t t = { .length = 24, .tx_buffer = tx, .rx_buffer = rx };
    esp_err_t r = spi_device_transmit(spi, &t);
    if (r != ESP_OK) { ESP_LOGE(TAG, "read reg 0x%02X failed: %d", addr, r); return 0; }
    return rx[2];
}
static esp_err_t spi_bit_modify(spi_device_handle_t spi, uint8_t addr, uint8_t mask, uint8_t data) {
    uint8_t tx[4] = { MCP_BITMOD, addr, mask, data };
    spi_transaction_t t = { .length = 32, .tx_buffer = tx };
    return spi_device_transmit(spi, &t);
}

// --- public API ---
esp_err_t mcp2515_init(spi_host_device_t host, int miso, int mosi, int sclk, int cs, spi_device_handle_t *handle) {
    spi_bus_config_t buscfg = {
        .miso_io_num = miso, .mosi_io_num = mosi, .sclk_io_num = sclk,
        .quadwp_io_num = -1, .quadhd_io_num = -1
    };
    spi_device_interface_config_t devcfg = {
        .mode = 0, .clock_speed_hz = 10*1000*1000,
        .spics_io_num = cs, .queue_size = 7
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO), TAG, "bus init");
    ESP_RETURN_ON_ERROR(spi_bus_add_device(host, &devcfg, handle), TAG, "add dev");
    return ESP_OK;
}

esp_err_t mcp2515_reset(spi_device_handle_t spi) {
    return spi_cmd_only(spi, MCP_RESET);
}

// Minimal CNF settings (example: 500k @ 8MHz)
// Adjust as needed for other bitrates/clocks.
esp_err_t mcp2515_set_bitrate(spi_device_handle_t spi, uint8_t bitrate, uint8_t clock) {
    if (bitrate == CAN_500KBPS && clock == MCP_8MHZ) {
        // CNF1..3 values for 500k @ 8MHz (example)
        ESP_RETURN_ON_ERROR(spi_write_register(spi, REG_CNF1, 0x00), TAG, "cnf1");
        ESP_RETURN_ON_ERROR(spi_write_register(spi, REG_CNF2, 0x90), TAG, "cnf2");
        ESP_RETURN_ON_ERROR(spi_write_register(spi, REG_CNF3, 0x02), TAG, "cnf3");
        // Accept-any on both RX buffers
        spi_write_register(spi, REG_RXB0CTRL, 0x60);
        spi_write_register(spi, REG_RXB1CTRL, 0x60);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Unsupported bitrate/clock combo");
    return ESP_ERR_INVALID_ARG;
}

esp_err_t mcp2515_enable_rx_interrupts(spi_device_handle_t spi) {
    // Enable RX0/RX1 interrupts and clear any pending flags
    ESP_RETURN_ON_ERROR(spi_write_register(spi, REG_CANINTE, RX0IE | RX1IE), TAG, "CANINTE");
    ESP_RETURN_ON_ERROR(spi_write_register(spi, REG_CANINTF, 0x00), TAG, "CANINTF");
    return ESP_OK;
}

esp_err_t mcp2515_set_normal_mode(spi_device_handle_t spi) {
    // Normal mode
    ESP_RETURN_ON_ERROR(spi_write_register(spi, REG_CANCTRL, 0x00), TAG, "CANCTRL");
    // Arm RX interrupts & clear flags once more for good measure
    mcp2515_enable_rx_interrupts(spi);
    return ESP_OK;
}

static esp_err_t read_status(spi_device_handle_t spi, uint8_t *status) {
    uint8_t tx[2] = { MCP_RX_STATUS, 0x00 }, rx[2] = {0};
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    esp_err_t r = spi_device_transmit(spi, &t);
    if (r == ESP_OK) *status = rx[1];
    return r;
}

esp_err_t mcp2515_read_message(spi_device_handle_t spi, can_frame *frame) {
    uint8_t st = 0;
    esp_err_t r = read_status(spi, &st);
    if (r != ESP_OK) return r;

    bool has0 = st & 0x40; // RXB0 full
    bool has1 = st & 0x80; // RXB1 full
    if (!has0 && !has1) return ESP_ERR_NOT_FOUND;

    uint8_t cmd  = has0 ? MCP_READ_RXB0 : MCP_READ_RXB1;
    uint8_t flag = has0 ? RX0IF       : RX1IF;

    // 1 (cmd) + 13 bytes = 14 bytes total
    uint8_t tx[14] = {0}, rx[14] = {0};
    tx[0] = cmd;
    spi_transaction_t t = { .length = 8 * sizeof(tx), .tx_buffer = tx, .rx_buffer = rx };
    r = spi_device_transmit(spi, &t);
    if (r != ESP_OK) return r;

    uint8_t sidh = rx[1], sidl = rx[2];
    uint8_t dlc  = rx[5] & 0x0F;
    uint16_t sid = ((uint16_t)sidh << 3) | (sidl >> 5);

    frame->can_id  = sid & 0x7FF;
    frame->can_dlc = (dlc > 8) ? 8 : dlc;
    for (int i = 0; i < frame->can_dlc; ++i) frame->data[i] = rx[6 + i];

    // Clear the specific RXnIF
    spi_bit_modify(spi, REG_CANINTF, flag, 0x00);
    return ESP_OK;
}

esp_err_t mcp2515_send_message(spi_device_handle_t spi, const can_frame *f) {
    // Write ID/DLC/DATA into TXB0, then RTS
    // TXB0SIDH=0x31, SIDL=0x32, EID8=0x33, EID0=0x34, DLC=0x35, DATA0=0x36
    uint8_t sidh = (f->can_id >> 3) & 0xFF;
    uint8_t sidl = (f->can_id & 0x07) << 5;

    ESP_RETURN_ON_ERROR(spi_write_register(spi, 0x31, sidh), TAG, "SIDH");
    ESP_RETURN_ON_ERROR(spi_write_register(spi, 0x32, sidl), TAG, "SIDL");
    ESP_RETURN_ON_ERROR(spi_write_register(spi, 0x35, f->can_dlc & 0x0F), TAG, "DLC");

    for (int i = 0; i < f->can_dlc; ++i) {
        ESP_RETURN_ON_ERROR(spi_write_register(spi, 0x36 + i, f->data[i]), TAG, "DATA");
    }

    return spi_cmd_only(spi, MCP_RTS_TXB0);
}
