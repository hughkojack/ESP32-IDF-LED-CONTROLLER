// hsg_pca9685.cpp
#include "hsg_pca9685.h"
#include "esp_log.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "PCA9685";

// Forward declaration of your low-level writer (already existing)
esp_err_t pca9685_write_pwm(i2c_port_t port, uint8_t addr, int channel, int duty);

// Keep track of last duty for fade calculation
static int last_duty[128][16] = {{0}};

// write register helper
static esp_err_t write_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(port, addr, buf, 2, pdMS_TO_TICKS(50));
}

esp_err_t pca9685_init(i2c_port_t port, uint8_t addr, int freq_hz)
{
    ESP_LOGI(TAG, "Init PCA9685 @0x%02X freq=%dHz", addr, freq_hz);

    // MODE1 reset
    ESP_ERROR_CHECK(write_reg(port, addr, 0x00, 0x00));

    // Set prescale for freq
    float prescale_val = 25000000.0 / (4096.0 * freq_hz) - 1;
    uint8_t prescale = (uint8_t)floor(prescale_val + 0.5);

    // Go to sleep before setting prescale
    ESP_ERROR_CHECK(write_reg(port, addr, 0x00, 0x10)); // MODE1 sleep
    ESP_ERROR_CHECK(write_reg(port, addr, 0xFE, prescale)); // prescale
    ESP_ERROR_CHECK(write_reg(port, addr, 0x00, 0xA1)); // MODE1 auto-increment, restart

    return ESP_OK;
}

esp_err_t pca9685_set_pwm(i2c_port_t port, uint8_t addr, int channel, int duty, int fade_ms)
{
    if (channel < 0 || channel > 15) return ESP_ERR_INVALID_ARG;

    // duty is expected 0–100 (percent)
    int off = (duty * 4096) / 100;
    if (off > 4095) off = 4095;
    if (off < 0) off = 0;

    uint8_t reg = 0x06 + 4 * channel;
    uint8_t buf[5];
    buf[0] = reg;
    buf[1] = 0;          // ON_L
    buf[2] = 0;          // ON_H
    buf[3] = off & 0xFF; // OFF_L
    buf[4] = off >> 8;   // OFF_H

    return i2c_master_write_to_device(port, addr, buf, 5, pdMS_TO_TICKS(50));
}
