// hsg_pca9685.cpp
#include "hsg_pca9685.h"
#include "esp_log.h"
#include <math.h>
#include "driver/i2c.h"

static const char* TAG = "PCA9685";

// -----------------------------------------------------------------------------
// Low-level function: set a raw 12-bit PWM value (0-4095) on one channel.
// This is now called by the processFades() loop in main.cpp.
// -----------------------------------------------------------------------------
esp_err_t pca9685_write_pwm_value(i2c_port_t port, uint8_t addr, int channel, uint16_t value)
{
    if (channel < 0 || channel > 15) {
        ESP_LOGE(TAG, "Invalid channel %d", channel);
        return ESP_ERR_INVALID_ARG;
    }

    // Clamp value to the 12-bit range of the PCA9685
    if (value > 4095) value = 4095;

    // PCA9685 registers for this channel
    uint8_t reg = 0x06 + 4 * channel; // LEDX_ON_L register address
    uint8_t buf[5];

    buf[0] = reg;
    buf[1] = 0x00;           // LEDn_ON_L (always start at time 0)
    buf[2] = 0x00;           // LEDn_ON_H
    buf[3] = value & 0xFF;   // LEDn_OFF_L
    buf[4] = value >> 8;     // LEDn_OFF_H

    // Use the generic I2C write function from the driver
    return i2c_master_write_to_device(port, addr, buf, sizeof(buf), pdMS_TO_TICKS(50));
}



// Helper function to write to a single register
static esp_err_t write_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(port, addr, buf, 2, pdMS_TO_TICKS(50));
}

// -----------------------------------------------------------------------------
// Initialize a single PCA9685 chip
// -----------------------------------------------------------------------------
esp_err_t pca9685_init(i2c_port_t port, uint8_t addr, int freq_hz)
{
    ESP_LOGI(TAG, "Init PCA9685 @0x%02X freq=%dHz", addr, freq_hz);

    // MODE1 reset
    ESP_ERROR_CHECK(write_reg(port, addr, 0x00, 0x00));

    // Calculate and set prescale for the requested frequency
    float prescale_val = 25000000.0 / (4096.0 * freq_hz) - 1;
    uint8_t prescale = (uint8_t)floor(prescale_val + 0.5);

    // Go to sleep before setting prescale, then wake up with auto-increment
    ESP_ERROR_CHECK(write_reg(port, addr, 0x00, 0x10)); // MODE1 sleep
    ESP_ERROR_CHECK(write_reg(port, addr, 0xFE, prescale)); // Set prescale register
    ESP_ERROR_CHECK(write_reg(port, addr, 0x00, 0xA1)); // MODE1 auto-increment, restart

    return ESP_OK;
}

