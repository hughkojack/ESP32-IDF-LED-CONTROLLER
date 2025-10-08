// hsg_pca9685.cpp
#include "hsg_pca9685.h"
#include "esp_log.h"
#include <math.h>
#include "driver/i2c_master.h"

static const char* TAG = "PCA9685";

extern i2c_master_bus_handle_t i2c_bus_handle;

namespace hsg_pca9685 {

// Helper function to add a device to the bus
static i2c_master_dev_handle_t add_pca9685_device(uint8_t addr) {
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = 400000; // Standard 400KHz for PCA9685

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &dev_handle));
    return dev_handle;
}

// -----------------------------------------------------------------------------
// Low-level function: set a raw 12-bit PWM value (0-4095) on one channel.
// This is now called by the processFades() loop in main.cpp.
// -----------------------------------------------------------------------------
// Low-level function using the new driver
esp_err_t pca9685_write_pwm_value(uint8_t addr, int channel, uint16_t value) {
    if (channel < 0 || channel > 15) return ESP_ERR_INVALID_ARG;
    if (value > 4095) value = 4095;

    uint8_t reg = 0x06 + 4 * channel;
    uint8_t buf[5];
    buf[0] = reg;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = value & 0xFF;
    buf[4] = value >> 8;
    
    i2c_master_dev_handle_t dev_handle = add_pca9685_device(addr);
    if (!dev_handle) return ESP_FAIL;
    
    esp_err_t ret = i2c_master_transmit(dev_handle, buf, sizeof(buf), -1);

    i2c_master_bus_rm_device(dev_handle);
    return ret;
}

// Helper function to write to a single register using the new driver
static esp_err_t write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    i2c_master_dev_handle_t dev_handle = add_pca9685_device(addr);
    if (!dev_handle) return ESP_FAIL;
    
    uint8_t buf[2] = { reg, val };
    esp_err_t ret = i2c_master_transmit(dev_handle, buf, sizeof(buf), -1); // -1 for max timeout
    
    i2c_master_bus_rm_device(dev_handle); // Clean up
    return ret;
}

// -----------------------------------------------------------------------------
// Initialize a single PCA9685 chip
// -----------------------------------------------------------------------------
// Initialize a single PCA9685 chip using the new driver
esp_err_t pca9685_init(uint8_t addr, int freq_hz) {

    esp_err_t ret;
    ret = write_reg(addr, 0x00, 0x10); // Attempt to put the device into sleep mode.
    
    if (ret != ESP_OK) {
        // If the first write fails, the device is not present or not responding.
        ESP_LOGE(TAG, "PCA9685 @0x%02X did not respond. Bypassing setup.", addr);
        return ret; // Propagate the error code up to the calling function.
    }

    ESP_LOGI(TAG, "PCA9685 @0x%02X is online. Freq=%dHz", addr, freq_hz);
    
    float prescale_val = 25000000.0 / (4096.0 * freq_hz) - 1;
    uint8_t prescale = (uint8_t)floor(prescale_val + 0.5);

    // Check every subsequent write for errors, but don't crash.
    ret = write_reg(addr, 0xFE, prescale); // Set prescale
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set prescale on PCA @0x%02X", addr);
        return ret;
    }

    ret = write_reg(addr, 0x00, 0xA1); // MODE1 auto-increment, restart
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart PCA @0x%02X", addr);
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // Wait for oscillator
    return ESP_OK;
}


} // namespace hsg_pca9685