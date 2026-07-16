// hsg_pca9685.cpp
#include "hsg_pca9685.h"
#include "i2c_bus_lock.h"
#include "esp_log.h"
#include <math.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "PCA9685";

extern i2c_master_bus_handle_t i2c_bus_handle;

namespace hsg_pca9685 {

static const TickType_t kI2cXferTimeout = pdMS_TO_TICKS(250);
static const int kMaxPcaDevices = 8;

static i2c_master_dev_handle_t s_dev_handles[kMaxPcaDevices] = {};
static uint8_t s_dev_addrs[kMaxPcaDevices] = {};
static size_t s_dev_count = 0;

static void drop_pca_device_cache_locked(void)
{
    for (size_t i = 0; i < s_dev_count; ++i) {
        if (s_dev_handles[i]) {
            esp_err_t rm = i2c_master_bus_rm_device(s_dev_handles[i]);
            if (rm != ESP_OK) {
                ESP_LOGW(TAG, "drop cache: rm_device 0x%02X failed: %s",
                         s_dev_addrs[i], esp_err_to_name(rm));
            }
            s_dev_handles[i] = nullptr;
        }
    }
    s_dev_count = 0;
}

void pca9685_invalidate_bus_devices(void)
{
    if (!i2c_bus_lock(kI2cXferTimeout)) {
        drop_pca_device_cache_locked();
        return;
    }
    drop_pca_device_cache_locked();
    i2c_bus_unlock();
}

static void recover_bus_on_error(esp_err_t err)
{
    if (err == ESP_OK || err == ESP_ERR_INVALID_ARG) {
        return;
    }
    if (i2c_bus_lock(kI2cXferTimeout)) {
        drop_pca_device_cache_locked();
        esp_err_t rr = i2c_bus_recover(i2c_bus_handle);
        if (rr != ESP_OK) {
            ESP_LOGW(TAG, "I2C bus recover failed: %s", esp_err_to_name(rr));
        }
        i2c_bus_unlock();
    } else {
        esp_err_t rr = i2c_bus_recover(i2c_bus_handle);
        if (rr != ESP_OK) {
            ESP_LOGW(TAG, "I2C bus recover failed: %s", esp_err_to_name(rr));
        }
    }
}

static esp_err_t add_pca9685_device(uint8_t addr, i2c_master_dev_handle_t* out_handle)
{
    for (size_t i = 0; i < s_dev_count; ++i) {
        if (s_dev_addrs[i] == addr) {
            *out_handle = s_dev_handles[i];
            return ESP_OK;
        }
    }
    if (s_dev_count >= kMaxPcaDevices) {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = addr;
    dev_cfg.scl_speed_hz = 100000;

    i2c_master_dev_handle_t dev_handle = nullptr;
    esp_err_t ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        return ret;
    }

    s_dev_handles[s_dev_count] = dev_handle;
    s_dev_addrs[s_dev_count] = addr;
    s_dev_count++;
    *out_handle = dev_handle;
    return ESP_OK;
}

static esp_err_t transmit_pwm_locked(uint8_t addr, int channel, uint16_t value)
{
    uint8_t reg = (uint8_t)(0x06 + 4 * channel);
    uint8_t buf[5] = {
        reg, 0x00, 0x00,
        (uint8_t)(value & 0xFF),
        (uint8_t)(value >> 8),
    };

    i2c_master_dev_handle_t dev_handle = nullptr;
    esp_err_t ret = add_pca9685_device(addr, &dev_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    return i2c_master_transmit(dev_handle, buf, sizeof(buf), kI2cXferTimeout);
}

esp_err_t pca9685_write_pwm_value(uint8_t addr, int channel, uint16_t value) {
    if (channel < 0 || channel > 15) return ESP_ERR_INVALID_ARG;
    if (value > 4095) value = 4095;

    esp_err_t last = ESP_FAIL;
    for (int attempt = 0; attempt < 2; ++attempt) {
        if (!i2c_bus_lock(kI2cXferTimeout)) {
            return ESP_ERR_TIMEOUT;
        }
        last = transmit_pwm_locked(addr, channel, value);
        i2c_bus_unlock();
        if (last == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "PWM write PCA@0x%02X ch%d attempt %d failed: %s",
                 addr, channel, attempt + 1, esp_err_to_name(last));
        recover_bus_on_error(last);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return last;
}

static esp_err_t write_reg(uint8_t addr, uint8_t reg, uint8_t val) {
    esp_err_t last = ESP_FAIL;
    for (int attempt = 0; attempt < 2; ++attempt) {
        if (!i2c_bus_lock(kI2cXferTimeout)) {
            ESP_LOGW(TAG, "I2C lock timeout writing reg 0x%02X to PCA @0x%02X", reg, addr);
            return ESP_ERR_TIMEOUT;
        }

        i2c_master_dev_handle_t dev_handle = nullptr;
        esp_err_t ret = add_pca9685_device(addr, &dev_handle);
        if (ret != ESP_OK) {
            i2c_bus_unlock();
            return ret;
        }

        uint8_t buf[2] = { reg, val };
        last = i2c_master_transmit(dev_handle, buf, sizeof(buf), kI2cXferTimeout);
        i2c_bus_unlock();
        if (last == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "I2C write reg 0x%02X to PCA @0x%02X attempt %d failed: %s",
                 reg, addr, attempt + 1, esp_err_to_name(last));
        recover_bus_on_error(last);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return last;
}

esp_err_t pca9685_init(uint8_t addr, int freq_hz) {
    ESP_LOGI(TAG, "PCA9685 @0x%02X init starting...", addr);

    esp_err_t ret = write_reg(addr, 0x00, 0x10);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 @0x%02X did not respond: %s", addr, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "PCA9685 @0x%02X online, configuring %dHz PWM", addr, freq_hz);

    float prescale_val = 25000000.0 / (4096.0 * freq_hz) - 1;
    uint8_t prescale = (uint8_t)floor(prescale_val + 0.5);

    ret = write_reg(addr, 0xFE, prescale);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set prescale on PCA @0x%02X", addr);
        return ret;
    }

    ret = write_reg(addr, 0x00, 0xA1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart PCA @0x%02X", addr);
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_LOGI(TAG, "PCA9685 @0x%02X is online. Freq=%dHz", addr, freq_hz);
    return ESP_OK;
}

} // namespace hsg_pca9685

void hsg_pca9685_invalidate_bus_devices(void)
{
    hsg_pca9685::pca9685_invalidate_bus_devices();
}
