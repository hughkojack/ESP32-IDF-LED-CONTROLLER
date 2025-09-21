// hsg_pca9685.h
#pragma once
#include "driver/i2c.h"
#include "esp_err.h"

namespace hsg_pca9685 {

    esp_err_t pca9685_write_pwm_value(uint8_t addr, int channel, uint16_t value);
    esp_err_t pca9685_init(uint8_t addr, int freq_hz = 1000);

} // namespace hsg_pca9685

