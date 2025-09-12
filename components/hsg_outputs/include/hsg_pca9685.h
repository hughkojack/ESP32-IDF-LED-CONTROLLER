// hsg_pca9685.h
#pragma once
#include "driver/i2c.h"
#include "esp_err.h"

esp_err_t pca9685_init(i2c_port_t port, uint8_t addr, int freq_hz);
esp_err_t pca9685_set_pwm(i2c_port_t port, uint8_t addr, int channel, int duty, int fade_ms);
