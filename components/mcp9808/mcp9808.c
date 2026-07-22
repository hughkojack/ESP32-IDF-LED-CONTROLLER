#include "mcp9808.h"
#include "i2c_bus_lock.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "mcp9808"
#define I2C_XFER_TIMEOUT_MS 100
#define I2C_SPEED_HZ 100000

#define MCP9808_REG_TEMP_AMB   0x05U
#define MCP9808_REG_MANUF_ID   0x6BU
#define MCP9808_REG_DEVICE_ID  0x6CU

#define MCP9808_MANUF_ID       0x0054U
#define MCP9808_DEVICE_ID      0x0400U

/* Disabled on all boards until a sensor is installed and probe is re-enabled
 * carefully. Config-page /api/adopt was probing Olimex and crashing the bus. */
#ifndef MCP9808_ENABLE
#define MCP9808_ENABLE 0
#endif

static bool s_installed = false;
static bool s_probe_attempted = false;
static uint8_t s_addr = 0;

void mcp9808_mark_installed(bool installed)
{
    s_installed = installed;
    if (!installed)
        s_addr = 0;
}

bool mcp9808_is_installed(void)
{
    return s_installed;
}

uint8_t mcp9808_get_addr(void)
{
    return s_installed ? s_addr : 0;
}

static esp_err_t mcp9808_transfer(i2c_master_bus_handle_t bus, uint8_t addr,
                                  const uint8_t *wbuf, size_t wlen,
                                  uint8_t *rbuf, size_t rlen)
{
    if (!i2c_bus_lock(pdMS_TO_TICKS(I2C_XFER_TIMEOUT_MS)))
        return ESP_ERR_TIMEOUT;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_SPEED_HZ,
    };
    i2c_master_dev_handle_t dev;
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (ret != ESP_OK) {
        i2c_bus_unlock();
        return ret;
    }

    const TickType_t xfer_timeout = pdMS_TO_TICKS(I2C_XFER_TIMEOUT_MS);
    if (wlen > 0 && rlen > 0)
        ret = i2c_master_transmit_receive(dev, wbuf, wlen, rbuf, rlen, xfer_timeout);
    else if (wlen > 0)
        ret = i2c_master_transmit(dev, wbuf, wlen, xfer_timeout);
    else
        ret = i2c_master_receive(dev, rbuf, rlen, xfer_timeout);

    esp_err_t xfer = ret;
    ret = i2c_bus_remove_device_safe(bus, dev, ret);
    const bool need_recover = i2c_bus_recover_pending() || (xfer == ESP_ERR_TIMEOUT);
    i2c_bus_unlock();
    if (need_recover) {
        i2c_bus_recover(bus);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return ret;
}

static bool mcp9808_verify_ids(i2c_master_bus_handle_t bus, uint8_t addr)
{
    uint8_t reg = MCP9808_REG_MANUF_ID;
    uint8_t manuf[2] = {0};
    if (mcp9808_transfer(bus, addr, &reg, 1, manuf, 2) != ESP_OK)
        return false;
    uint16_t manuf_id = ((uint16_t)manuf[0] << 8) | manuf[1];
    if (manuf_id != MCP9808_MANUF_ID)
        return false;

    reg = MCP9808_REG_DEVICE_ID;
    uint8_t dev[2] = {0};
    if (mcp9808_transfer(bus, addr, &reg, 1, dev, 2) != ESP_OK)
        return false;
    uint16_t device_id = ((uint16_t)dev[0] << 8) | dev[1];
    return (device_id & 0xFF00U) == MCP9808_DEVICE_ID;
}

static bool mcp9808_try_address(i2c_master_bus_handle_t bus, uint8_t addr)
{
    if (mcp9808_verify_ids(bus, addr)) {
        s_addr = addr;
        return true;
    }
    ESP_LOGD(TAG, "No MCP9808 at 0x%02X", addr);
    return false;
}

bool mcp9808_probe(i2c_master_bus_handle_t bus)
{
#if !MCP9808_ENABLE
    (void)bus;
    return false;
#else
    if (!bus)
        return false;
    /* Do not use i2c_master_probe — false ACKs on a shared bus can wedge the driver. */
    return mcp9808_try_address(bus, MCP9808_I2C_ADDR_DEFAULT);
#endif
}

bool mcp9808_ensure_ready(i2c_master_bus_handle_t bus)
{
#if !MCP9808_ENABLE
    (void)bus;
    return false;
#else
    if (s_installed && s_addr != 0)
        return true;
    if (s_probe_attempted)
        return false;
    s_probe_attempted = true;

    if (mcp9808_probe(bus)) {
        mcp9808_mark_installed(true);
        ESP_LOGI(TAG, "MCP9808 @0x%02X ready", s_addr);
        return true;
    }
    mcp9808_mark_installed(false);
    ESP_LOGI(TAG, "MCP9808 not found on I2C");
    return false;
#endif
}

static float mcp9808_raw_to_celsius(uint16_t raw)
{
    raw &= 0x1FFFU;
    if (raw & 0x1000U) {
        raw = (uint16_t)(0x2000U - raw);
        return -(float)raw * 0.0625f;
    }
    return (float)raw * 0.0625f;
}

esp_err_t mcp9808_read_celsius(i2c_master_bus_handle_t bus, float *out_celsius)
{
#if !MCP9808_ENABLE
    (void)bus;
    (void)out_celsius;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!bus || !out_celsius)
        return ESP_ERR_INVALID_ARG;
    if (!mcp9808_ensure_ready(bus))
        return ESP_ERR_NOT_FOUND;

    uint8_t reg = MCP9808_REG_TEMP_AMB;
    uint8_t buf[2] = {0};
    esp_err_t ret = mcp9808_transfer(bus, s_addr, &reg, 1, buf, 2);
    if (ret != ESP_OK)
        return ret;

    uint16_t raw = ((uint16_t)buf[0] << 8) | buf[1];
    *out_celsius = mcp9808_raw_to_celsius(raw);
    return ESP_OK;
#endif
}
