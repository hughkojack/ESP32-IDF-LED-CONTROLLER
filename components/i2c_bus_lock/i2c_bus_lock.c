#include "i2c_bus_lock.h"
#include "esp_log.h"

static const char* TAG = "i2c_bus";
static SemaphoreHandle_t s_i2c_bus_mutex;
static i2c_bus_post_recover_cb_t s_post_recover_cb;

static void i2c_bus_notify_recovered(void)
{
    if (s_post_recover_cb) {
        s_post_recover_cb();
    }
}

void i2c_bus_set_post_recover_cb(i2c_bus_post_recover_cb_t cb)
{
    s_post_recover_cb = cb;
}

void i2c_bus_lock_init(void)
{
    if (!s_i2c_bus_mutex) {
        s_i2c_bus_mutex = xSemaphoreCreateMutex();
    }
}

bool i2c_bus_lock(TickType_t timeout_ticks)
{
    if (!s_i2c_bus_mutex) {
        return true;
    }
    return xSemaphoreTake(s_i2c_bus_mutex, timeout_ticks) == pdTRUE;
}

void i2c_bus_unlock(void)
{
    if (s_i2c_bus_mutex) {
        xSemaphoreGive(s_i2c_bus_mutex);
    }
}

esp_err_t i2c_bus_recover(i2c_master_bus_handle_t bus)
{
    if (!bus) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = i2c_master_bus_reset(bus);
    if (ret == ESP_OK) {
        i2c_bus_notify_recovered();
    }
    return ret;
}

esp_err_t i2c_bus_remove_device_safe(i2c_master_bus_handle_t bus,
                                     i2c_master_dev_handle_t dev,
                                     esp_err_t xfer_err)
{
    if (xfer_err != ESP_OK && bus) {
        esp_err_t rr = i2c_master_bus_reset(bus);
        if (rr != ESP_OK) {
            ESP_LOGW(TAG, "bus reset after xfer error (%s) failed: %s",
                     esp_err_to_name(xfer_err), esp_err_to_name(rr));
        } else {
            i2c_bus_notify_recovered();
        }
    }
    if (!dev) {
        return xfer_err;
    }
    esp_err_t rm = i2c_master_bus_rm_device(dev);
    if (rm != ESP_OK) {
        ESP_LOGW(TAG, "rm_device failed (%s) after xfer %s; resetting bus",
                 esp_err_to_name(rm), esp_err_to_name(xfer_err));
        if (bus) {
            i2c_master_bus_reset(bus);
            i2c_bus_notify_recovered();
        }
        return (xfer_err != ESP_OK) ? xfer_err : rm;
    }
    return xfer_err;
}
