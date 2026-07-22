#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

void i2c_bus_lock_init(void);
bool i2c_bus_lock(TickType_t timeout_ticks);
void i2c_bus_unlock(void);

/** Optional: called after any I2C bus reset (e.g. drop cached device handles). */
typedef void (*i2c_bus_post_recover_cb_t)(void);
void i2c_bus_set_post_recover_cb(i2c_bus_post_recover_cb_t cb);

/**
 * Bus reset + post-recover CB. Must NOT be called while holding the bus lock.
 */
esp_err_t i2c_bus_recover(i2c_master_bus_handle_t bus);

/**
 * Remove a temporary I2C device after a transfer.
 * Caller must hold the bus lock. Never resets the bus or runs the post-recover CB.
 * If rm_device fails, sets recover-pending only;
 * caller must call i2c_bus_recover() after unlock when i2c_bus_recover_pending().
 */
esp_err_t i2c_bus_remove_device_safe(i2c_master_bus_handle_t bus,
                                     i2c_master_dev_handle_t dev,
                                     esp_err_t xfer_err);

bool i2c_bus_recover_pending(void);

#ifdef __cplusplus
}
#endif
