/**
 * DS3231 I2C RTC driver - minimal API for hub time source.
 * Uses existing I2C bus handle; no device handle caching.
 */
#ifndef DS3231_H
#define DS3231_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Default I2C address for DS3231 */
#define DS3231_I2C_ADDR  0x68U
/** Alternative address when AD pin is high */
#define DS3231_I2C_ADDR_ALT 0x69U

/**
 * Probe for DS3231 on the bus (tries 0x68 then 0x69).
 * @param bus I2C master bus handle (already initialized)
 * @return true if device ACKs at 0x68 or 0x69
 */
bool ds3231_probe(i2c_master_bus_handle_t bus);

/**
 * Read current date/time from DS3231 into struct tm.
 * RTC is read as UTC; caller should set TZ if RTC is set to local time.
 * @param bus I2C master bus handle
 * @param out_tm output struct tm (tm_wday, tm_isdst not set by RTC)
 * @return ESP_OK on success
 */
esp_err_t ds3231_get_time(i2c_master_bus_handle_t bus, struct tm *out_tm);

/**
 * Write system time to DS3231 (for NTP sync-back).
 * @param bus I2C master bus handle
 * @param tm time to write (typically from gmtime_r / localtime_r)
 * @return ESP_OK on success
 */
esp_err_t ds3231_set_time(i2c_master_bus_handle_t bus, const struct tm *tm);

/**
 * Read status register (0x0F). Bit 7 = OSF (Oscillator Stop Flag); set when power was lost.
 * @param bus I2C master bus handle
 * @param out_status value of status register (0x00 = time valid, 0x80 = oscillator had stopped)
 * @return ESP_OK on success
 */
esp_err_t ds3231_get_status(i2c_master_bus_handle_t bus, uint8_t *out_status);

#ifdef __cplusplus
}
#endif

#endif /* DS3231_H */
