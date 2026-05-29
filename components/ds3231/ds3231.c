/**
 * DS3231 I2C RTC driver - read/write time registers in BCD.
 */
#include "ds3231.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

/* DS3231 time/date registers (BCD) */
#define DS3231_REG_SECONDS  0x00
#define DS3231_REG_MINUTES  0x01
#define DS3231_REG_HOURS    0x02
#define DS3231_REG_DAY      0x03   /* 1-7, user-defined */
#define DS3231_REG_DATE     0x04   /* 1-31 */
#define DS3231_REG_MONTH    0x05   /* 1-12, bit 7 = century */
#define DS3231_REG_YEAR     0x06   /* 00-99 */
#define DS3231_REG_STATUS   0x0F   /* bit 7 = OSF (Oscillator Stop Flag); clear to mark time valid */

static inline uint8_t bcd_to_byte(uint8_t bcd)
{
    return (uint8_t)((bcd >> 4) * 10 + (bcd & 0x0Fu));
}

static inline uint8_t byte_to_bcd(uint8_t byte)
{
    return (uint8_t)(((byte / 10) << 4) | (byte % 10));
}

static esp_err_t ds3231_transfer(i2c_master_bus_handle_t bus, uint8_t addr,
    const uint8_t *wbuf, size_t wlen, uint8_t *rbuf, size_t rlen)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 400000,
    };
    i2c_master_dev_handle_t dev;
    esp_err_t ret = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (ret != ESP_OK) return ret;

    if (wlen > 0 && rlen > 0) {
        ret = i2c_master_transmit_receive(dev, wbuf, wlen, rbuf, rlen, pdMS_TO_TICKS(100));
    } else if (wlen > 0) {
        ret = i2c_master_transmit(dev, wbuf, wlen, pdMS_TO_TICKS(100));
    } else {
        ret = i2c_master_receive(dev, rbuf, rlen, pdMS_TO_TICKS(100));
    }
    i2c_master_bus_rm_device(dev);
    return ret;
}

bool ds3231_probe(i2c_master_bus_handle_t bus)
{
    if (i2c_master_probe(bus, DS3231_I2C_ADDR, pdMS_TO_TICKS(100)) == ESP_OK)
        return true;
    if (i2c_master_probe(bus, DS3231_I2C_ADDR_ALT, pdMS_TO_TICKS(100)) == ESP_OK)
        return true;
    return false;
}

/* Try to read using given address; return ESP_OK and set out_addr if found */
static esp_err_t ds3231_read_regs(i2c_master_bus_handle_t bus, uint8_t addr, uint8_t *regs)
{
    uint8_t reg = DS3231_REG_SECONDS;
    return ds3231_transfer(bus, addr, &reg, 1, regs, 7);
}

/* Determine which I2C address responds (0x68 or 0x69) */
static uint8_t ds3231_resolve_addr(i2c_master_bus_handle_t bus)
{
    uint8_t regs[7];
    if (ds3231_read_regs(bus, DS3231_I2C_ADDR, regs) == ESP_OK)
        return DS3231_I2C_ADDR;
    if (ds3231_read_regs(bus, DS3231_I2C_ADDR_ALT, regs) == ESP_OK)
        return DS3231_I2C_ADDR_ALT;
    return 0;
}

esp_err_t ds3231_get_time(i2c_master_bus_handle_t bus, struct tm *out_tm)
{
    if (!bus || !out_tm) return ESP_ERR_INVALID_ARG;

    uint8_t addr = ds3231_resolve_addr(bus);
    if (addr == 0) return ESP_ERR_NOT_FOUND;

    uint8_t regs[7];
    uint8_t reg = DS3231_REG_SECONDS;
    esp_err_t ret = ds3231_transfer(bus, addr, &reg, 1, regs, 7);
    if (ret != ESP_OK) return ret;

    memset(out_tm, 0, sizeof(*out_tm));
    out_tm->tm_sec  = (int)bcd_to_byte(regs[0] & 0x7Fu);
    out_tm->tm_min  = (int)bcd_to_byte(regs[1]);
    /* Hours: bit 6 = 0 for 24h mode */
    out_tm->tm_hour = (int)bcd_to_byte(regs[2] & 0x3Fu);
    out_tm->tm_mday = (int)bcd_to_byte(regs[4]);
    out_tm->tm_mon  = (int)bcd_to_byte(regs[5] & 0x1Fu) - 1; /* 0-11 */
    int year = (int)bcd_to_byte(regs[6]);
    if (regs[5] & 0x80u) year += 100; /* century bit = 19xx */
    else                 year += 2000;
    out_tm->tm_year = year - 1900;
    /* tm_wday: DS3231 day 1-7 often 1=Sun; set from date or leave 0 */
    out_tm->tm_isdst = -1;

    return ESP_OK;
}

esp_err_t ds3231_set_time(i2c_master_bus_handle_t bus, const struct tm *tm)
{
    if (!bus || !tm) return ESP_ERR_INVALID_ARG;

    uint8_t addr = ds3231_resolve_addr(bus);
    if (addr == 0) return ESP_ERR_NOT_FOUND;

    int year = tm->tm_year + 1900;
    uint8_t century = (year >= 2000) ? 0u : 0x80u;
    uint8_t y = (uint8_t)(year % 100);

    uint8_t buf[8];
    buf[0] = DS3231_REG_SECONDS;
    buf[1] = byte_to_bcd((uint8_t)tm->tm_sec);
    buf[2] = byte_to_bcd((uint8_t)tm->tm_min);
    buf[3] = byte_to_bcd((uint8_t)tm->tm_hour);
    buf[4] = byte_to_bcd((uint8_t)(tm->tm_wday <= 0 ? 1 : (tm->tm_wday + 1))); /* 1-7 */
    buf[5] = byte_to_bcd((uint8_t)tm->tm_mday);
    buf[6] = byte_to_bcd((uint8_t)(tm->tm_mon + 1)) | century;
    buf[7] = byte_to_bcd(y);

    esp_err_t ret = ds3231_transfer(bus, addr, buf, 8, NULL, 0);
    if (ret != ESP_OK) return ret;
    /* Clear OSF (Oscillator Stop Flag) so RTC marks time as valid */
    uint8_t clear_osf[2] = { DS3231_REG_STATUS, 0x00 };
    return ds3231_transfer(bus, addr, clear_osf, 2, NULL, 0);
}

esp_err_t ds3231_get_status(i2c_master_bus_handle_t bus, uint8_t *out_status)
{
    if (!bus || !out_status) return ESP_ERR_INVALID_ARG;
    uint8_t addr = ds3231_resolve_addr(bus);
    if (addr == 0) return ESP_ERR_NOT_FOUND;
    uint8_t reg = DS3231_REG_STATUS;
    return ds3231_transfer(bus, addr, &reg, 1, out_status, 1);
}
