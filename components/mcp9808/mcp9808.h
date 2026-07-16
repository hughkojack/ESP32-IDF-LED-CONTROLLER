/**
 * MCP9808 I2C temperature sensor driver (minimal API for hub board temp).
 */
#ifndef MCP9808_H
#define MCP9808_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Default 7-bit address when A2/A1/A0 are tied to GND */
#define MCP9808_I2C_ADDR_DEFAULT  0x18U
/** First / last address in MCP9808 address-select range */
#define MCP9808_I2C_ADDR_MIN      0x18U
#define MCP9808_I2C_ADDR_MAX      0x1FU

void mcp9808_mark_installed(bool installed);
bool mcp9808_is_installed(void);

/** Address detected at boot (0 if not installed). */
uint8_t mcp9808_get_addr(void);

/**
 * Probe for MCP9808 (ID register read). Rack32: 0x18 only.
 * @return true if manufacturer/device IDs match
 */
bool mcp9808_probe(i2c_master_bus_handle_t bus);

/** Lazy probe on first temperature read (safe to skip at boot). */
bool mcp9808_ensure_ready(i2c_master_bus_handle_t bus);

/**
 * Read ambient temperature in degrees Celsius.
 * @return ESP_OK on success
 */
esp_err_t mcp9808_read_celsius(i2c_master_bus_handle_t bus, float *out_celsius);

#ifdef __cplusplus
}
#endif

#endif /* MCP9808_H */
