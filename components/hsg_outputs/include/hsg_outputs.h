#pragma once
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize the output manager (loads config from HSG-API)
esp_err_t hsg_outputs_init(i2c_port_t port);
void hsg_outputs_reload_config();

// Set a single logical output to brightness/fade
esp_err_t hsg_outputs_set(int out, int brightness, int fade_ms);

// Set a group by name ("AA", "BB", etc.)
esp_err_t hsg_outputs_set_group(const char* name, const char* state, int fade_ms);


#ifdef __cplusplus
}
#endif