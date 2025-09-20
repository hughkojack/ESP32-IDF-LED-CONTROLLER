#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// The init and reload functions now take the config as a parameter
esp_err_t hsg_outputs_init(int i2c_port, cJSON* config_json);
esp_err_t hsg_outputs_reload_config(cJSON* config_json);

// Set a single logical output to brightness/fade
esp_err_t hsg_outputs_set(int out, int brightness, int fade_ms);

// Set a group by name ("AA", "BB", etc.)
//esp_err_t hsg_outputs_set_group(const char* name, const char* state, int fade_ms);

/**
 * @brief Gets the physical mapping (PCA9685 address and channel) for a logical output number.
 * @param output The logical output number (1-160).
 * @param addr Pointer to store the I2C address of the PCA9685.
 * @param channel Pointer to store the channel number (0-15) on the PCA9685.
 * @return True if a mapping was found, false otherwise.
 */
bool hsg_outputs_get_mapping(int output, uint8_t *addr, uint8_t *channel);

void hsg_outputs_clear_all();

#ifdef __cplusplus
}
#endif