idf_component_register(
    SRCS
        "HSG-API.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        src
        esp_http_server
        nvs_flash
        cJSON
        app_update
        mqtt
        driver
        esp_netif
        esp_wifi
        i2c_bus_lock
        hsg_outputs
        ds3231
)
# Embed your HTML file as binary (path relative to this component)
target_add_binary_data(${COMPONENT_TARGET} "${CMAKE_CURRENT_SOURCE_DIR}/ESP32-POE.html" TEXT)
target_add_binary_data(${COMPONENT_TARGET} favicon.ico BINARY)
target_add_binary_data(${COMPONENT_TARGET} "control.html" TEXT)