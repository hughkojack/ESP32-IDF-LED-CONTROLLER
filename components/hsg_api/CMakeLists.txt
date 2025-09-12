idf_component_register(
    SRCS
        "HSG-API.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_http_server
        esp_netif
        nvs_flash
        driver
        freertos
        cJSON
        app_update
        hsg_outputs
)
# Embed your HTML file as binary
target_add_binary_data(${COMPONENT_TARGET} "ESP32-POE.html" TEXT)