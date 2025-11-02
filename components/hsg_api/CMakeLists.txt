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
)
# Embed your HTML file as binary
target_add_binary_data(${COMPONENT_TARGET} "ESP32-POE.html" TEXT)
target_add_binary_data(${COMPONENT_TARGET} favicon.ico BINARY)
target_add_binary_data(${COMPONENT_TARGET} "control.html" TEXT)