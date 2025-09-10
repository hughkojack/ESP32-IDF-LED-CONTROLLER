
idf_component_register(
    SRCS
        "main.cpp"
    INCLUDE_DIRS
        "."
    REQUIRES
        mcp2515          # your component
        driver           # GPIO, I2C, SPI, etc.
        freertos
        esp_event        # event loop used by Wi-Fi
        esp_wifi
        esp_netif
        esp_http_server
        nvs_flash
        esp_eth
        cJSON          # private dependency (not exposed to other components
        hsg_api    
)
