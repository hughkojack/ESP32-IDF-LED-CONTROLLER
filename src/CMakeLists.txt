
option(BOARD_OLIMEX_POE "Build for Olimex ESP32-POE board" ON)
option(BOARD_RACK32 "Build for Rack32 board with W5500" OFF)

idf_component_register(
    SRCS
        "main.cpp"
    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
        hsg_api
        hsg_outputs
        esp32-mcp2515
        mqtt
        driver
        esp_wifi
        esp_netif
        esp_http_server
        nvs_flash
        esp_eth
        cJSON
        freertos
        esp_event
)

if(BOARD_OLIMEX_POE)
    target_compile_definitions(${COMPONENT_TARGET} PRIVATE "BOARD_OLIMEX_POE")
elseif(BOARD_RACK32)
    target_compile_definitions(${COMPONENT_TARGET} PRIVATE "BOARD_RACK32")
endif()