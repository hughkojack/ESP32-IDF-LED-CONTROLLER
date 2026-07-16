idf_component_register(
    SRCS "mcp9808.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos i2c_bus_lock log
)
