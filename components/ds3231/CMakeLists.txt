idf_component_register(
    SRCS "ds3231.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos log
)
