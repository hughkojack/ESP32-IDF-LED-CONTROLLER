idf_component_register(
    SRCS "hsg_outputs.cpp" "hsg_pca9685.cpp"
    INCLUDE_DIRS "include"
    REQUIRES hsg_api cJSON driver
)
