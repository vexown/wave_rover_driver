idf_component_register(
    SRCS "Source/IMU.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos i2c_manager web_server
)