idf_component_register(
    SRCS "Source/i2c_manager.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos 
)