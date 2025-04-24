idf_component_register(
    SRCS "Source/motor_control.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos
)