idf_component_register(
    SRCS "Source/Common.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos comms_uart
)