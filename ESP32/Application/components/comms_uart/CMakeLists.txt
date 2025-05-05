idf_component_register(
    SRCS "Source/comms_uart.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos 
)