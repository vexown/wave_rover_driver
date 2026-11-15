idf_component_register(
    SRCS "Source/robot_status.c"
    INCLUDE_DIRS "Include"
    REQUIRES esp_common freertos esp_timer esp_wifi web_server
)
