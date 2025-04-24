idf_component_register(
    SRCS "Source/web_server.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos esp_wifi esp_event esp_http_server esp_netif nvs_flash motor_control
)