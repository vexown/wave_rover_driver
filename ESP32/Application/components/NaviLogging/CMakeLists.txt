idf_component_register(
    SRCS "Source/NaviLogging.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos esp_wifi esp_netif esp_event web_server Common
)