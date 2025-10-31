idf_component_register(SRCS "src/wifi_manager.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_event esp_netif nvs_flash
                       PRIV_REQUIRES freertos esp_system)
