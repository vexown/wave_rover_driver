idf_component_register(
    SRCS "Source/esp_now_comm.c" "Source/esp_now_comm_callbacks.c"
    INCLUDE_DIRS "Include"
    REQUIRES esp_wifi nvs_flash
)
