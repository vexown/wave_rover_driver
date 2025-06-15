idf_component_register(
    SRCS "Source/oled_display.c" "Source/font8x8_basic.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos esp_lcd i2c_manager
)