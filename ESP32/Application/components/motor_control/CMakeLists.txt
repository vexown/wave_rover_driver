idf_component_register(
    SRCS "Source/motor_control.c" "Source/roarm_m3_motor_control.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos web_server comms_uart Common esp_now_comm
)