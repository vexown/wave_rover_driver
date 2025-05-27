idf_component_register(
    SRCS "Source/web_server.c" "Source/HTML_page.c"
    INCLUDE_DIRS "Include"
    REQUIRES driver esp_common freertos esp_wifi esp_event esp_http_server esp_netif nvs_flash motor_control esp_http_server esp_https_ota oled_display
)

# Embed the certificate file
target_add_binary_data(${COMPONENT_TARGET} "certs/apache_ip_cert.crt" TEXT EMBED_IN_RODATA)